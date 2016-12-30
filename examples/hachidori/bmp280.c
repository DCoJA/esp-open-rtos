#include <stdio.h>
#include <string.h>

#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "esp/spi.h"

#include "FreeRTOS.h"
#include "task.h"
#include <semphr.h>

#include "lwip/err.h"
#include "lwip/sockets.h"

#include "i2c/i2c.h"

#include "lrpacket.h"

#if defined (INA226_I2C)
#include "ina226.h"
#endif

// BMP/BME280
#define BMP280_ADDRESS		0x77

#define BMP280_CHIP_ID		0x58
#define BME280_CHIP_ID		0x60

#define BMP280_REG_CALIB0	0x88
#define BMP280_CALIB_SIZE	(0xa0-0x88)
#define BMP280_REG_ID		0xd0
#define BMP280_REG_RESET	0xe0
#define BMP280_REG_STATUS	0xf3
#define BMP280_REG_CTRL		0xf4
#define BMP280_REG_CONFIG	0xf5
#define BMP280_REG_RAW		0xf7
#define BMP280_RAW_SIZE		(0xfd-0xf7)

#define BME280_REG_CALIB1	0xa1
#define BME280_REG_CALIB2	0xe1
#define BME280_CALIB2_SIZE	(0xe8-0xe1)
#define BME280_REG_HUM		0xfd
#define BME280_HUM_SIZE		(0xff-0xfd)
#define BME280_REG_CTRLHUM	0xf2

// oversampling 3 gives 14ms conversion time
#define OVERSAMPLING 3

// BMP280 dig data are little endian
#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx+1] << 8) | v[2*idx]))
#define uint16_val(v, idx)(((uint16_t)v[2*idx+1] << 8) | v[2*idx])

extern SemaphoreHandle_t i2c_sem;
extern SemaphoreHandle_t send_sem;

static bool enable_hum = false;

static uint8_t bmp280_calib_buff[BMP280_CALIB_SIZE];
static uint8_t bme280_calib2_buff[BME280_CALIB2_SIZE];
// Internal calibration registers
static uint16_t dig_T1;
static int16_t dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t dig_P2, dig_P3, dig_P4, dig_P5;
static int16_t dig_P6, dig_P7, dig_P8, dig_P9;
static uint8_t dig_H1, dig_H3;
static int16_t dig_H2, dig_H4, dig_H5;
static int8_t dig_H6;

static uint8_t bmp280_read(uint8_t reg)
{
    uint8_t rv = 0;
    if (!i2c_slave_read(BMP280_ADDRESS, reg, &rv, 1)) {
        rv = 0;
    }
    return rv;
}

static bool bmp280_write(uint8_t reg, uint8_t val)
{
    uint8_t d[] = { reg, val };

    return i2c_slave_write(BMP280_ADDRESS, d, 2);
}

static bool bmp280_readn(uint8_t reg, uint8_t *buf, size_t len)
{
    if (!i2c_slave_read(BMP280_ADDRESS, reg, buf, len)) {
        return false;
    }
    return true;
}

static void bmp280_init(void)
{

    xSemaphoreTake(i2c_sem, portMAX_DELAY);

    uint8_t id = bmp280_read(BMP280_REG_ID);
    if (id != BMP280_CHIP_ID && id != BME280_CHIP_ID) {
        printf("BMP280 bad id value %02x\n", id);
    }

    if (id == BME280_CHIP_ID) {
        enable_hum = true;
    }

    // reset and wait to start up
    bmp280_write(BMP280_REG_RESET, 0xb6);
    while (bmp280_read(BMP280_REG_STATUS) & 1)
        ;

    if (enable_hum) {
        // osrs_h = 1  This should be prior to setting REG_CTRL(ctrl_meas).
        // See datasheet 5.4.3 and 5.4.4.
        bmp280_write(BME280_REG_CTRLHUM, 1);
    }
    // osrs_t = 1, osrs_p = OVERSAMPLING, mode = 3(cyclic)
    bmp280_write(BMP280_REG_CTRL, (1 << 5) | (OVERSAMPLING << 2) | 3);
    // t_sb = 0, filter = 5
    bmp280_write(BMP280_REG_CONFIG, (0 << 5) | (4 << 2));

    // Read calibration data
    bmp280_readn(BMP280_REG_CALIB0, bmp280_calib_buff, BMP280_CALIB_SIZE);
    uint8_t *d = bmp280_calib_buff;
    dig_T1 = uint16_val(d, 0);
    dig_T2 = int16_val(d, 1);
    dig_T3 = int16_val(d, 2);
    dig_P1 = uint16_val(d, 3);
    dig_P2 = int16_val(d, 4);
    dig_P3 = int16_val(d, 5);
    dig_P4 = int16_val(d, 6);
    dig_P5 = int16_val(d, 7);
    dig_P6 = int16_val(d, 8);
    dig_P7 = int16_val(d, 9);
    dig_P8 = int16_val(d, 10);
    dig_P9 = int16_val(d, 11);

    if (enable_hum) {
        bmp280_readn(BME280_REG_CALIB1, &dig_H1, sizeof(dig_H1));
        bmp280_readn(BME280_REG_CALIB2, bme280_calib2_buff, BME280_CALIB2_SIZE);
        uint8_t *d = bme280_calib2_buff;
        dig_H2 = int16_val(d, 0);
        dig_H3 = d[2];
        dig_H4 = (int16_t)((d[4] & 0x0f) | d[3] << 4);
        dig_H5 = (int16_t)(((d[4] >> 4) & 0x0f) | d[5] << 4);
        dig_H6 = (int8_t)d[6];
    }

    xSemaphoreGive(i2c_sem);
}

// Calculate Temperature and Pressure in real units.
// See Datasheet page 22 for this formulas calculations.
static void calculate(int32_t adc_P, int32_t adc_T, int32_t adc_H,
                      uint8_t *pp, uint8_t *tp, uint8_t *hp)
{
    int32_t t_fine;
    int32_t tv1, tv2;
    union { float f; uint8_t bytes[sizeof(float)]; } press, temp;

    // Temperature calculations
    tv1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1)))
	   * ((int32_t)dig_T2)) >> 11;
    tv2 = (((((adc_T >> 4) - ((int32_t)dig_T1))
	     * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12)
	   * ((int32_t)dig_T3)) >> 14;
    t_fine = tv1 + tv2;
    temp.f = ((t_fine * 5 + 128) >> 8)/100.0;
    memcpy (tp, temp.bytes, sizeof(temp));

    // Pressure calculations
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8)
      + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47)+var1)) * ((int64_t)dig_P1) >> 33;
    // avoid exception caused by division by zero
    if (var1 == 0) {
        press.f = 0;
        memcpy (pp, press.bytes, sizeof(press));
	return;
    }

    p = 1048576-adc_P;
    p = (((p << 31)-var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    press.f = ((uint32_t) p)/256.0;
    memcpy (pp, press.bytes, sizeof(press));
    // printf("%f %f\n", press.f, temp.f);

    if (!enable_hum) {
        return;
    }

    // Returns humidity in %RH as unsigned 32 bit integer in Q22.10
    // format (22 integer and 10 fractional bits).
    // Output value of 47445 represents 47445/1024 = 46.333 %RH
    int32_t h, v_x1_u32r;
    union { float f; uint8_t bytes[sizeof(float)]; } hum;

    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20)
                    - (((int32_t)dig_H5) * v_x1_u32r))
                   + ((int32_t)16384)) >> 15)
                 * (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10)
                        * (((v_x1_u32r *((int32_t)dig_H3)) >> 11)
                           + ((int32_t)32768))) >> 10)
                      + ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7)
                               * ((int32_t)dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    h = (uint32_t)(v_x1_u32r>>12);
    hum.f = h/1024.0;
    memcpy (hp, hum.bytes, sizeof(hum));
    // printf("%f\n", hum.f);

    return;
}

static bool bmp280_read_sample(uint8_t *buf)
{
    xSemaphoreTake(i2c_sem, portMAX_DELAY);
    bmp280_readn(BMP280_REG_RAW, buf,
                 BMP280_RAW_SIZE + (enable_hum ? BME280_HUM_SIZE : 0));
    xSemaphoreGive(i2c_sem);

    // get ADC values
    int32_t adc_P = (int32_t)(buf[0] << 12 | buf[1] << 4 | buf[2] >> 4);
    int32_t adc_T = (int32_t)(buf[3] << 12 | buf[4] << 4 | buf[5] >> 4);
    int32_t adc_H = 0;
    if (enable_hum) {
        adc_H = (int32_t)(buf[6] << 8 | buf[7]);
        // printf("adc_H: %d\n", adc_H);
    }

    // calculate pressure, tempetature and humidity
    calculate(adc_P, adc_T, adc_H, &buf[0], &buf[4], &buf[8]);

    return true;
}

void baro_task(void *pvParameters)
{
    bmp280_init();

    bool power_monitor = false;
#if defined (INA226_I2C)
    power_monitor = ina226_init();
#endif

    int count = 0;
    struct LRpacket pkt;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, 20/portTICK_PERIOD_MS);
        bmp280_read_sample(pkt.data);

        // Also send TOUT voltage
        union { float f; uint8_t bytes[sizeof(float)]; } voltage;
#if defined (INA226_I2C)
        union { float f; uint8_t bytes[sizeof(float)]; } vbus, curr;
#endif
        if (count == 0) {
            // update every second. 1/19 ATT
            voltage.f = sdk_system_adc_read()/1024.0 * 19;
            // printf("voltage %f\n", voltage.f);

            if (power_monitor) {
#if defined (INA226_I2C)
                uint16_t v, c;
                ina226_read_sample (&c, &v);
                vbus.f = (float)v * INA226_VBUS_COEFF;
                curr.f = (float)c * INA226_CURR_COEFF;
                memcpy (&pkt.data[16], vbus.bytes, sizeof(vbus));
                memcpy (&pkt.data[20], curr.bytes, sizeof(curr));
#endif
            }
        }
        if (++count == 50) {
            count = 0;
        }
        memcpy (&pkt.data[12], voltage.bytes, sizeof(voltage));
        // Send it
        xSemaphoreTake(send_sem, portMAX_DELAY);
        pkt.head = LRHEADER;
        pkt.tos = TOS_BARO;
        int n = send((int)pvParameters, &pkt, sizeof(pkt), 0);
        if (n < 0) {
        }
        xSemaphoreGive(send_sem);
    }
}
