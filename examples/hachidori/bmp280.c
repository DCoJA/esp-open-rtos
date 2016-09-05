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

// oversampling 3 gives 14ms conversion time
#define OVERSAMPLING 3

// BMP280 dig data are little endian
#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx+1] << 8) | v[2*idx]))
#define uint16_val(v, idx)(((uint16_t)v[2*idx+1] << 8) | v[2*idx])

extern xSemaphoreHandle i2c_sem;
extern xSemaphoreHandle send_sem;

static uint8_t bmp280_calib_buff[BMP280_CALIB_SIZE];
// Internal calibration registers
static uint16_t dig_T1;
static int16_t dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t dig_P2, dig_P3, dig_P4, dig_P5;
static int16_t dig_P6, dig_P7, dig_P8, dig_P9;

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

    // reset and wait to start up
    bmp280_write(BMP280_REG_RESET, 0xb6);
    while (bmp280_read(BMP280_REG_STATUS) & 1)
        ;

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
 
    xSemaphoreGive(i2c_sem);
}

// Calculate Temperature and Pressure in real units.
// See Datasheet page 22 for this formulas calculations.
static void calculate(int32_t adc_P, int32_t adc_T, uint8_t *pp, uint8_t *tp)
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
    return;
}

static bool bmp280_read_sample(uint8_t *buf)
{
    xSemaphoreTake(i2c_sem, portMAX_DELAY);
    bmp280_readn(BMP280_REG_RAW, buf, BMP280_RAW_SIZE);
    xSemaphoreGive(i2c_sem);

    // get ADC values
    int32_t adc_P = (int32_t)(buf[0] << 12 | buf[1] << 4 | buf[2] >> 4);
    int32_t adc_T = (int32_t)(buf[3] << 12 | buf[4] << 4 | buf[5] >> 4);
    // calculate pressure and tempetature
    calculate(adc_P, adc_T, &buf[0], &buf[4]);

    return true;
}

void baro_task(void *pvParameters)
{
    bmp280_init();

    struct LRpacket pkt;
    portTickType xLastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, 20/portTICK_RATE_MS);
        bmp280_read_sample(pkt.data);
        // Send it
        xSemaphoreTake(send_sem, portMAX_DELAY);
        pkt.head = 0xD3;
        pkt.tos = TOS_BARO;
        int n = send((int)pvParameters, &pkt, sizeof(pkt), 0);
        if (n < 0) {
        }
        xSemaphoreGive(send_sem);
    }
}
