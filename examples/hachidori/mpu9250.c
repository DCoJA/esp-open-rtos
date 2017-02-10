/* SPI driver for MPU9250.
   MPU9250 is a multi-chip module with 2 dies.  One die implements
   accelerometer/gyroscope and other die is AK8963 magnetometer.
   AK8963 is connected with I2C as a slave device.  MPU9250 has
   registers which make I2C accesses of AK8963 enable.
*/

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

#include "b3packet.h"

#include "MadgwickAHRS.h"
#include "kfacc.h"

#include "pwm.h"
#include "battery.h"

/* MPU9250 */
#define MPU9250_ID	0x71

/* MPU9250 registers */
#define SMPLRT_DIV	0x19
#define MPU_CONFIG	0x1A
#define GYRO_CONFIG	0x1B
#define ACCEL_CONFIG	0x1C

#define I2C_MST_CTRL	0x24
#define I2C_SLV0_ADDR	0x25
#define I2C_SLV0_REG	0x26
#define I2C_SLV0_CTRL	0x27

#define I2C_SLV4_CTRL	0x34

#define INT_PIN_CFG	0x37
#define INT_ENABLE	0x38
#define INT_STATUS	0x3A

#define ACCEL_XOUT_H	0x3B
#define ACCEL_XOUT_L	0x3C
#define ACCEL_YOUT_H	0x3D
#define ACCEL_YOUT_L	0x3E
#define ACCEL_ZOUT_H	0x3F
#define ACCEL_ZOUT_L	0x40
#define TEMP_OUT_H	0x41
#define TEMP_OUT_L	0x42
#define GYRO_XOUT_H	0x43
#define GYRO_XOUT_L	0x44
#define GYRO_YOUT_H	0x45
#define GYRO_YOUT_L	0x46
#define GYRO_ZOUT_H	0x47
#define GYRO_ZOUT_L	0x48

#define EXT_SENS_DATA_00 0x49
#define I2C_SLV0_DO	0x63
#define I2C_MST_DELAY_CTRL 0x67

#define USER_CTRL	0x6A
#define PWR_MGMT_1	0x6B
#define PWR_MGMT_2	0x6C

#define WHO_IM_I	0x75

/* AK8963 */
#define AK8963_I2C_ADDR	0x0c
#define AK8963_ID	0x48

/* AK8963 registers */
#define AK8963_WIA	0x00
#define AK8963_HXL	0x03
#define AK8963_CNTL1	0x0A
#define AK8963_CNTL2	0x0B
#define AK8963_ASAX	0x10

#define GRAVITY_MSS     9.80665f
// MPU9250 accelerometer scaling for 16g range
#define MPU9250_ACCEL_SCALE_1G    (GRAVITY_MSS / 2048.0f)

/*
 *  PS-MPU-9250A-00.pdf, page 8, lists LSB sensitivity of
 *  gyro as 16.4 LSB/DPS at scale factor of +/- 2000dps (FS_SEL==3)
 */
static const float GYRO_SCALE = 0.0174532f / 16.4f;

#define AK8963_MILLIGAUSS_SCALE 10.0f
static const float ADC_16BIT_RESOLUTION = 0.15f;

// MPU9250 IMU data are big endian
#define be16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx] << 8) | v[2*idx+1]))
// AK8963 data are little endian
#define le16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx+1] << 8) | v[2*idx]))

static uint8_t mpu9250_read(uint8_t reg)
{
    uint16_t in = (uint16_t)(0x80 | reg) << 8;
    uint16_t out = spi_transfer_16 (1, in);
    return out & 0xff;
}

static void mpu9250_write(uint8_t reg, uint8_t val)
{
    uint16_t in = ((uint16_t)(reg) << 8) | val;
    spi_transfer_16 (1, in);
}
    
static bool mpu9250_ready(void)
{
    uint8_t val = mpu9250_read(INT_STATUS);
    return (val & 1);
}

struct sample {
    uint8_t cmd;
    uint8_t int_status;
    uint8_t d[14];
};

static bool mpu9250_read_sample(struct sample *rx)
{
    static struct sample tx;
    tx.cmd = 0x80 | INT_STATUS;
    return (spi_transfer(1, &tx, rx, sizeof(tx), SPI_8BIT) == sizeof(tx));
}

static void mpu9250_start(void)
{
    mpu9250_write(PWR_MGMT_2, 0);
    vTaskDelay(1/portTICK_PERIOD_MS);

    // No LPF
    mpu9250_write(MPU_CONFIG, 0);
    vTaskDelay(1/portTICK_PERIOD_MS);

    // Sample rate 1000Hz
    mpu9250_write(SMPLRT_DIV, 0);
    vTaskDelay(1/portTICK_PERIOD_MS);

    // Gyro 2000dps
    mpu9250_write(GYRO_CONFIG, 3<<3);
    vTaskDelay(1/portTICK_PERIOD_MS);

    // Accel full scale 16g
    mpu9250_write(ACCEL_CONFIG, 3<<3);
    vTaskDelay(1/portTICK_PERIOD_MS);

    // INT enable on RDY
    mpu9250_write(INT_ENABLE, 1);
    vTaskDelay(1/portTICK_PERIOD_MS);

    uint8_t val = mpu9250_read(INT_PIN_CFG);
    val |= 0x30;
    mpu9250_write(INT_PIN_CFG, val);
    vTaskDelay(1/portTICK_PERIOD_MS);
}

static void slv0_readn(uint8_t reg, uint8_t size)
{
    mpu9250_write(I2C_SLV0_CTRL, 0);
    mpu9250_write(I2C_SLV0_ADDR, 0x80 | AK8963_I2C_ADDR);
    mpu9250_write(I2C_SLV0_REG, reg);
    mpu9250_write(I2C_SLV0_CTRL, 0x80 | size);
}

static void slv0_write1(uint8_t reg, uint8_t out)
{
    mpu9250_write(I2C_SLV0_CTRL, 0);
    mpu9250_write(I2C_SLV0_DO, out);
    mpu9250_write(I2C_SLV0_ADDR, AK8963_I2C_ADDR);
    mpu9250_write(I2C_SLV0_REG, reg);
    mpu9250_write(I2C_SLV0_CTRL, 0x80 | 1);
}

static uint8_t ak8963_read(uint8_t reg)
{
    slv0_readn(reg, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    uint8_t rv = mpu9250_read(EXT_SENS_DATA_00);

    mpu9250_write(I2C_SLV0_CTRL, 0);
    return rv;
}

static void ak8963_write(uint8_t reg, uint8_t val)
{
    slv0_write1(reg, val);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    mpu9250_write(I2C_SLV0_CTRL, 0);
}

struct ak_sample {
    uint8_t cmd;
    uint8_t d[6];
    uint8_t st2;
};

static void ak8963_read_sample_start(void)
{
    slv0_readn(AK8963_HXL, 7);
}

static bool ak8963_read_sample(struct ak_sample *rx)
{
    static struct ak_sample tx;
    tx.cmd = 0x80 | EXT_SENS_DATA_00;
    uint8_t n = spi_transfer(1, &tx, rx, sizeof(tx), SPI_8BIT);

    mpu9250_write(I2C_SLV0_CTRL, 0);
    return (n == sizeof(tx));
}

struct ak_asa {
    uint8_t cmd;
    uint8_t a[3];
};

static bool ak8963_read_asa(struct ak_asa *rx)
{
    slv0_readn(AK8963_ASAX, 3);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    static struct ak_asa tx;
    tx.cmd = 0x80 | EXT_SENS_DATA_00;
    uint8_t n = spi_transfer(1, &tx, rx, sizeof(tx), SPI_8BIT);

    mpu9250_write(I2C_SLV0_CTRL, 0);
    return (n == sizeof(tx));
}

static struct ak_asa ak8963_asa;
static float ak8963_calib[3];

static void ak8963_start(void)
{
    // Reset
    // ak8963_write(AK8963_CNTL2, 0x01);

    // Calibrate - fuse, 16-bit adc
    ak8963_write(AK8963_CNTL1, 0x1f);
    ak8963_read_asa(&ak8963_asa);

    for (int i = 0; i < 3; i++) {
        float data = ak8963_asa.a[i];
        // factory sensitivity
        ak8963_calib[i] = ((data - 128) / 256 + 1);
        // adjust by ADC sensitivity and convert to milligauss
        ak8963_calib[i] *= ADC_16BIT_RESOLUTION * AK8963_MILLIGAUSS_SCALE;
    }

    // Setup mode - continuous mode 2, 16-bit adc
    ak8963_write(AK8963_CNTL1, 0x16);
    // Start measurement
}

#if DISARM_ON_INVERSION
static int maybe_inverted;
#endif
bool maybe_landed = true;

extern SemaphoreHandle_t send_sem;

void imu_task(void *pvParameters)
{
    uint8_t rv;

    //
    vTaskDelay(1000/portTICK_PERIOD_MS);
       
    if (!spi_init(1, SPI_MODE0, SPI_FREQ_DIV_1M, true, SPI_BIG_ENDIAN, false)) {
        printf("Failed spi_init\n");
    }

    rv = mpu9250_read(WHO_IM_I);
    if (rv != MPU9250_ID) {
        printf("Wrong id: %02x\n", rv);
    }

    uint8_t tries;
    for (tries = 0; tries < 5; tries++) {
        // Disable master I2C here
        if ((rv = mpu9250_read(USER_CTRL)) & (1<<5)) {
            mpu9250_write(USER_CTRL, rv &~ (1<<5));
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        // Reset
        mpu9250_write(PWR_MGMT_1, 0x80);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        // Disable I2C interface
        mpu9250_write(USER_CTRL, 0x10);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        // Wake up with appropriate clock
        mpu9250_write(PWR_MGMT_1, 0x03);
        vTaskDelay(5 / portTICK_PERIOD_MS);
        if (mpu9250_read(PWR_MGMT_1) == 0x03)
            break;

        vTaskDelay(10 / portTICK_PERIOD_MS);
        if (mpu9250_ready())
            break;
    }

    if (tries == 5) {
        printf("Failed to boot MPU9250 5 times");
    }

    mpu9250_start();

    // Configure slaves
    // Set I2C_MST_EN, MST_P_NSR and set bus speed to 400kHz
    rv = mpu9250_read(USER_CTRL);
    mpu9250_write(USER_CTRL, rv | (1<<5));
    mpu9250_write(I2C_MST_CTRL, (1<<4)|13);
    // Sample rate 100Hz
    mpu9250_write(I2C_SLV4_CTRL, 9);
    mpu9250_write(I2C_MST_DELAY_CTRL, 0x0f);

    rv = ak8963_read(AK8963_WIA);
    if (rv != AK8963_ID) {
        printf("Wrong id: %02x\n", rv);
    }

    ak8963_start();

    ak8963_read_sample_start();
    vTaskDelay(10/portTICK_PERIOD_MS);

    struct sample rx;
    struct ak_sample akrx;
    struct B3packet pkt;
    int count = 0;
    int fscount = 0;
    float gx, gy, gz, ax, ay, az, mx, my, mz;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1/portTICK_PERIOD_MS);
        if (!mpu9250_ready())
            continue;

        if (low_battery) {
            // Sleep
            mpu9250_write(PWR_MGMT_1, 0x40);
            printf("low_battery: stop imu_task\n");
            vTaskSuspend(NULL);
        }

        mpu9250_read_sample(&rx);

        // adjust and serialize floats into packet bytes
        union { float f; uint8_t bytes[sizeof(float)];} ux, uy, uz;
        ux.f = ((float)be16_val(rx.d, 0)) * MPU9250_ACCEL_SCALE_1G;
        uy.f = ((float)be16_val(rx.d, 1)) * MPU9250_ACCEL_SCALE_1G;
        uz.f = ((float)be16_val(rx.d, 2)) * MPU9250_ACCEL_SCALE_1G;
        ax = ux.f; ay = uy.f; az = uz.f;
        memcpy(&pkt.data[0], ux.bytes, sizeof(ux));
        memcpy(&pkt.data[4], uy.bytes, sizeof(uy));
        memcpy(&pkt.data[8], uz.bytes, sizeof(uz));
        ux.f = ((float)be16_val(rx.d, 4)) * GYRO_SCALE;
        uy.f = ((float)be16_val(rx.d, 5)) * GYRO_SCALE;
        uz.f = ((float)be16_val(rx.d, 6)) * GYRO_SCALE;
        gx = ux.f; gy = uy.f; gz = uz.f;
        memcpy(&pkt.data[12], ux.bytes, sizeof(ux));
        memcpy(&pkt.data[16], uy.bytes, sizeof(uy));
        memcpy(&pkt.data[20], uz.bytes, sizeof(uz));

        xSemaphoreTake(send_sem, portMAX_DELAY);
        pkt.head = B3HEADER;
        pkt.tos = TOS_IMU;
        int n = send((int)pvParameters, &pkt, sizeof(pkt), 0);
        if (n < 0) {
        }
        xSemaphoreGive(send_sem);

        if ((count++ % 10) == 0) {
            ak8963_read_sample(&akrx);
            // trigger next sampling of ak8963
            ak8963_read_sample_start();

            // skip if overflow
            if (akrx.st2 & 0x08) {
                continue;
            }

            // adjust and serialize floats into packet bytes
            union { float f; uint8_t bytes[sizeof(float)];} ux, uy, uz;
            ux.f = ((float)le16_val(akrx.d, 0)) * ak8963_calib[0];
            uy.f = ((float)le16_val(akrx.d, 1)) * ak8963_calib[1];
            uz.f = ((float)le16_val(akrx.d, 2)) * ak8963_calib[2];
            mx = ux.f; my = uy.f; mz = uz.f;
            memcpy(&pkt.data[0], ux.bytes, sizeof(ux));
            memcpy(&pkt.data[4], uy.bytes, sizeof(uy));
            memcpy(&pkt.data[8], uz.bytes, sizeof(uz));

            xSemaphoreTake(send_sem, portMAX_DELAY);
            pkt.head = B3HEADER;
            pkt.tos = TOS_MAG;
            int n = send((int)pvParameters, &pkt, sizeof(pkt), 0);
            if (n < 0) {
            }
            xSemaphoreGive(send_sem);

            if (prepare_failsafe) {
                beta = (fscount++ < 1000) ? 2.0f : 0.2f;
                MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, my, mx, -mz);
                KFACCupdate(ax, ay, az);
            } else {
                fscount = 0;
            }
#if DISARM_ON_INVERSION
            if (az < -GRAVITY_MSS * 0.6) {
                if(++maybe_inverted > INVERSION_WM) {
                    if (in_arm) {
                        in_arm = false;
                    }
                }
            } else {
                maybe_inverted = 0;
            }
#endif
            if ((ax < 0.8 && ax > -0.8)
                && (ay < 0.8 && ay > -0.8)
                && (az < GRAVITY_MSS + 0.6 && az > GRAVITY_MSS - 0.6)
                && (gx < 0.05 && gx > -0.05)
                && (gy < 0.05 && gy > -0.05)
                && (gz < 0.05 && gz > -0.05)) {
                maybe_landed = true;
            } else {
                maybe_landed = false;
            }
        }
    }
}
