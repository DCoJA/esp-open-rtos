#include <stdio.h>
#include <string.h>

#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "esp/spi.h"

#include "FreeRTOS.h"
#include "task.h"
#include <semphr.h>

#include "i2c/i2c.h"

#include "ina226.h"

// INA226
#define INA226_ADDRESS		0x44

#define INA226_MID		0xfe
#define INA226_DID		0xff
#define INA226_CONFIG		0x00
#define INA226_SHUNT		0x01
#define INA226_VBUS		0x02
#define INA226_POWER		0x03
#define INA226_CURR		0x04
#define INA226_CALIB		0x05

#define INA226_DIE_ID		0x2260

extern SemaphoreHandle_t i2c_sem;

#ifdef INA226_I2C
static uint16_t ina226_read(uint8_t reg)
{
    uint8_t d[2];
    uint16_t rv = 0;
    if (!i2c_slave_read(INA226_ADDRESS, reg, d, 2)) {
        rv = 0;
    } else {
        rv = ((uint16_t)d[0] << 8) | d[1];
    }
    return rv;
}

static void ina226_write(uint8_t reg, uint16_t val)
{
    uint8_t d[] = { reg, val >> 8, val & 0xff };
    i2c_slave_write(INA226_ADDRESS, d, 3);
}

bool ina226_init(void)
{

    xSemaphoreTake(i2c_sem, portMAX_DELAY);

    // Try to read Die ID
    uint16_t id = ina226_read(INA226_DID);
    if (id != 0x2260) {
        printf ("no working INA226 on I2C\n");
        xSemaphoreGive(i2c_sem);
        return false;
    }

    // Default operation mode
    ina226_write(INA226_CONFIG, 0x4127);
    // Set full scall
    ina226_write(INA226_CALIB, 0x0a00);

    xSemaphoreGive(i2c_sem);
    return true;
}

bool ina226_read_sample(uint16_t *curr, uint16_t *vbus)
{
    xSemaphoreTake(i2c_sem, portMAX_DELAY);

    // get ADC values
    *curr = ina226_read(INA226_CURR);
    *vbus = ina226_read(INA226_VBUS);

    xSemaphoreGive(i2c_sem);
    //printf("ina226: v %d i %d\n", *vbus, *curr);

    return true;
}
#endif
