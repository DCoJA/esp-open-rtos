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

#include "battery.h"
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
extern SemaphoreHandle_t send_sem;

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

    // # of average 64, VCT 140us, CCT 140us, continuous
    ina226_write(INA226_CONFIG, 0x4000 | (3 << 9) | (7 << 0));
    // Set full scall
    ina226_write(INA226_CALIB, INA226_CALIB_VALUE);

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

bool low_battery = false;

void bat_task(void *pvParameters)
{
    bool power_monitor = ina226_init();

    if (!power_monitor) {
        printf ("no working INA226 I2C\n");
        vTaskSuspend(NULL);
    }

    struct B3packet pkt;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, 100/portTICK_PERIOD_MS);

        union { float f; uint8_t bytes[sizeof(float)]; } voltage;
#if defined (USE_ESP_TOUT)
        // Also send TOUT voltage
        voltage.f = sdk_system_adc_read()/1024.0 * 19;
        // printf("voltage %f\n", voltage.f);
#else
        voltage.f = 0.0f;
#endif
        memcpy (&pkt.data[12], voltage.bytes, sizeof(voltage));

        union { float f; uint8_t bytes[sizeof(float)]; } vbus, curr;
        uint16_t v, c;
        ina226_read_sample (&c, &v);
        vbus.f = (float)v * INA226_VBUS_COEFF;
        curr.f = (float)c * INA226_CURR_COEFF;
        memcpy (&pkt.data[0], vbus.bytes, sizeof(vbus));
        memcpy (&pkt.data[4], curr.bytes, sizeof(curr));
        if (vbus.f < LOW_BATTERY_WM) {
            low_battery = true;
        }

        // Send it
        xSemaphoreTake(send_sem, portMAX_DELAY);
        pkt.head = B3HEADER;
        pkt.tos = TOS_BAT;
        int n = send((int)pvParameters, &pkt, sizeof(pkt), 0);
        if (n < 0) {
        }
        xSemaphoreGive(send_sem);
    }
}

