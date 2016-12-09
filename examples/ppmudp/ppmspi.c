/* SPI driver for LPC8xx SPI
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

#define READY           0x7f
#define CLEAR_READY     0x7e

static uint8_t lpc_read(uint8_t reg)
{
    uint8_t in = (0x80 | reg);
    (void)spi_transfer_8 (1, in);
    // dummy read for lpc
    (void)spi_transfer_8 (1, in);
    uint8_t out = spi_transfer_8 (1, 0);
    return out;
}

extern SemaphoreHandle_t send_sem;

// packet for Ardupilot RC UDP protocol
#define RCINPUT_UDP_NUM_CHANNELS 8
#define RCINPUT_UDP_VERSION 2

static struct __attribute__((packed)) pkt {
    uint32_t version;
    uint64_t timestamp_us;
    uint16_t sequence;
    uint16_t pwms[RCINPUT_UDP_NUM_CHANNELS];
} pkt;

#define PWM_LIMIT 2200

void lpc_task(void *pvParameters)
{
    pkt.version = RCINPUT_UDP_VERSION;
    pkt.timestamp_us = 0;
    pkt.sequence = 0;

    // Avoid to set SPI_CS(GPIO15) low early in the boot sequence
    vTaskDelay(1000/portTICK_PERIOD_MS);

    if (!spi_init(1, SPI_MODE0, SPI_FREQ_DIV_1M, true, SPI_BIG_ENDIAN, false)) {
        printf("Failed spi_init\n");
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1/portTICK_PERIOD_MS);
        if (lpc_read(READY)) {
            for (int i = 0; i < RCINPUT_UDP_NUM_CHANNELS; i++) {
                uint16_t pwm = lpc_read(2*i) + ((uint16_t)lpc_read(2*i+1) << 8);
                if (pwm > PWM_LIMIT) {
                    // don't send bad value
                    lpc_read(CLEAR_READY);
                    continue;
                }
                pkt.pwms[i] = pwm;
            }
            lpc_read(CLEAR_READY);
        } else {
            continue;
        }
        pkt.timestamp_us = ((uint64_t)1000) * xTaskGetTickCount();
        xSemaphoreTake(send_sem, portMAX_DELAY);
        int n = send((int)pvParameters, &pkt, sizeof(pkt), 0);
        if (n < 0) {
        }
        xSemaphoreGive(send_sem);
        pkt.sequence++;
    }
}
