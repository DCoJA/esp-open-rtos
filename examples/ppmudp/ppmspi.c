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
    uint16_t in = (uint16_t)(0x80 | reg) << 8;
    // dummy reads and wait for slave
    (void)spi_transfer_16 (1, in);
    (void)spi_transfer_16 (1, in);
    uint16_t out = spi_transfer_16 (1, in);
    return (uint8_t)out;

}

static void lpc_write(uint8_t reg, uint8_t val)
{
    uint16_t in = ((uint16_t)(reg) << 8) | val;
    spi_transfer_16 (1, in);
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

#define PWM_WM_HIGH 2200
#define PWM_WM_LOW   800

void lpc_task(void *pvParameters)
{
    pkt.version = RCINPUT_UDP_VERSION;
    pkt.timestamp_us = 0;
    pkt.sequence = 0;

    // Avoid to set SPI_CS(GPIO15) low early in the boot sequence
    vTaskDelay(1000/portTICK_PERIOD_MS);

    if (!spi_init(1, SPI_MODE0, SPI_FREQ_DIV_20M, true, SPI_BIG_ENDIAN, false)) {
        printf("Failed spi_init\n");
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
    wait:
        vTaskDelayUntil(&xLastWakeTime, 1/portTICK_PERIOD_MS);
        if (lpc_read(READY) == 1) {
            for (int i = 0; i < RCINPUT_UDP_NUM_CHANNELS; i++) {
                uint16_t pwm = lpc_read(2*i) + ((uint16_t)lpc_read(2*i+1) << 8);
                if (pwm > PWM_WM_HIGH || pwm < PWM_WM_LOW) {
                    // skip this round
                    lpc_write(CLEAR_READY, 0);
                    goto wait;
                }
                pkt.pwms[i] = pwm;
            }
            lpc_write(CLEAR_READY, 0);
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
