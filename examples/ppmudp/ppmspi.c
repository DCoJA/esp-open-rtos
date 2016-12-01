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

#define CPPM_NUM_CHANNELS 16

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

extern xSemaphoreHandle send_sem;

static struct pkt {
    uint8_t head;
    uint8_t tos;
    uint8_t data[2*CPPM_NUM_CHANNELS];
} pkt;

void lpc_task(void *pvParameters)
{
    pkt.head = 0x15;
    pkt.tos = 0;

    // Avoid to set SPI_CS(GPIO15) low early in the boot sequence
    vTaskDelay(1000/portTICK_RATE_MS);

    if (!spi_init(1, SPI_MODE0, SPI_FREQ_DIV_1M, true, SPI_BIG_ENDIAN, false)) {
        printf("Failed spi_init\n");
    }

    portTickType xLastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1/portTICK_RATE_MS);
        if (lpc_read(READY)) {
            for (int i = 0; i < 2*CPPM_NUM_CHANNELS; i++) {
                pkt.data[i] = lpc_read(i);
            }
            lpc_read(CLEAR_READY);
        } else {
            continue;
        }
            
        xSemaphoreTake(send_sem, portMAX_DELAY);
        int n = send((int)pvParameters, &pkt, sizeof(pkt), 0);
        if (n < 0) {
        }
        xSemaphoreGive(send_sem);
    }
}
