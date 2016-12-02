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

#include "lwip/api.h"
#include "lwip/err.h"
#include "lwip/sockets.h"

#define CPPM_NUM_CHANNELS 16

#define FRAME 0x7f

static void lpc_write(uint8_t reg, uint8_t val)
{
    uint16_t in = ((uint16_t)(reg) << 8) | val;
    spi_transfer_16 (1, in);
}

static struct pkt {
    uint8_t head;
    uint8_t tos;
    uint8_t data[2*CPPM_NUM_CHANNELS];
} pkt;

#define PKT_HEAD 0x15

void lpc_task(void *pvParameters)
{
    // Avoid to set SPI_CS(GPIO15) low early in the boot sequence
    vTaskDelay(1000/portTICK_PERIOD_MS);

    printf("Start lpc task\n");

    if (!spi_init(1, SPI_MODE0, SPI_FREQ_DIV_1M, true, SPI_BIG_ENDIAN, false)) {
        printf("Failed spi_init\n");
    }

    TickType_t last_time = xTaskGetTickCount();
    while (1) {
        struct sockaddr_in cli_addr;
        socklen_t clilen = sizeof(cli_addr);
        memset(&cli_addr, 0, clilen);
        int n = recvfrom((int)pvParameters, &pkt, sizeof(pkt), 0,
                         (struct sockaddr *) &cli_addr, &clilen);
        //printf("recv %d bytes\n", n);
        if (n < 0) {
        }

        if (pkt.head != PKT_HEAD) {
            continue;
        }

        // TODO check client address

        // skip output so as not to too fast update of pwm
        TickType_t current_time = xTaskGetTickCount();
        if ((uint32_t)(current_time - last_time) <= 10) {
            last_time = current_time;
            continue;
        } else {
            last_time = current_time;
        }

        for (int i = 0; i < 2*CPPM_NUM_CHANNELS; i++) {
            lpc_write(i, pkt.data[i]);
        }

        lpc_write(FRAME, 1);
    }
}
