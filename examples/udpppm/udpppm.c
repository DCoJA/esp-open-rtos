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

#define CPPM_NUM_CHANNELS 8

#define FRAME 0x7f

static uint8_t lpc_read(uint8_t reg) __attribute__((unused));

static uint8_t lpc_read(uint8_t reg)
{
    uint16_t in = (uint16_t)(0x80 | reg) << 8;
    // Dummy reads will return garbage
    (void)spi_transfer_16 (1, in);
    (void)spi_transfer_16 (1, in);
    // Now slave is ready to send register value
    uint16_t out = spi_transfer_16 (1, in);
    return (uint8_t)out;
}

static void lpc_write(uint8_t reg, uint8_t val)
{
    uint16_t in = ((uint16_t)(reg) << 8) | val;
    spi_transfer_16 (1, in);
}

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
        //printf("recv %d bytes pkt.head %02x\n", n, pkt.version);
        if (n < 0) {
        }

        if (pkt.version != RCINPUT_UDP_VERSION) {
            continue;
        }

        // TODO check client address

        // Skip output so as not to too fast update of pwm
        TickType_t current_time = xTaskGetTickCount();
        if ((uint32_t)(current_time - last_time) <= 4) {
            last_time = current_time;
            continue;
        } else {
            last_time = current_time;
        }

        bool good_packet = true;
        // TODO check timestamp and sequence number

        // Check pwm values not so as to use bad value
        for (int i = 0; i < RCINPUT_UDP_NUM_CHANNELS; i++) {
            if (pkt.pwms[i] > PWM_WM_HIGH || pkt.pwms[i] < PWM_WM_LOW) {
                good_packet = false;
                break;
            }
        }

        if (!good_packet) {
            continue;
        }

        for (int i = 0; i < RCINPUT_UDP_NUM_CHANNELS; i++) {
            uint16_t pwm = pkt.pwms[i];
            //printf("ch %d: pwm %04d(us)\n", i, pwm);
            lpc_write(2*i, (uint8_t)(pwm & 0xff));
            lpc_write(2*i+1, (uint8_t)(pwm >> 8));
        }

        lpc_write(FRAME, 1);
    }
}
