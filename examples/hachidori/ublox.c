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

#ifdef UBLOX_I2C
#include "i2c/i2c.h"
#endif

#include "b3packet.h"

#include "ringbuf.h"

#ifdef UBLOX_I2C
// u-blox neo-7m i2c
#define UBLOX_ADDRESS 0x42

#define UBLOX_COUNT_REG 0xfd
#define UBLOX_RAED_REG 0xff

extern SemaphoreHandle_t i2c_sem;
#endif

extern SemaphoreHandle_t send_sem;
extern SemaphoreHandle_t ringbuf_sem;

extern struct ringbuf ubloxbuf;

#ifdef UBLOX_I2C
static uint16_t ublox_count;

/* Similar with i2c_slave_read but use restart condition when writing
   register numbler to slave.  */

static bool i2c_slave_read_rc(uint8_t slave_addr, uint8_t data, uint8_t *buf, uint32_t len)
{
    bool success = false;
    do {
        i2c_start();
        if (!i2c_write(slave_addr << 1)) {
            break;
        }
        i2c_write(data);
        // Restrat condition
        i2c_start();
        if (!i2c_write(slave_addr << 1 | 1)) { // Slave address + read
            break;
        }
        while(len) {
            *buf = i2c_read(len == 1);
            buf++;
            len--;
        }
        success = true;
    } while(0);
    i2c_stop();
    if (!success) {
        printf("I2C: read rc error\n");
    }
    return success;
}

static int ublox_writen(uint8_t *buf, size_t size)
{
#if 0
    // Wait for read completed
    if (ublox_count) {
        return 0;
    }
#endif

    // Should be 2 bytes, at least (protocol limitation of chips)
    if (size <= 1) {
        return 0;
    }

    // Limit occupation.  Don't make 1 byte fragment
    if (size > 31) {
        size = 30;
    }

    xSemaphoreTake(i2c_sem, portMAX_DELAY);
    if (!i2c_slave_write(UBLOX_ADDRESS, buf, size)) {
        printf ("ublox: can't write\n");
        xSemaphoreGive(i2c_sem);
        return 0;
    }
    
    xSemaphoreGive(i2c_sem);
    return size;
}

static int ublox_readn(uint8_t *buf, size_t n)
{
    if (ublox_count == 0) {    
        uint8_t block[2];
        xSemaphoreTake(i2c_sem, portMAX_DELAY);
        if (!i2c_slave_read_rc(UBLOX_ADDRESS, UBLOX_COUNT_REG, block,
                               sizeof(block))) {
            printf ("ublox: can't read counter\n");
            xSemaphoreGive(i2c_sem);
            return 0;
        }
        xSemaphoreGive(i2c_sem);
        ublox_count = (uint16_t) ((block[0] << 8) | block[1]);
#if 0
        printf("retreive count %d\n", ublox_count);
#endif
        if (ublox_count == 0) {
            return 0;
        }
    }

    if (ublox_count < n) {
        n = ublox_count;
    }

    // Limit occupation.  Don't make 1 byte fragment
    if (n > 31) {
        n = 30;
    }
    
    xSemaphoreTake(i2c_sem, portMAX_DELAY);
    if (!i2c_slave_read_rc(UBLOX_ADDRESS, UBLOX_RAED_REG, buf, n)) {
        printf ("ublox: can't read data\n");
        xSemaphoreGive(i2c_sem);
        return 0;
    }
#if 0
    printf("%d bytes retrieved\n", n);
    {
        int j;
        for (j = 0; j < n; j++)
            printf("%02x ", buf[j]);
        printf("\n");
    }
#endif
    xSemaphoreGive(i2c_sem);
    ublox_count -= n;
    return n;
}

static bool tx_space(size_t len)
{
    return true;
}

#else
// ublox with uart interface
// stdin/stdout are connected to null device
long _read_r(struct _reent *r, int fd, char *ptr, int len)
{
    return 0;
}

long _write_r(struct _reent *r, int fd, const char *ptr, int len)
{
    return len;
}

static int ublox_writen(uint8_t *buf, size_t size)
{
    int i;
    for(i = 0; i < size; i++) {
        uart_putc_nowait(0, buf[i]);
    }
    return i;
}

static int ublox_readn(uint8_t *buf, size_t size)
{
    int i, ch;
    for(i = 0; i < size; i++) {
        ch = uart_getc_nowait(0);
        if (ch < 0) break;
        buf[i] = ch;
    }
    return i;
}

static bool tx_space(size_t len)
{
    size_t n;
    n = UART_FIFO_MAX - FIELD2VAL(UART_STATUS_TXFIFO_COUNT, UART(0).STATUS);
    if (n >= len) {
        return true;
    }
    return false;
}

// Baudrate iterator
static int
nextbaud (void)
{
    // baud rate list from ardupilot libraries/AP_GPS/AP_GPS.cpp
    static const int baudrates[] =
        { 4800, 19200, 38400, 115200, 57600, 9600, 230400 };
    // try from 38400
    static int idx = 1;

    idx = (idx + 1) % (sizeof (baudrates) / sizeof (baudrates[0]));
    return baudrates[idx];
}
#endif

static uint8_t cmdbuf[32];

void gps_task(void *pvParameters)
{
    struct B3packet pkt;
    int count = 0;
#ifdef UBLOX_I2C
    uint8_t block[2];
    xSemaphoreTake(i2c_sem, portMAX_DELAY);
    if (!i2c_slave_read_rc(UBLOX_ADDRESS, UBLOX_COUNT_REG, block,
                           sizeof(block))) {
        printf ("no working ublox I2C\n");
        xSemaphoreGive(i2c_sem);
        vTaskSuspend(NULL);
    }
    xSemaphoreGive(i2c_sem);
#else
    // set to u-blox default baud rate
    uart_set_baud(0, 9600);

    TickType_t last_mark = xTaskGetTickCount();
#endif

    bool in_tune = false;

    // Runnig at lowest priority.  See udpcli.c:user_init.
    while (1) {
        vTaskDelay(5/portTICK_PERIOD_MS);
        if (tx_space (30)) {
            uint32_t len = 0;
            xSemaphoreTake(ringbuf_sem, portMAX_DELAY);
            uint32_t size = ringbuf_size(&ubloxbuf);
            if (size >= 2) {
                if (size > 30) {
                    len = 30;
                } else {
                    len = size;
                }
                for (int i = 0; i < len; i++) {
                    cmdbuf[i] = ringbuf_get(&ubloxbuf);
                }
            }
            xSemaphoreGive(ringbuf_sem);
            if (len > 0) {
                if (ublox_writen(cmdbuf, len) != len) {
                    printf("[ublox] failed to write %d bytes\n", len);
                }
                // printf("write GPSCMD %d bytes\n", len);
            }
        }

        // Read 30bytes.  See above.
        count = ublox_readn(&pkt.data[1], sizeof(pkt.data)-2);
#ifndef UBLOX_I2C
        if (count == 0
            || (!memchr (&pkt.data[1], '$', count)
                && !memchr (&pkt.data[1], 0xb5, count))) {
            if (xTaskGetTickCount() - last_mark > 2000/portTICK_PERIOD_MS) {
                // Couldn't find marker 2 sec.  Try another baudrate.
                int baud = nextbaud();
                uart_set_baud(0, baud);
                // printf("try %d baud\n", baud);
                last_mark = xTaskGetTickCount();
                in_tune = true;
            }
        } else {
                last_mark = xTaskGetTickCount();
                in_tune = false;
        }
#endif
        if (count == 0 || in_tune) {
            continue;
        }
        pkt.data[0] = count;
        // Send it
        xSemaphoreTake(send_sem, portMAX_DELAY);
        pkt.head = B3HEADER;
        pkt.tos = TOS_GPS;
        int n = send((int)pvParameters, &pkt, sizeof(pkt), 0);
        if (n < 0) {
        }
        xSemaphoreGive(send_sem);
    }
}
