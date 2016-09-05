#if 1
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

// u-blox neo-7m i2c

#define UBLOX_ADDRESS 0x42

#define UBLOX_COUNT_REG 0xfd
#define UBLOX_RAED_REG 0xff

extern xSemaphoreHandle i2c_sem;
extern xSemaphoreHandle send_sem;

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
    // Wait for read completed
    if (ublox_count) {
        return 0;
    }

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

void gps_task(void *pvParameters)
{
    struct LRpacket pkt;
    int count = 0;
    uint8_t block[2];
    xSemaphoreTake(i2c_sem, portMAX_DELAY);
    if (!i2c_slave_read_rc(UBLOX_ADDRESS, UBLOX_COUNT_REG, block,
                           sizeof(block))) {
        printf ("no working ublox I2C\n");
        xSemaphoreGive(i2c_sem);
        vTaskSuspend(NULL);
    }
    xSemaphoreGive(i2c_sem);

    portTickType xLastWakeTime = xTaskGetTickCount();
    while (1) {
        if (ublox_count == 0) {
            vTaskDelayUntil(&xLastWakeTime, 100/portTICK_RATE_MS);
        }
        // Read 30bytes.  See above.
        count = ublox_readn(&pkt.data[1], sizeof(pkt.data)-2);
        if (count == 0) {
            continue;
        }
        pkt.data[0] = count;
        // Send it
        xSemaphoreTake(send_sem, portMAX_DELAY);
        pkt.head = 0xD3;
        pkt.tos = TOS_GPS;
        int n = send((int)pvParameters, &pkt, sizeof(pkt), 0);
        if (n < 0) {
        }
        xSemaphoreGive(send_sem);
    }
}
#endif
