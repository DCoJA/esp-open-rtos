/* udp/tcp servers
   Recieve udp packets and send pmw data with SPI. Send/recv mavlink
   data to/from server.
 */

#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "esp/spi.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include <semphr.h>
#include <dhcpserver.h>

#include "lwip/api.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include "ssid_config.h"

#include "ip_config.h"

#include "ringbuf.h"

extern void ppm_task(void *pvParameters);
extern void uart_task(void *pvParameters);

SemaphoreHandle_t lpc_sem;
SemaphoreHandle_t rbuf_sem;
SemaphoreHandle_t sbuf_sem;

struct ringbuf mavrbuf;
struct ringbuf mavsbuf;


static void recv_task(void *pvParameters)
{
    int s = (int)pvParameters;

    // Read loop
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        uint8_t c;
        int n = read(s, &c, 1);
        if (n < 0) {
        } else if (n > 0) {
            //printf("read %02x\n", c);
            xSemaphoreTake(rbuf_sem, portMAX_DELAY);
            ringbuf_put(&mavrbuf, c);
            xSemaphoreGive(rbuf_sem);
        } else {
            vTaskDelayUntil(&xLastWakeTime, 10/portTICK_PERIOD_MS);
        }
    }
}

static uint8_t line[256+8+2];

static void send_task(void *pvParameters)
{
    int s = (int)pvParameters;

    // Write loop
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1) {
        xSemaphoreTake(sbuf_sem, portMAX_DELAY);
        uint32_t size = ringbuf_size(&mavsbuf);
        //printf("ringbuf_size %d\n", size);
        if (size == 0) {
            xSemaphoreGive(sbuf_sem);
            vTaskDelayUntil(&xLastWakeTime, 10/portTICK_PERIOD_MS);
            continue;
        }
        if (size > sizeof(line)) {
            size = sizeof(line);
        }
        for (int i = 0; i < size; i++) {
            line[i] = ringbuf_get(&mavsbuf);
        }
        xSemaphoreGive(sbuf_sem);
        write(s, line, size);
    }
}

static void ip_task(void *pvParameters)
{
    struct sockaddr_in saddr;
    struct sockaddr_in caddr;
    int rtn;

    printf("IP server tasks starting...\r\n");

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sockfd < 0) {
        printf("... Failed to allocate detagram socket.\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        return;
    }

    printf("... allocated detagram socket\r\n");

    int option = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));

    memset(&saddr, 0, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_addr.s_addr = htonl(INADDR_ANY);
    saddr.sin_port = htons(UDP_PORT);
    rtn = bind(sockfd, (struct sockaddr *) &saddr, sizeof(saddr));
    if(rtn < 0) {
        printf("... Failed to bind socket.\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        close(sockfd);
        return;
    }

    // Avoid to set SPI_CS(GPIO15) low early in the boot sequence
    vTaskDelay(1000/portTICK_PERIOD_MS);

    // Initialize SPI before ppm task
    printf("Initialize SPI\n");

    if (!spi_init(1, SPI_MODE0, SPI_FREQ_DIV_20M, true, SPI_BIG_ENDIAN, false)) {
        printf("Failed spi_init\n");
    }

    // Create ppm task
    xTaskCreate(ppm_task, "ppm_task", 512, (void*)sockfd, 2, NULL);

    int s = socket(AF_INET, SOCK_STREAM, 0);
    if(s < 0) {
        printf("... Failed to allocate socket.\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        return;
    }

    printf("... allocated stream socket\r\n");

    option = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));

    memset((char *) &saddr, 0, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_addr.s_addr = htonl(INADDR_ANY);
    saddr.sin_port = htons(TCP_PORT);

    rtn = bind (s, (struct sockaddr *)&saddr, sizeof(saddr));
    if (rtn < 0) {
        printf("... Failed to bind socket.\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        close (s);
        return;
    }

    rtn = listen(s, 1);
    if (rtn < 0) {
        printf("... Failed to listen socket.\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        close (s);
        return;
    }

    socklen_t clen;
    rtn = accept(s, (struct sockaddr *) &caddr, &clen);
    if (rtn < 0) {
        printf("... Failed to accept socket.\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        close (s);
        return;
    }

    int mavsockfd = rtn;
    // start recv/send tasks
    xTaskCreate(uart_task, "uart_task", 256, NULL, 3, NULL);
    xTaskCreate(recv_task, "recv_task", 256, (void *)mavsockfd, 4, NULL);
    xTaskCreate(send_task, "send_task", 256, (void *)mavsockfd, 4, NULL);

    while (1) {
        vTaskSuspend(NULL);
    }
}

void user_init(void)
{
    uart_set_baud(0, 115200);

    printf("SDK version : %s\n", sdk_system_get_sdk_version());
    printf("GIT version : %s\n", GITSHORTREV);

    sdk_wifi_set_opmode(SOFTAP_MODE);
    struct ip_info ap_ip;
    ipaddr_aton(UDP_SERVER, &ap_ip.ip);
    IP4_ADDR(&ap_ip.gw, 0, 0, 0, 0);
    IP4_ADDR(&ap_ip.netmask, 255, 255, 255, 0);
    sdk_wifi_set_ip_info(1, &ap_ip);

    struct sdk_softap_config ap_config = {
        .ssid = WIFI_SSID,
        .ssid_hidden = 0,
        .channel = 6,
        .ssid_len = strlen(WIFI_SSID),
        .authmode = AUTH_WPA_WPA2_PSK,
        .password = WIFI_PASS,
        .max_connection = 3,
        .beacon_interval = 100,
    };
    sdk_wifi_softap_set_config(&ap_config);

#ifndef USE_STATIC_IP_ADDRESS
    ip_addr_t first_client_ip;
    ipaddr_aton(UDP_CLIENT, &first_client_ip);
    dhcpserver_start(&first_client_ip, 4);
#endif

    vSemaphoreCreateBinary(lpc_sem);
    vSemaphoreCreateBinary(rbuf_sem);
    vSemaphoreCreateBinary(sbuf_sem);

    ringbuf_init(&mavrbuf);
    ringbuf_init(&mavsbuf);

    xTaskCreate(ip_task, "ip_task", 1024, NULL, 5, NULL);
}
