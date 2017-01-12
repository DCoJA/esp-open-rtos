/* A udp/tcp client.
   Send pwm data to server.  Send/recv mavlink data to/from server.
 */

#include "espressif/esp_common.h"
#include "espressif/user_interface.h"
#include "esp/uart.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include <semphr.h>

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include "ringbuf.h"

#include "ssid_config.h"

#include "ip_config.h"

extern void lpc_task(void *pvParameters);

static int sockfd = -1;
static int mavsockfd = -1;

SemaphoreHandle_t send_sem;
SemaphoreHandle_t rbuf_sem;
SemaphoreHandle_t sbuf_sem;

struct ringbuf mavrbuf;
struct ringbuf mavsbuf;

static void ip_task(void *pvParameters)
{
    struct sockaddr_in saddr;
    struct sockaddr_in caddr;
    int rtn;
    int s;

    printf("client task starting...\r\n");

    s = socket(AF_INET, SOCK_DGRAM, 0);
    if(s < 0) {
        printf("... Failed to allocate socket.\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        return;
    }

    printf("... allocated datagram socket\r\n");

    memset((char *) &caddr, 0, sizeof(caddr));
    caddr.sin_family = AF_INET;
    caddr.sin_addr.s_addr = htonl(INADDR_ANY);
    caddr.sin_port = htons(UDP_PORT);

    memset((char *) &saddr, 0, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_addr.s_addr = inet_addr (UDP_SERVER);
    saddr.sin_port = htons(UDP_PORT);

    rtn = bind (s, (struct sockaddr *)&caddr, sizeof(caddr));
    if(rtn < 0) {
        printf("... Failed to bind socket.\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        close (s);
        return;
    }
    rtn = connect(s, (struct sockaddr *) &saddr, sizeof(saddr));
    if (rtn < 0) {
        printf("... Failed to connect socket.\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        close (s);
        return;
    }

    sockfd = s;

 tcp_retry:
    s = socket(AF_INET, SOCK_STREAM, 0);
    if(s < 0) {
        printf("... Failed to allocate socket.\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        return;
    }

    printf("... allocated stream socket\r\n");

    memset((char *) &saddr, 0, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_addr.s_addr = inet_addr (TCP_SERVER);
    saddr.sin_port = htons(TCP_PORT);

    rtn = connect(s, (struct sockaddr *) &saddr, sizeof(saddr));
    if (rtn < 0) {
        printf("... Failed to connect socket.\r\n");
        vTaskDelay(4000 / portTICK_PERIOD_MS);
        close (s);
        goto tcp_retry;
    }

    mavsockfd = s;

    while (1) {
        vTaskSuspend(NULL);
    }
}

static void recv_task(void *pvParameters)
{
    while (mavsockfd < 0) {
        vTaskDelay(4000 / portTICK_PERIOD_MS);
    }

    int s = mavsockfd;

    // Read loop
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        uint8_t c;
        int n = read(s, &c, 1);
        if (n < 0) {
        } else if (n > 0) {
            xSemaphoreTake(rbuf_sem, portMAX_DELAY);
            ringbuf_put(&mavrbuf, c);
            xSemaphoreGive(rbuf_sem);
        }
        vTaskDelayUntil(&xLastWakeTime, 10/portTICK_PERIOD_MS);
    }
}

static uint8_t line[256+8+2];

static void send_task(void *pvParameters)
{
    while (mavsockfd < 0) {
        vTaskDelay(4000 / portTICK_PERIOD_MS);
    }

    int s = mavsockfd;

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

static void led_task(void *pvParameters)
{
    static const int led = 4;

    gpio_enable(led, GPIO_OUTPUT);
    while(1) {
        gpio_write(led, 1);
        vTaskDelay(1000/portTICK_PERIOD_MS);
        gpio_write(led, 0);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

void user_init(void)
{
    uart_set_baud(0, 115200);
    // SCL:IO5 SDA:IO4
    // i2c_init(5, 4);

    // gpio_enable(2, GPIO_OUTPUT);

    // Just some information
    printf("\n");
    printf("SDK version : %s\n", sdk_system_get_sdk_version());
    printf("GIT version : %s\n", GITSHORTREV);

    struct sdk_station_config config = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASS,
    };

    // required to call wifi_set_opmode before station_set_config
    sdk_wifi_set_opmode(STATION_MODE);
#ifdef USE_STATIC_IP_ADDRESS
    sdk_wifi_station_dhcpc_stop();
    struct ip_info info;
    memset(&info, 0, sizeof(info));
    info.ip.addr = ipaddr_addr(UDP_CLIENT);
    info.netmask.addr = ipaddr_addr("255.255.255.0");
    info.gw.addr = ipaddr_addr(UDP_SERVER);
    sdk_wifi_set_ip_info(STATION_IF, &info);
#endif
    sdk_wifi_station_set_config(&config);

    vSemaphoreCreateBinary(send_sem);
    vSemaphoreCreateBinary(rbuf_sem);
    vSemaphoreCreateBinary(sbuf_sem);

    ringbuf_init(&mavrbuf);
    ringbuf_init(&mavsbuf);

    // start led task
    xTaskCreate(led_task, "led_task", 256, NULL, 6, NULL);

    // start ip task
    xTaskCreate(ip_task, "ip_task", 256, NULL, 5, NULL);

    // start recv/send tasks
    xTaskCreate(recv_task, "recv_task", 256, NULL, 4, NULL);
    xTaskCreate(send_task, "send_task", 256, NULL, 3, NULL);

    while (sockfd < 0) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // Create lpc task
    xTaskCreate(lpc_task, "lpc_task", 512, (void*)sockfd, 2, NULL);
}
