/* A udp client.
   Send pwm data to server.
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

#include "ssid_config.h"

#include "udp_config.h"

extern void lpc_task(void *pvParameters);

static int sockfd = -1;

static void udp_task(void *pvParameters)
{
    struct sockaddr_in saddr;
    struct sockaddr_in caddr;
    int rtn;

    printf("UDP client task starting...\r\n");

    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if(s < 0) {
        printf("... Failed to allocate socket.\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        return;
    }

    printf("... allocated socket\r\n");

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

    while (1) {
        sockfd = s;
        vTaskSuspend(NULL);
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

SemaphoreHandle_t send_sem;

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

    // start led task
    xTaskCreate(led_task, "led_task", 256, NULL, 4, NULL);

    // start udp task
    xTaskCreate(udp_task, "udp_task", 256, NULL, 3, NULL);

    while (sockfd < 0) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    // Create lpc tasks
    xTaskCreate(lpc_task, "lpc_task", 512, (void*)sockfd, 2, NULL);
}
