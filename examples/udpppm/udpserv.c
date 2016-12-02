/* A udp server
   Recieve udp packets and send pmw data with SPI.
 */

#include "espressif/esp_common.h"
#include "esp/uart.h"

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

#include "udp_config.h"

extern void lpc_task(void *pvParameters);

static void udp_task(void *pvParameters)
{
    struct sockaddr_in saddr;
    int rtn;

    printf("UDP server task starting...\r\n");

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sockfd < 0) {
        printf("... Failed to allocate socket.\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        return;
    }

    printf("... allocated socket\r\n");

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

    // Create lpc task
    xTaskCreate(lpc_task, "lpc_task", 512, (void*)sockfd, 2, NULL);

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

    xTaskCreate(udp_task, "udp_task", 512, NULL, 3, NULL);
}
