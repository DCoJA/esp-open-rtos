/* A udp client.
   Send IMU/compass samples and recieve pmw data.
 */

#include "espressif/esp_common.h"
#include "esp/uart.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include <semphr.h>

#include "i2c/i2c.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include "ssid_config.h"

#include "udp_config.h"

#include "ringbuf.h"

extern void imu_task(void *pvParameters);
extern void pwm_task(void *pvParameters);
extern void baro_task(void *pvParameters);
extern void gps_task(void *pvParameters);
extern void fs_task(void *pvParameters);

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

SemaphoreHandle_t i2c_sem;
SemaphoreHandle_t send_sem;
SemaphoreHandle_t ringbuf_sem;

struct ringbuf ubloxbuf;

void user_init(void)
{
    uart_set_baud(0, 115200);
    // SCL:IO5 SDA:IO4
    i2c_init(5, 4);

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
    sdk_wifi_station_set_config(&config);

    vSemaphoreCreateBinary(i2c_sem);
    vSemaphoreCreateBinary(send_sem);
    vSemaphoreCreateBinary(ringbuf_sem);

    // Initialize ring buffer
    ringbuf_init (&ubloxbuf);

    // start udp task
    xTaskCreate(udp_task, "udp_task", 256, NULL, 6, NULL);

    while (sockfd < 0) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    // Create sensor and pwm tasks
    xTaskCreate(imu_task, "imu_task", 768, (void*)sockfd, 5, NULL);
    xTaskCreate(pwm_task, "pwm_task", 256, (void*)sockfd, 4, NULL);
    xTaskCreate(baro_task, "baro_task", 512, (void*)sockfd, 3, NULL);
    xTaskCreate(gps_task, "gps_task", 512, (void*)sockfd, 2, NULL);
}
