/* A udp client.
   Send IMU/compass samples and recieve pmw data.
 */

#include "espressif/esp_common.h"
#include "esp/uart.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <semphr.h>

#include "i2c/i2c.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include "ssid_config.h"

#define ECHO_SERVER "10.253.253.40"
#define PORT 5790

extern void imu_task(void *pvParameters);
extern void pwm_task(void *pvParameters);
extern void baro_task(void *pvParameters);
extern void gps_task(void *pvParameters);

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
        vTaskDelay(1000 / portTICK_RATE_MS);
        return;
    }

    printf("... allocated socket\r\n");

    memset((char *) &caddr, 0, sizeof(caddr));
    caddr.sin_family = AF_INET;
    caddr.sin_addr.s_addr = htonl(INADDR_ANY);
    caddr.sin_port = htons(PORT);

    memset((char *) &saddr, 0, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_addr.s_addr = inet_addr (ECHO_SERVER);
    saddr.sin_port = htons(PORT);

    rtn = bind (s, (struct sockaddr *)&caddr, sizeof(caddr));
    if(rtn < 0) {
        printf("... Failed to bind socket.\r\n");
        vTaskDelay(1000 / portTICK_RATE_MS);
        close (s);
        return;
    }
    rtn = connect(s, (struct sockaddr *) &saddr, sizeof(saddr));
    if (rtn < 0) {
        printf("... Failed to connect socket.\r\n");
        vTaskDelay(1000 / portTICK_RATE_MS);
        close (s);
        return;
    }

    while (1) {
        sockfd = s;
        vTaskSuspend(NULL);
    }
}

xSemaphoreHandle i2c_sem;
xSemaphoreHandle send_sem;

#define WRITE_QUEUE_SIZE 256
xQueueHandle *write_queue;

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
    write_queue = xQueueCreate(WRITE_QUEUE_SIZE, sizeof(uint8_t *));

    // start udp task
    xTaskCreate(udp_task, (int8_t *)"udp_task", 256, NULL, 5, NULL);

    while (sockfd < 0) {
        vTaskDelay(100 / portTICK_RATE_MS);
    }

    // Create sensor and pwm tasks
    xTaskCreate(imu_task, (int8_t *)"imu_task", 512, (void*)sockfd, 4, NULL);
    xTaskCreate(pwm_task, (int8_t *)"pwm_task", 256, (void*)sockfd, 3, NULL);
    xTaskCreate(baro_task, (int8_t *)"baro_task", 512, (void*)sockfd, 2, NULL);
    xTaskCreate(gps_task, (int8_t *)"gps_task", 512, (void*)sockfd, 2, NULL);
}
