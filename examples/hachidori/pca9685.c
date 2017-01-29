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

#include "ringbuf.h"

#include "pwm.h"

// PCA9685
#define PCA9685_ADDRESS            0x40

#define PCA9685_RA_MODE1           0x00
#define PCA9685_RA_MODE2           0x01
#define PCA9685_RA_LED0_ON_L       0x06
#define PCA9685_RA_LED0_ON_H       0x07
#define PCA9685_RA_LED0_OFF_L      0x08
#define PCA9685_RA_LED0_OFF_H      0x09
#define PCA9685_RA_ALL_LED_ON_L    0xFA
#define PCA9685_RA_ALL_LED_ON_H    0xFB
#define PCA9685_RA_ALL_LED_OFF_L   0xFC
#define PCA9685_RA_ALL_LED_OFF_H   0xFD
#define PCA9685_RA_PRE_SCALE       0xFE

#define PCA9685_MODE1_RESTART_BIT  (1 << 7)
#define PCA9685_MODE1_EXTCLK_BIT   (1 << 6)
#define PCA9685_MODE1_AI_BIT       (1 << 5)
#define PCA9685_MODE1_SLEEP_BIT    (1 << 4)
#define PCA9685_MODE1_SUB1_BIT     (1 << 3)
#define PCA9685_MODE1_SUB2_BIT     (1 << 2)
#define PCA9685_MODE1_SUB3_BIT     (1 << 1)
#define PCA9685_MODE1_ALLCALL_BIT  (1 << 0)
#define PCA9685_ALL_LED_OFF_H_SHUT (1 << 4)
#define PCA9685_MODE2_INVRT_BIT    (1 << 4)
#define PCA9685_MODE2_OCH_BIT      (1 << 3)
#define PCA9685_MODE2_OUTDRV_BIT   (1 << 2)
#define PCA9685_MODE2_OUTNE1_BIT   (1 << 1)
#define PCA9685_MODE2_OUTNE0_BIT   (1 << 0)

#define PWM_FREQ_HZ 400

#if (PWM_FREQ_HZ == 400)
#define PCA9685_FREQ_PRESCALE	15
#elif (PWM_FREQ_HZ == 200)
#define PCA9685_FREQ_PRESCALE	31
#elif (PWM_FREQ_HZ == 100)
#define PCA9685_FREQ_PRESCALE	63
#elif (PWM_FREQ_HZ == 50)
#define PCA9685_FREQ_PRESCALE	127
#else
#error "unknown freq_hz"
#endif

extern SemaphoreHandle_t i2c_sem;
extern SemaphoreHandle_t ringbuf_sem;

extern struct ringbuf ubloxbuf;

static void pca9685_write(uint8_t reg, uint8_t val)
{
    uint8_t d[] = { reg, val };
    i2c_slave_write(PCA9685_ADDRESS, d, 2);
}

static void pca9685_write_led_on(uint8_t reg, uint16_t val)
{
    uint8_t d[] = { reg + 2, val & 0xff, val >> 8 };
    i2c_slave_write(PCA9685_ADDRESS, d, 3);
}

static void pca9685_out(int ch, uint16_t width)
{
    uint32_t length = 0;
    // length = round((width * 4096)/(1000000.f/(freq_hz*(1+epsilon)) - 1
#if (PWM_FREQ_HZ == 400)
    // approx 1.6402 with 6718/4096
    length = ((width * 6718) >> 12) - 1;
#elif (PWM_FREQ_HZ == 200)
    // approx 0.8201 with 3359/4096
    length = ((width * 3359) >> 12) - 1;
#elif (PWM_FREQ_HZ == 100)
    // approx 0.4099 with 1679/4096
    length = ((width * 1679) >> 12) - 1;
#elif (PWM_FREQ_HZ == 50)
    // approx 0.2063 with 845/4096
    length = ((width * 845) >> 12) - 1;
#else
#error "unknown freq_hz"
#endif

    // pca9685_write(PCA9685_RA_LED0_ON_L + 4*ch + 0, 0);
    // pca9685_write(PCA9685_RA_LED0_ON_L + 4*ch + 1, 0);
    // pca9685_write(PCA9685_RA_LED0_ON_L + 4*ch + 2, length & 0xff);
    // pca9685_write(PCA9685_RA_LED0_ON_L + 4*ch + 3, length >> 8);
    pca9685_write_led_on(PCA9685_RA_LED0_ON_L + 4*ch, length);

    //printf("ch%d %d(%d)%c", ch, length, width, (ch == 3 ? '\n' : ' '));
}

static void pca9685_init(void)
{
    xSemaphoreTake(i2c_sem, portMAX_DELAY);

    pca9685_write(PCA9685_RA_ALL_LED_ON_L, 0);
    pca9685_write(PCA9685_RA_ALL_LED_ON_H, 0);
    pca9685_write(PCA9685_RA_ALL_LED_OFF_L, 0);
    pca9685_write(PCA9685_RA_ALL_LED_OFF_H, 0);

    // Shutdown before sleeping
    pca9685_write(PCA9685_RA_ALL_LED_OFF_H, PCA9685_ALL_LED_OFF_H_SHUT);
    // Put PCA9685 to sleep so to write prescaler
    pca9685_write(PCA9685_RA_MODE1, PCA9685_MODE1_SLEEP_BIT);
    // prescale 67 for freq 99Hz
    pca9685_write(PCA9685_RA_PRE_SCALE, PCA9685_FREQ_PRESCALE);
    // Wait 1ms
    vTaskDelay(1/portTICK_PERIOD_MS);
    // Restart PCA9685, auto-increment enabled
    pca9685_write(PCA9685_RA_MODE1,
                  (PCA9685_MODE1_RESTART_BIT|PCA9685_MODE1_AI_BIT));

    xSemaphoreGive(i2c_sem);
}

// pwm global
bool in_failsafe = false;
bool disarm = false;
uint32_t pwm_count = 0;
float last_width[NUM_CHANNELS];
static int chmap[NUM_MAPPED_CHANNELS] = CHANNEL_MAP;

static inline int channel_map (int i)
{
    return (i < NUM_MAPPED_CHANNELS ? chmap[i] : i);
}

static inline int pwm_scale (uint16_t width)
{
    if (width < LO_WIDTH) {
        return 0;
    }
#if defined (USE_ESC)
    if (width > HI_WIDTH) {
        return HI_WIDTH;
    }
    return width;
#elif defined (USE_1S_BATT)
    // 1S case: Map [1100, 1900] to [0, 2500] and cut less than 200
    int32_t length = width - 1100;
    // 2500/800 times
    length = (length * 25) >> 3;
    if (length > 2500) {
        return 2500;
    } else if (length < 200) {
        return 0;
    }
    return (uint16_t)length;
#elif defined (USE_2S_BATT)
    // 2S case: Map [1100, 1900] to [0, 1250] and cut less than 100
    int32_t length = width - 1100;
    // 1250/800 times
    length = (length * 25) >> 4;
    if (length > 1250) {
        return 1250;
    } else if (length < 100) {
        return 0;
    }
    return (uint16_t)length;
#else
#error "No ESC or Battery specified"
    return 0;
#endif
}

void pwm_output(uint16_t *wd, int nch)
{
    xSemaphoreTake(i2c_sem, portMAX_DELAY);
    for (int i = 0; i < nch; i++) {
        uint16_t width = wd[i];
        if (width >= MIN_WIDTH && width <= HI_WIDTH) {
            // write ch data to PCA9685
            pca9685_out(channel_map (i), pwm_scale (width));
        }
    }
    xSemaphoreGive(i2c_sem);
}

void pwm_init(void)
{
    pca9685_init();

#ifdef USE_ESC
    // wait 3 sec for esc start up
    printf("wait 3 sec for esc start up\n");
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    xSemaphoreTake(i2c_sem, portMAX_DELAY);
    for (int i = 0; i < NUM_CHANNELS; i++) {
        // write minimal stick data to PCA9685
        pca9685_out(i, LO_WIDTH + 2);
    }
    xSemaphoreGive(i2c_sem);
    // wait 6 sec for normal esc/motor start up
    printf("wait 6 sec for normal esc/motor start up\n");
    vTaskDelay(6000 / portTICK_PERIOD_MS);
    xSemaphoreTake(i2c_sem, portMAX_DELAY);
    for (int i = 0; i < NUM_CHANNELS; i++) {
        pca9685_out(i, 0);
    }
    xSemaphoreGive(i2c_sem);
#endif
}

void pwm_task(void *pvParameters)
{
    struct LRpacket pkt;
    TickType_t last_time = xTaskGetTickCount();
    while (1) {
        // Wait udp packet
        int n = recv((int)pvParameters, &pkt, sizeof(pkt), 0);
        if (n != sizeof(pkt) || pkt.head != LRHEADER)
            continue;

        if (pkt.tos == TOS_GPSCMD) {
            size_t len = pkt.data[0];
            xSemaphoreTake(ringbuf_sem, portMAX_DELAY);
            // Write ringbuf
            for (int i = 0; i < len; i++) {
                ringbuf_put(&ubloxbuf, pkt.data[1+i]);
            }
            xSemaphoreGive(ringbuf_sem);
            // printf("receive GPSCMD %d bytes\n", len);
            continue;
        } else if (pkt.tos != TOS_PWM) {
            continue;
        }

        pwm_count++;
        if (in_failsafe || disarm) {
            continue;
        }

        // skip output so as not to eat up cpu time with bit-bang
        TickType_t current_time = xTaskGetTickCount();
        if ((uint32_t)(current_time - last_time) <= 4) {
            last_time = current_time;
            continue;
        } else {
            last_time = current_time;
        }

        uint16_t wd[NUM_CHANNELS];
        for (int i = 0; i < NUM_CHANNELS; i++) {
            uint16_t width = ((uint16_t)pkt.data[2*i] << 8)|pkt.data[2*i+1];
            wd[i] = width;
            last_width[i] = (float)width;
        }

        pwm_output(wd, NUM_CHANNELS);
    }
}
