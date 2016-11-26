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

#include "MadgwickAHRS.h"
#include "kfacc.h"

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

extern xSemaphoreHandle i2c_sem;
extern xSemaphoreHandle ringbuf_sem;

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
    vTaskDelay(1/portTICK_RATE_MS);
    // Restart PCA9685, auto-increment enabled
    pca9685_write(PCA9685_RA_MODE1,
                  (PCA9685_MODE1_RESTART_BIT|PCA9685_MODE1_AI_BIT));

    xSemaphoreGive(i2c_sem);
}

#define NUM_CHANNELS	8

static bool in_failsafe = false;
static uint32_t pwm_count = 0;
static float last_width[NUM_CHANNELS];

#define MIN_WIDTH 900
#define MAX_WIDTH 2200

void pwm_task(void *pvParameters)
{
    pca9685_init();

    struct LRpacket pkt;
    portTickType last_time = xTaskGetTickCount();
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
        if (in_failsafe) {
            continue;
        }

        // skip output so as not to eat up cpu time with bit-bang
        portTickType current_time = xTaskGetTickCount();
        if ((uint32_t)(current_time - last_time) <= 4) {
            last_time = current_time;
            continue;
        } else {
            last_time = current_time;
        }

        xSemaphoreTake(i2c_sem, portMAX_DELAY);
        for (int i = 0; i < NUM_CHANNELS; i++) {
            uint16_t width = ((uint16_t)pkt.data[2*i] << 8)|pkt.data[2*i+1];
            last_width[i] = (float)width;
            if (width >= MIN_WIDTH && width <= MAX_WIDTH) {
                // write ch data to PCA9685
                pca9685_out(i, width);
            }
        }
        xSemaphoreGive(i2c_sem);
    }
}

// parachute code

#define NUM_MOTORS 4
// (1-DRATE)^100 = 0.5
#define DRATE (1.0f - 0.9965403f)
#define GEPSILON 0.1f

#define MAXADJ 50
#define ROLL 1.0f
#define PITCH 1.0f
#define YAW 10.0f

#define PGAIN 0.9f
#define DGAIN 32.00f
#define GCOEFF 200.0f
#define FORGET 0.1f
#define BCOUNT 10

static float dp[4], dd[4];
static float qp0 = 1.0f, qp1 = 0.0f, qp2 = 0.0f, qp3 = 0.0f;
static float stick_last = 900.0f;
static float base_adjust[4];
static float adjust[4];

void fs_task(void *pvParameters)
{
 restart:
    in_failsafe = false;
    uint32_t last_count = pwm_count;
    portTickType xLastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, 2000/portTICK_RATE_MS);
        if (pwm_count) {
            if (last_count == pwm_count) {
                break;
            }
            last_count = pwm_count;
        }
    }

    // Start failsafe
    in_failsafe = true;
    uint32_t count;
    count = 0;

    // Take the mean value of last widths as the virtual throttle
    uint16_t sum = 0;
    for (int i = 0; i < NUM_MOTORS; i++) {
        sum += last_width[i];
    }
    stick_last = (float)(sum / NUM_MOTORS);

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, 10/portTICK_RATE_MS);
        if (count < 4*100 && last_count != pwm_count) {
            goto restart;
        }
        count++;

        float stick;
        // Try to avoid free fall.
        if (accz < -GEPSILON) {
            stick = stick_last;
        } else {
            // Decrease pwm exponentially to MIN_WIDTH
            stick = (1-DRATE)*stick_last + DRATE*MIN_WIDTH;
            stick_last = stick;
        }
        /* Try to keep horizontal attitude.  Rough AHRS gives the values
           which estimate current roll and pitch.  Adjustment value
           of each motor is determined by simple mix of these values
           according to the position of the motor.  We assume X-copter
           with motors configured like as:
           M3   M1       head
              x     left  ^  right
           M2   M4       tail
         */
        float rup, hup, ydelta, d[NUM_MOTORS];
        // These are rough approximations which would be enough for
        // our purpose.
        rup = q0*q1+q3*q2;
        hup = q0*q2-q3*q1;

        // Estimate yaw change
        if (qp0 == 1.0f) {
	    ydelta = 0;
        } else {
            float qDot0, qDot1, qDot2, qDot3;
            qDot0 = q0 - qp0;
            qDot1 = q1 - qp1;
            qDot2 = q2 - qp2;
            qDot3 = q3 - qp3;
            // yaw speed at body frame is 2 * qDot * qBar
            ydelta = -q3*qDot0 - q2*qDot1 + q1*qDot2 + q0*qDot3;
        }
        qp0 = q0;
        qp1 = q1;
        qp2 = q2;
        qp3 = q3;

        d[0] =  ROLL*rup + PITCH*hup + YAW*ydelta; // M1 right head
        d[1] = -ROLL*rup - PITCH*hup + YAW*ydelta; // M2 left  tail
        d[2] = -ROLL*rup + PITCH*hup - YAW*ydelta; // M3 left  head
        d[3] =  ROLL*rup - PITCH*hup - YAW*ydelta; // M4 right tail
        //printf ("d0 %7.3f d1 %7.3f d2 %7.3f d3 %7.3f\n", d[0], d[1], d[2], d[3]);
        for (int i = 0; i < NUM_MOTORS; i++) {
            float adj = base_adjust[i];
            float pv = d[i];
            float dv = dp[i]-d[i];
            dp[i] = d[i];
            dd[i] = dv;
            adj = (1-FORGET)*(pv*PGAIN-dv*DGAIN)*GCOEFF + FORGET*adj;
            if (adj > MAXADJ) {
                adj = MAXADJ;
            } else if (adj < -MAXADJ) {
                adj = -MAXADJ;
            }
            adjust[i] = adj;
            if ((count % BCOUNT) == 0) {
                base_adjust[i] = adj;
            }
        }
        
        xSemaphoreTake(i2c_sem, portMAX_DELAY);
        for (int i = 0; i < NUM_MOTORS; i++) {
            uint16_t width = (uint16_t)(stick + adjust[i]);
            if (width >= MIN_WIDTH && width <= MAX_WIDTH) {
                // write ch data to PCA9685
                pca9685_out(i, width);
            }
        }
        xSemaphoreGive(i2c_sem);
     }
}
