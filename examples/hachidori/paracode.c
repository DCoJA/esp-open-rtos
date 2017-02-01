#include <stdio.h>
#include <string.h>

#include "espressif/esp_common.h"
#include "esp/uart.h"

#include "FreeRTOS.h"
#include "task.h"

#include "pwm.h"
#include "MadgwickAHRS.h"
#include "kfacc.h"

// parachute code for failsafe

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
static float stick_last = MIN_WIDTH;
static float base_adjust[4];
static float adjust[4];

void fs_task(void *pvParameters)
{
#ifdef RESTART_AT_FAST_RECONNECT
 restart:
#endif
    in_failsafe = false;
    uint32_t last_count = pwm_count;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1000/portTICK_PERIOD_MS);
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
        vTaskDelayUntil(&xLastWakeTime, 10/portTICK_PERIOD_MS);
        if (!in_arm) {
            continue;
        }
#ifdef RESTART_AT_FAST_RECONNECT
        if (count < 4*100 && last_count != pwm_count) {
            goto restart;
        }
#endif
        count++;

        float stick;
        // Try to avoid free fall.
        if (accz < GEPSILON) {
            stick = stick_last;
        } else {
            // Decrease pwm exponentially to MIN_WIDTH
            stick = (1-DRATE)*stick_last + DRATE*MIN_WIDTH;
            if (stick < LO_WIDTH) {
                stick = LO_WIDTH;
            }
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
        hup = -(q0*q1+q3*q2);
        rup = q0*q2-q3*q1;

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

        uint16_t wd[NUM_MOTORS];
        for (int i = 0; i < NUM_MOTORS; i++) {
            wd[i] = (uint16_t)(stick + adjust[i]);
        }

        pwm_output(wd, NUM_MOTORS);
     }
}

void fs_disarm(void)
{
    uint16_t wd[NUM_MOTORS];
    for (int i = 0; i < NUM_MOTORS; i++) {
        wd[i] = LO_WIDTH;
    }
    pwm_output(wd, NUM_MOTORS);
}
