// pwm definitions and globals

#define NUM_CHANNELS	8

#define MIN_WIDTH 900
#define MAX_WIDTH 2200

extern bool in_failsafe;
extern uint32_t pwm_count;
extern float last_width[NUM_CHANNELS];

extern void pwm_output(uint16_t *wd, int nch);
