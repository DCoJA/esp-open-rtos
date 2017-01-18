// pwm definitions and globals

#define NUM_CHANNELS	8

#define MIN_WIDTH 800
#define LO_WIDTH 1100
#define HI_WIDTH 1900

#define NUM_MAPPED_CHANNELS 4
#ifdef HACHIDORI_CORE
# define USE_ESC
# define CHANNEL_MAP { 0, 1, 2, 3 }
#else
# define CHANNEL_MAP { 2, 0, 3, 1 }
#endif

extern bool in_failsafe;
extern uint32_t pwm_count;
extern float last_width[NUM_CHANNELS];

extern void pwm_output(uint16_t *wd, int nch);
