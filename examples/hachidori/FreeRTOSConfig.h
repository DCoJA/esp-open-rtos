/* udpcli FreeRTOSConfig overrides.
*/

#define configCPU_CLOCK_HZ			( ( unsigned long ) 160000000 )
#define configTICK_RATE_HZ			( ( TickType_t ) 1000 )

/* Use the defaults for everything else */
#include_next<FreeRTOSConfig.h>
