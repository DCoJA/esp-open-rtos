/* udpcli FreeRTOSConfig overrides.
*/

#define configTICK_RATE_HZ			( ( TickType_t ) 1000 )

/* Use the defaults for everything else */
#include_next<FreeRTOSConfig.h>
