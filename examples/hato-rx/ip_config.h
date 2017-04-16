// If USE_EXTERNAL_AP is defined, use the external AP instead of soft AP
#define USE_EXTERNAL_AP
//#undef USE_EXTERNAL_AP

#if defined(USE_EXTERNAL_AP)
#define GW_ADDRESS "192.168.11.1"
#define USE_STATIC_IP_ADDRESS
#endif

// UDP/TCP server and client
#if defined(USE_STATIC_IP_ADDRESS)
#define UDP_SERVER "192.168.11.11"
#define UDP_CLIENT "192.168.11.12"
#else
#define UDP_SERVER "192.168.11.1"
#define UDP_CLIENT "192.168.11.2"
#endif
#define UDP_PORT 5890
#define TCP_SERVER UDP_SERVER
#define TCP_PORT 5900

// You can override the default WIFI setting in ssid_config.h here.
//#undef WIFI_SSID
//#define WIFI_SSID "hato-0"

