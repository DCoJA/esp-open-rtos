// Hachidori "last resort" Sensor/RCout packet

#define LRSIZE		32
#define LRHEADER	0xD3

struct LRpacket {
    uint8_t head;	// LRHEADER
    uint8_t tos;
    uint8_t data[LRSIZE];
};

#define TOS_IMU		0
#define TOS_MAG		2
#define TOS_BARO	8
#define TOS_GPS		12

#define TOS_PWM		64
#define TOS_GPSCMD	(64+12)
