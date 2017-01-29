// ina226 functions and constants

// 1.25mv/bit
#define INA226_VBUS_COEFF	(1.25*0.001)
#if defined(HACHIDORI_CORE)
// 1.25mA/bit
#define INA226_CURR_COEFF	(1.25*0.001)
// Set full scall to 40.96A
#define INA226_CALIB_VALUE	0x0800
#else
// 0.2mA/bit
#define INA226_CURR_COEFF	(0.2*0.001)
// Set full scall to 6.4A
#define INA226_CALIB_VALUE	0x0500
#endif

extern bool ina226_init(void);
extern bool ina226_read_sample(uint16_t *curr, uint16_t *vbus);
