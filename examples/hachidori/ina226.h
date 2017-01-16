// ina226 functions and constants

// 1.25mv/bit
#define INA226_VBUS_COEFF	(1.25*0.001)
// 0.2mA/bit
#define INA226_CURR_COEFF	(0.2*0.001)

extern bool ina226_init(void);
extern bool ina226_read_sample(uint16_t *curr, uint16_t *vbus);
