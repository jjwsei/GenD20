#ifndef COMAPSS_H
#define COMAPSS_H

#define COMPASS_ADDRESS 	0x1E
#define CONFIG_REGISTER_A	0x00
#define CONFIG_REGISTER_B	0x01
#define MODE_REGISTER		0x02
#define DATA_X_MSB_REGISTER	0x03
#define DATA_X_LSB_REGISTER	0x04	
#define DATA_Z_MSB_REGISTER	0x05
#define DATA_Z_LSB_REGISTER	0x06
#define DATA_Y_MSB_REGISTER	0x07
#define DATA_Y_LSB_REGISTER	0x08
#define STATUS_REGISTER		0x09
#define ID_REGISTER_A		0x0A
#define ID_REGISTER_B		0x0B
#define ID_REGISTER_C		0x0C

#define SAMPLES_AVERAGED_1	0x00
#define SAMPLES_AVERAGED_2	0x20
#define SAMPLES_AVERAGED_4	0x40
#define SAMPLES_AVERAGED_8	0x60

void CompassInit(struct i2c_master_module);
void CompassGetData(void);
int16_t CompassGetXData(void);
int16_t CompassGetYData(void);
int16_t CompassGetZData(void);

#endif
