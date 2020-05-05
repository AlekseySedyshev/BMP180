#ifndef BMP180_H
#define BMP180_H
#include "stdint.h"
#include "stm32f0xx.h"

/*BMP180 I2C Address*/

#define BMP180_Addr 0xEE

/*register definitions */

#define BMP180_PROM_START__ADDR		(0xAA)
#define BMP180_PROM_DATA__LEN			(22)

#define BMP180_CHIP_ID_REG				(0xD0)
#define BMP180_VERSION_REG				(0xD1)

#define BMP180_CTRL_MEAS_REG			(0xF4)
#define BMP180_ADC_OUT_MSB_REG		(0xF6)
#define BMP180_ADC_OUT_LSB_REG		(0xF7)

#define BMP180_SOFT_RESET_REG			(0xE0)
#define BMP180_RESET_VALUE				(0xB6)

/* temperature measurement */
#define BMP180_T_MEASURE					(0x2E)
/* pressure measurement*/
#define BMP180_P_MEASURE					(0x34)

#define BMP180_TEMP_CONVERSION_TIME  (5)

//#define BMP180_PARAM_MG		(3038)
//#define BMP180_PARAM_MH		(-7357)
//#define BMP180_PARAM_MI		(3791)

struct bmp180_calib_param_t {
	int16_t ac1;	// ac1
	int16_t ac2;	//ac2
	int16_t ac3;	//ac3 
	uint16_t ac4;	//ac4 
	uint16_t ac5;	//ac5 
	uint16_t ac6;	//ac6 
	int16_t b1;		//b1 
	int16_t b2;		//b2 
	int16_t mb;		//mb 
	int16_t mc;		//mc 
	int16_t md;		//md 
};

struct bmp180_t {
	struct bmp180_calib_param_t calib_param;			/**<calibration data*/
	uint8_t oss;																	/**<power mode*/
	uint8_t chip_id; 															/**<chip id*/
	int32_t	pr_out;																//pressure Pa
	float t_out;																	//temp Float format xx.x C
	int32_t raw_temp;															//raw temp
	int32_t raw_press;														//raw pressure
	uint8_t dev_addr;															/**<device address*/
	uint8_t sensortype;														/**< sensor type*/
	
//	int32_t number_of_samples;										/**<sample calculation*/
//	int16_t oversamp_setting;											/**<oversampling setting*/
//	int16_t sw_oversamp;													/**<software oversampling*/
};
void 			writeReg8(uint8_t reg, uint8_t value);
uint8_t 	readReg8(uint8_t reg);
uint16_t 	readReg16(uint8_t reg);
void readMulti(uint8_t reg, uint8_t * dst, uint8_t count);
void getData(struct bmp180_t* sensorX);
void bmp180_init(struct bmp180_t* sensorX);
uint8_t bmp180_Reset(void);

#endif /* BMP180_H */
