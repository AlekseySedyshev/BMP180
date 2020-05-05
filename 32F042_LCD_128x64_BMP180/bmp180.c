#include "stm32f0xx.h"
#include "BMP180.h"
#include "main.h"


void 			writeReg8(uint8_t reg, uint8_t value)										{//Write a 8-bit register
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD) & (~I2C_CR2_RD_WRN); 	
 	I2C1->CR2 |= (BMP180_Addr << I2C_CR2_SADD_Pos) | (0x80 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 	
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; I2C1->TXDR = reg;	while((I2C1->ISR & I2C_ISR_TC)){}; 	 
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; I2C1->TXDR = value;	while((I2C1->ISR & I2C_ISR_TC)){}; 
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; // TX buf Empty
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;	
}
uint8_t 	readReg8(uint8_t reg)																		{//Read an 8-bit register
	
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD) & (~I2C_CR2_RD_WRN); 	
 	I2C1->CR2 |= (BMP180_Addr << I2C_CR2_SADD_Pos) | (0x80 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 	
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; I2C1->TXDR = reg;	while((I2C1->ISR & I2C_ISR_TC)){};
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; // TX buf Empty
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;	
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD);	
	I2C1->CR2 |= I2C_CR2_RD_WRN;																				// Direction - data in
	I2C1->CR2 |= (BMP180_Addr << I2C_CR2_SADD_Pos) | (1 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 
	while(!(I2C1->ISR & I2C_ISR_RXNE)){};		
	uint8_t value=I2C1->RXDR;
	while(!(I2C1->ISR & I2C_ISR_TXE)){};
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;
	I2C1->CR2 &=(~I2C_CR2_RD_WRN) &(~I2C_CR2_NACK);
	return value; 		
}

uint16_t 	readReg16(uint8_t reg)																	{//Read a 16-bit register
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD) & (~I2C_CR2_RD_WRN); 	
 	I2C1->CR2 |= (BMP180_Addr << I2C_CR2_SADD_Pos) | (0x80 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 	
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; I2C1->TXDR = reg;	while((I2C1->ISR & I2C_ISR_TC)){};
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; // TX buf Empty
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;	
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD);	
	I2C1->CR2 |= I2C_CR2_RD_WRN;																				// Direction - data in
	I2C1->CR2 |= (BMP180_Addr << I2C_CR2_SADD_Pos) | (2 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 
	uint16_t value = 0;
	for (uint8_t i = 2; i > 0; i--) {
		while(!(I2C1->ISR & I2C_ISR_RXNE)){};
		value |= (uint16_t) (I2C1->RXDR << (8 * (i - 1)));
	}
	while(!(I2C1->ISR & I2C_ISR_TXE)){};
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;
	I2C1->CR2 &=(~I2C_CR2_RD_WRN) &(~I2C_CR2_NACK);
	return value; 
}


void 			readMulti(uint8_t reg, uint8_t * dst, uint8_t count)		{// readMulti
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD) & (~I2C_CR2_RD_WRN); 	
 	I2C1->CR2 |= (BMP180_Addr << I2C_CR2_SADD_Pos) | (0xff << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 	
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; I2C1->TXDR = reg;	while((I2C1->ISR & I2C_ISR_TC)){}; 	
	while(!(I2C1->ISR & I2C_ISR_TXE)){};
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY	
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD);	
	I2C1->CR2 |= I2C_CR2_RD_WRN;																				// Direction - data in
	I2C1->CR2 |= (BMP180_Addr << I2C_CR2_SADD_Pos) | (count << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 
	while (count-- > 0) {
		
		while(!(I2C1->ISR & I2C_ISR_RXNE)){};
		*(dst++) = I2C1->RXDR;
	}
	while(!(I2C1->ISR & I2C_ISR_TXE)){};
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;
	I2C1->CR2 &=(~I2C_CR2_RD_WRN) &(~I2C_CR2_NACK);
}


void 			getData(struct bmp180_t* sensorX)												{// Read Temp and Pressure 

  int32_t  x1, x2, b5, b6, x3, b3, p;
  uint32_t b4, b7;
	uint8_t dat[3];
	
  /* Get the raw pressure and temperature values */
  writeReg8(BMP180_CTRL_MEAS_REG, BMP180_T_MEASURE);
	delay_ms (BMP180_TEMP_CONVERSION_TIME);
  sensorX->raw_temp = readReg16(BMP180_ADC_OUT_MSB_REG);
			
  writeReg8(BMP180_CTRL_MEAS_REG, BMP180_P_MEASURE + (sensorX->oss << 6));
	switch (sensorX->oss) {				//Generate pause
		case 1: {delay_ms (BMP180_TEMP_CONVERSION_TIME *2 );}			//Pause calculate Data 10 mS
			break;
		case 2: {delay_ms (BMP180_TEMP_CONVERSION_TIME *3 );}			//Pause calculate Data 15 mS
			break;
		case 3: {delay_ms ((BMP180_TEMP_CONVERSION_TIME *5)+1);}	//Pause calculate Data 26 mS	
			break;
		default: {	delay_ms (BMP180_TEMP_CONVERSION_TIME);}			//Pause calculate Data 5 mS
			break; 
	}		
	readMulti(BMP180_ADC_OUT_MSB_REG, dat, 3);
	
	sensorX->raw_press= ((dat[0]<<16) | dat[1]<<8 | dat[2])>>(8-sensorX->oss);
    
   /* Temperature compensation */
  x1 = (sensorX->raw_temp - (int32_t)sensorX->calib_param.ac6) * ((int32_t)sensorX->calib_param.ac5) >> 15;
  x2 = ((int32_t)sensorX->calib_param.mc << 11) / (x1+(int32_t)sensorX->calib_param.md);
  b5 = x1 + x2;
	
	sensorX->t_out = ((float)((b5+8) >> 4))/10;
  

  /* Pressure compensation */
  b6 = b5 - 4000;
  x1 = ((sensorX->calib_param.b2) * ((b6 * b6) >> 12)) >> 11;
  x2 = (sensorX->calib_param.ac2 * b6) >> 11;
  x3 = x1 + x2;
  b3 = (((((int32_t) sensorX->calib_param.ac1) * 4 + x3) << sensorX->oss) + 2) >> 2;
  x1 = (sensorX->calib_param.ac3 * b6) >> 13;
  x2 = (sensorX->calib_param.b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (sensorX->calib_param.ac4 * (uint32_t) (x3 + 32768)) >> 15;
  b7 = ((uint32_t) (sensorX->raw_press - b3) * (50000 >> sensorX->oss));

  if (b7 < 0x80000000)
  {
    p = (b7 << 1) / b4;
  }
  else
  {
    p = (b7 / b4) << 1;
  }

  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  sensorX->pr_out = p + ((x1 + x2 + 3791) >> 4);
  /* Assign compensated pressure value */
}

void bmp180_init(struct bmp180_t* sensorX)												{// Init BMP180
    
    uint8_t data[BMP180_PROM_DATA__LEN];	//array for read Calibration Data
    
		sensorX->oss =3;	//Maximum sensivity (8 times rescan)
		sensorX->sensortype=readReg8(BMP180_VERSION_REG);
	
		readMulti(BMP180_PROM_START__ADDR, data, 22);
	
		sensorX->calib_param.ac1 = (int16_t)(data[0]<<8 | data[1]);  
	  sensorX->calib_param.ac2 = (int16_t)(data[2]<<8 | data[3]);
    sensorX->calib_param.ac3 = (int16_t)(data[4]<<8 | data[5]);
  
		sensorX->calib_param.ac4 =	(uint16_t)(data[6]<<8 | data[7]);
    sensorX->calib_param.ac5 = 	(uint16_t)(data[8]<<8 | data[9]);
    sensorX->calib_param.ac6 = 	(uint16_t)(data[10]<<8 | data[11]);
    
    sensorX->calib_param.b1 = (int16_t)(data[12]<<8 | data[13]);
    sensorX->calib_param.b2 = (int16_t)(data[14]<<8 | data[15]);
    sensorX->calib_param.mb = (int16_t)(data[16]<<8 | data[17]);
    sensorX->calib_param.mc = (int16_t)(data[18]<<8 | data[19]);
    sensorX->calib_param.md = (int16_t)(data[20]<<8 | data[21]);
 }

uint8_t bmp180_Reset(void) 																			{// Reset and read Dev Id
	writeReg8(BMP180_SOFT_RESET_REG, BMP180_RESET_VALUE);
	delay_ms (BMP180_TEMP_CONVERSION_TIME);
	return 	readReg8(BMP180_CHIP_ID_REG);
	}

