#include "stm32f0xx.h"        // Device header
#include "SSD1306.h"
#include "BMP180.h"
#include "main.h"
#include <math.h>

//#define I2C_100				1	//		100 kHz i2C Freq
#define I2C_400				1		//		400 kHz i2C Freq

uint8_t i,j,flag=0;

uint16_t TimingDelay,led_count,sec05_f=0,s=0;
uint16_t counter;


void TimingDelayDec(void) 																													{ //msec - timer
 if (TimingDelay			!=0x00) TimingDelay--;
 if (!led_count) {led_count=500; GPIOB->ODR ^=1;sec05_f++;}
 led_count--;
 }

void TIM17_IRQHandler(void)																													{
		if (TIM17->SR & TIM_SR_UIF) {
					TimingDelayDec();
  				TIM17->SR &=(~TIM_SR_UIF);
		}
}	
void TIM2_IRQHandler(void)																													{
if (TIM2->SR & TIM_SR_TIF) {
	//				counter = TIM2->CNT/469;
  //				TIM2->SR &=(~TIM_SR_TIF);
}
}
void delay_ms (uint16_t DelTime) 																										{
    TimingDelay=DelTime;
  while(TimingDelay!= 0x00);
}
//------------------------------LCD ------------------------------

void initial (void)																																	{
//---------------TIM17------------------
  RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;    																			//HSI 8 MHz - 1 msek
  TIM17->PSC = 8000-1;
  TIM17->ARR = 1;
  TIM17->CR1 |= TIM_CR1_ARPE | TIM_CR1_DIR | TIM_CR1_CEN; 											// 
	TIM17->DIER |=TIM_DIER_UIE;
	NVIC_EnableIRQ (TIM17_IRQn);
	NVIC_SetPriority(TIM17_IRQn,0x05);	

//-------------------GPIOB-Blinking Led		
	RCC->AHBENR  |= RCC_AHBENR_GPIOBEN; 								//
	GPIOB->MODER |= GPIO_MODER_MODER0_0;								//Pb0-Out 
//------------I2C1---------------------	
	RCC->AHBENR 		|=RCC_AHBENR_GPIOFEN;
	GPIOF->MODER 		|=GPIO_MODER_MODER0_1 		| GPIO_MODER_MODER1_1; 							// Alt -mode /Pf0 - SDA, Pf1- SCL
	GPIOF->OSPEEDR 	|=GPIO_OSPEEDER_OSPEEDR0 	| GPIO_OSPEEDER_OSPEEDR1;
	GPIOF->OTYPER		|=GPIO_OTYPER_OT_0 				| GPIO_OTYPER_OT_1;
	GPIOF->AFR[0] 	|=(1<<GPIO_AFRL_AFRL0_Pos) |(1<<GPIO_AFRL_AFRL1_Pos);  				// I2C - Alternative

	RCC->APB1ENR |=RCC_APB1ENR_I2C1EN;
#ifdef I2C_100	
	I2C1->TIMINGR |=(0x1	<<I2C_TIMINGR_PRESC_Pos); 	//100 kHz - I2C bus speed
	I2C1->TIMINGR |=(0x13	<<I2C_TIMINGR_SCLL_Pos);
	I2C1->TIMINGR |=(0xF	<<I2C_TIMINGR_SCLH_Pos);
	I2C1->TIMINGR |=(0x2	<<I2C_TIMINGR_SDADEL_Pos);
	I2C1->TIMINGR |=(0x4	<<I2C_TIMINGR_SCLDEL_Pos);
#endif
#ifdef I2C_400	
	I2C1->TIMINGR |=(0x0	<<I2C_TIMINGR_PRESC_Pos); 	//400 kHz - I2C bus speed
	I2C1->TIMINGR |=(0x9	<<I2C_TIMINGR_SCLL_Pos);
	I2C1->TIMINGR |=(0x3	<<I2C_TIMINGR_SCLH_Pos);
	I2C1->TIMINGR |=(0x1	<<I2C_TIMINGR_SDADEL_Pos);
	I2C1->TIMINGR |=(0x3	<<I2C_TIMINGR_SCLDEL_Pos);
#endif	
	I2C1->CR2 &=(~I2C_CR2_HEAD10R) & (~I2C_CR2_ADD10);
	I2C1->CR1 |=I2C_CR1_PE;
	
	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN; 								//
	GPIOA->MODER |= GPIO_MODER_MODER1_0;							//Output mode;							
	
	GPIOA->MODER &=(~GPIO_MODER_MODER0);							//Input_mode
	GPIOA->PUPDR |=GPIO_PUPDR_PUPDR0_1;								//Pull-Up

} 




int main(void)																																			{

initial();
delay_ms (100);	
LCD_Init();LCD_Clear();
LCD_Gotoxy (10,0);LCD_PrintStr(" TEST BMP180",1);
	
struct bmp180_t bmp180;
	
LCD_Gotoxy (1,1); LCD_PrintStr("ID: 0x",0);LCD_PrintHex(bmp180_Reset(),0);	
LCD_Gotoxy (64,1);LCD_PrintStr("Rev: 0x",0);LCD_PrintHex(bmp180.sensortype,0);	
	
bmp180_init(&bmp180);
	
	//-----------------------------initial data----------------------------------

while (1)  /* Main loop */
{
 if (sec05_f)											{// Run - 1 time in second
		sec05_f=0;
	 
getData(&bmp180);
	long altitude = 44330 * (1 - pow((double)(bmp180.pr_out)/101325, 0.1903));	
	 
	 LCD_Gotoxy (1,3);LCD_PrintStr("Temp.: ",0);LCD_PrintDec(bmp180.t_out,0);LCD_PrintStr(".",0);LCD_PrintDec((int32_t)(bmp180.t_out*10)%10,0);LCD_PrintStr(" C  ",0);
	 LCD_Gotoxy (1,4);LCD_PrintStr("Pres.: ",0);LCD_PrintDec((int32_t)(bmp180.pr_out),0);LCD_PrintStr(" Pa.   ",0);
	 LCD_Gotoxy (1,5);LCD_PrintStr("Pres.: ",0);LCD_PrintDec((int32_t)(bmp180.pr_out*0.00750062),0);LCD_PrintStr(" mm.hg.   ",0);
	 LCD_Gotoxy (1,7);LCD_PrintStr("Alt: ",0);LCD_PrintDec(altitude,0);LCD_PrintStr(" m   ",0);
 }		
				
		

} // end - main loop 
} // end - Main  
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{ while (1)  {  } }
#endif
