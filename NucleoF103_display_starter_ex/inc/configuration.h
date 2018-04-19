/**
  ******************************************************************************
  * @file     configuration.h
	* @autor		rnm1
	* @version	V1.0
	* @date			22.02.2016
  * @brief    library file configuration.c
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONFIGURATION_H
#define __CONFIGURATION_H

/* Includes ------------------------------------------------------------------*/	
#include "stm32f10x.h"

/* Private defines------------------------------------------------------------*/
/* Prototypes ------------------------------------------------------------------*/
void RCC_configuration(void);
void GPIO_configuration(void);
void ADC_configuration(void);
void SPI_configuration(void);
void I2C_configuration(void);
void RTC_configuration(void);
void NVIC_configuration(void);
void Beep_configuration(void);
void DMA_configuration(void);

int get_Poti1(void);
int get_Poti2(void);

#endif /*__CONFIGURATION_H */

/********** (C) COPYRIGHT BFH TI Bienne - Mechatronics Lab *****END OF FILE****/
