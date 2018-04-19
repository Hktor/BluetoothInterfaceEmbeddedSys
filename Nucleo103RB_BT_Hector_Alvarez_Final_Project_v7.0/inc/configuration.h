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

void ADC_configuration(void);
void SPI_configuration(void);
void I2C_configuration(void);
void RTC_configuration(void);
void RCC_configuration (void);
void GPIO_configuration (void);
void NVIC_configuration (void);
void USART_configuration(void);
void DMA_configuration (void);
int  Pot1_value(void);
int  Pot2_value(void);
#endif /*__CONFIGURATION_H */

/********** (C) COPYRIGHT BFH TI Bienne - Mechatronics Lab *****END OF FILE****/
