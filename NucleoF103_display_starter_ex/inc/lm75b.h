/**
  ******************************************************************************
  * @file     lm75b.h
	* @autor		rnm1
	* @version	V1.0
	* @date			23.02.2016
  * @brief    library file lm75b.c
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LM75B_H
#define __LM75B_H

/* Includes ------------------------------------------------------------------*/	
#include "stm32f10x.h"
#include "i2c.h"

/* Private defines------------------------------------------------------------*/
/* Prototypes ------------------------------------------------------------------*/
float readTemperatur(void);

#endif /*__LM75B_H */

/********** (C) COPYRIGHT BFH TI Bienne - Mechatronics Lab *****END OF FILE****/
