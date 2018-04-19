/**
  ******************************************************************************
  * @file     mma7660.h
	* @autor		rnm1
	* @version	V1.0
	* @date			22.02.2016
  * @brief    library file display.c
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MMA7660_H
#define __MMA7660_H

/* Includes ------------------------------------------------------------------*/	
#include "stm32f10x.h"
#include "i2c.h"

/* Private defines------------------------------------------------------------*/
/* Prototypes ------------------------------------------------------------------*/
void MMA7660init(void);
void MMA7660setMode(uint8_t mode);
void MMA7660setSampleRate(uint8_t rate);
void MMA7660getAcceleration(float *x, float *y, float *z);

#endif /*__MMA7660_H */

/********** (C) COPYRIGHT BFH TI Bienne - Mechatronics Lab *****END OF FILE****/
