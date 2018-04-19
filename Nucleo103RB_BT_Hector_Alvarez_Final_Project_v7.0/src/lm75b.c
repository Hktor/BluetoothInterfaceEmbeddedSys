/**
  ******************************************************************************
  * @file    lm75b.c 
  * @author  rnm1
  * @version V1.0
  * @date    23.02.2016
  * @brief   LM75B temperature library functions
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 BFH TI Bienne - rnm1 </center></h2>
  *
  ******************************************************************************
	* @information
	*
	*
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "lm75b.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//$TASK LM75B
#define LM75B_ADDR  0x90
#define LM75B_TEMP  0x00 //Temperature register (RO)
#define LM75B_CONF  0x00 //Configuration register (R/W)
#define LM75B_THYST 0x00 //Overtemperature shutdown threshold register (R/W)
#define LM75B_TOS   0x00 //Hysteresis register (R/W)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Reads the temperatur from LM75B over I2C
  * @param  None
  * @retval Temperatur
  */
	
	
float readTemperatur(void)
{
	int16_t tmp;
	
	//$TASK LM75B
	tmp = I2C_Write_1_Read_2_byte(I2C1, LM75B_ADDR, LM75B_TEMP) >> 5;
	if (tmp & (1<<10) ) tmp |= 0xFC00;
	float temp = (float) tmp * 0.125;
	
	return temp;
}

/********** (C) COPYRIGHT BFH TI Bienne - Mechatronics Lab *****END OF FILE****/
