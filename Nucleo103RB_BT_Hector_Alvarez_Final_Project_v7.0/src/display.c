/**
  ******************************************************************************
  * @file    display.c 
  * @author  rnm1
  * @version V1.0
  * @date    22.02.2016
  * @brief   Hardware specific display functions
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
#include "display.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void glcd_spi_write(uint8_t c)
{
	//$TASK glcd
	/* GLCD SELECT */
	GPIO_ResetBits (GPIOB, GPIO_Pin_6);
	
	/*!< Loop while DR register in not emplty */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	
	SPI_I2S_SendData(SPI1, (uint16_t) c);

	/* Wait until byte has been written */
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) != RESET);

	/* GLCD DESELECT */
	GPIO_SetBits (GPIOB, GPIO_Pin_6);

}

void glcd_reset(void)
{
	
	/* GLCD SELECT */
	GPIO_ResetBits (GPIOB, GPIO_Pin_6);
	
	/* GLCD RESET LOW() -> reset signal is at A6 */
	GPIO_ResetBits (GPIOA, GPIO_Pin_6);
	
	delay_ms(1);
	
	/* GLCD RESET HIGH() */
	GPIO_SetBits (GPIOA, GPIO_Pin_6);
	
	/* GLCD DESELECT */
	GPIO_SetBits (GPIOB, GPIO_Pin_6);
}

/**
  * @brief  Displays the current time, the temperatur and the acceleration.
  * @param  TimeVar: RTC counter value.
	* @param	TempVar: Temperatur value.
	* @param	axVar: x accleration value.
	* @param	ayVar: y accleration value.
	* @param	azVar: z accleration value.
  * @retval None
  */
void Display(uint32_t TimeVar, float TempVar, float axVar, float ayVar, float azVar)
{
	//$TASK RTC  
  /* Reset RTC Counter when Time is 23:59:59 */
  /* Compute  hours */
  /* Compute minutes */
  /* Compute seconds */
	
	glcd_clear_buffer();
	
	//$TASK RTC  
  /* write TimeVar to display */
	
	//$TASK LM75B
	/* write TempVar to display*/
	
	//$TASK MMA7660
	/* write accelerations as decimal to display*/

	/* draw bar graphs*/

	glcd_write();
}

/********** (C) COPYRIGHT BFH TI Bienne - Mechatronics Lab *****END OF FILE****/
