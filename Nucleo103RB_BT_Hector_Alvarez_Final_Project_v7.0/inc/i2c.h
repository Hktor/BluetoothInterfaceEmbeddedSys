/******************************************************************************
* @file			I2C.h
* @brief    I2C Routines
* @author   BFH-TI / hul1
* @version  V1.2
* @date     23.06.2014
* @hardware MCBSTM32C & STM32-P103 @72MHz
*******************************************************************************/
/*
  V1.2 23.06.2014
                  Replace SysTick timeout by hardcoded loop timeout
                  Add I2C Status enum to know about communication success
                  Replace datatypes by unsigned types where applicable
                  Add I2C_GetStatus function
  V1.1 07.05.2012

*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H
#define __I2C_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Definitions for I2C device to be accessed----------------------------------*/
#define I2C_CLOCK_FREQ	400000			// 400 kHz
#define TIMEOUT					1833+137*72	// 1833 + 137 multiplied by systemclock in MHz

/* I2C1 */
#define I2C1_GPIO										GPIOB
#define I2C1_GPIO_PIN_SCL						GPIO_Pin_6
#define I2C1_GPIO_PIN_SDA						GPIO_Pin_7
#define I2C1_REMAP_GPIO_PIN_SCL			GPIO_Pin_8
#define I2C1_REMAP_GPIO_PIN_SDA			GPIO_Pin_9
#define RCC_APBPeriphClockCmd_I2C1	RCC_APB2PeriphClockCmd
#define RCC_APB2Periph_I2C1_IO			RCC_APB2Periph_GPIOB

/* I2C2 */
#define I2C2_GPIO										GPIOB
#define I2C2_GPIO_PIN_SCL						GPIO_Pin_10
#define I2C2_GPIO_PIN_SDA						GPIO_Pin_11
#define RCC_APBPeriphClockCmd_I2C2	RCC_APB2PeriphClockCmd
#define RCC_APB2Periph_I2C2_IO			RCC_APB2Periph_GPIOB

/* Remaps */
#define USE_I2C1_Remap
#define Remap_I2C1				GPIO_Remap_I2C1

/* Types ---------------------------------------------------------------------*/
/**
  * @brief  Returns information about I2C Communication Errors
  * @param  I2C_FAULT: specifies the error
  *   This parameter can be one of the following values:
  *     @arg NONE: No error occured
  *     @arg SCF: Start condition failure
  *     @arg ADNM: Address not matched to any slave
  *     @arg B1NT: Byte 1 not transmitted
  *     @arg B2NT: Byte 2 not transmitted
  *     @arg B3NT: Byte 3 not transmitted
  *     @arg B4NT: Byte 4 not transmitted
  *     @arg BxNT: Byte 5 or higher not transmitted
  *     @arg B1NR: Byte 1 not received
  *     @arg B2NR: Byte 2 not received
  *     @arg B3NR: Byte 3 not received
  *     @arg B4NR: Byte 4 not received
  *     @arg BxNR: Byte 5 or higher not received
  *     @arg BTF: Byte transfer finished
  *     @arg BRD: Byte receive finished
  */
enum I2CState { NONE, SCF, ADNM, B1NT, B2NT, B3NT, B4NT, BxNT, B1NR, B2NR, B3NR, B4NR, BxNR, BTF, BRF };

/* Function prototypes -------------------------------------------------------*/
void I2C_Configuration(I2C_TypeDef * I2C);

u8 I2C_Read_1_byte(I2C_TypeDef * I2Cx, u8 address);
u16 I2C_Read_2_byte(I2C_TypeDef * I2Cx, u8 address);
u32 I2C_Read_3_byte(I2C_TypeDef * I2Cx, u8 address);
void I2C_Read_n_byte(I2C_TypeDef * I2Cx, u8 address, u8 n, u8 * buffer);

void I2C_Write_1_byte(I2C_TypeDef * I2Cx, u8 address, u8 data1);
void I2C_Write_2_byte(I2C_TypeDef * I2Cx, u8 address, u8 data1, u8 data2);
void I2C_Write_3_byte(I2C_TypeDef * I2Cx, u8 address, u8 data1, u8 data2, u8 data3);
void I2C_Write_n_byteNS(I2C_TypeDef * I2Cx, u8 address, u8 n, u8 * data);
void I2C_Write_n_byte(I2C_TypeDef * I2Cx, u8 address, u8 n, u8 * data);

u8 I2C_Write_1_Read_1_byte(I2C_TypeDef * I2Cx, u8 address, u8 data1);
u16 I2C_Write_1_Read_2_byte(I2C_TypeDef * I2Cx, u8 address, u8 data1);
void I2C_Write_1_Read_n_byte(I2C_TypeDef * I2Cx, u8 address, u8 data1, u8 n, u8 * buffer);

void I2C_SetState(I2C_TypeDef* I2Cx, enum I2CState state);
enum I2CState I2C_GetState(I2C_TypeDef* I2Cx);

#endif /* __I2C_H */

/******END OF FILE****/
