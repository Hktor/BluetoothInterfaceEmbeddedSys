/**
  ******************************************************************************
  * @file     BT.h
	* @autor		Hector Alvarez
	* @version	V1.0
	* @date			20.10.2016
  * @brief    library file BT.c
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BT_H
#define __BT_H

/* Includes ------------------------------------------------------------------*/	
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "string.h"
#include "configuration.h"
#include "i2c.h"
#include <stdio.h>
#include "stdbool.h"

/* Definitions for BT device to be accessed----------------------------------*/
#define BT_CLOCK_FREQ		400000			// 400 kHz
#define TIMEOUT					1833+137*72	// 1833 + 137 multiplied by systemclock in MHz


/* UART3 - BT Communication*/
/* Default */
/*		Pin name				Type				Main function					Alternate functions
																	(after reset)					Default
-------------------------------------------------------------------------------																												
			PB12						I/O					PB12									SPI2_NSS/USART3_CK/
																												2C2_SMBAl/TIM1_BKIN(8)
-------------------------------------------------------------------------------																												
			PB10						I/O					PB10									I2C2_SCL/USART1_TX(9)
-------------------------------------------------------------------------------																	
			PB11						I/O					PB11									I2C2_SDA/USART2_RX(9)
-------------------------------------------------------------------------------																												
			PB13						I/O					PB13									SPI2_SCK/USART1_CTS(9)
																												/TIM1_CH1N
-------------------------------------------------------------------------------																												
			PB14						I/O					PB14									SPI2_MISO/USART1_RTS(9)
																												/TIM1_CH2N(8)		
-------------------------------------------------------------------------------				
*
*	Remapped
---------------------------------------------------------------------------	
			PC12						I/O				PC12									USART3_CK
---------------------------------------------------------------------------	
			PC10						I/O				PC10									USART1_TX
---------------------------------------------------------------------------	
			PC11						I/O				PC11									USART2_RX
---------------------------------------------------------------------------	

*/
//Changed for USART 3 check datasheet for Pin
#define BT_GPIO_PARTIALREMAP								GPIOC	
#define BT_GPIO_PARTIALREMAP_PIN_CK					GPIO_Pin_12					//USART3_CK(8)
#define BT_GPIO_PARTIALREMAP_PIN_TX					GPIO_Pin_10					//USART3_TX(8)
#define BT_GPIO_PARTIALREMAP_PIN_RX					GPIO_Pin_11					//USART3_RX(8)

#define BT_GPIO_NOREMAP										GPIOB	
#define BT_GPIO_NOREMAP_PIN_TX						GPIO_Pin_10					//USART3_TX(8)
#define BT_GPIO_NOREMAP_PIN_RX						GPIO_Pin_11					//USART3_RX(8)
#define BT_GPIO_NOREMAP_PIN_CK						GPIO_Pin_12					//USART3_CK(8)
#define BT_GPIO_NOREMAP_PIN_CTS						GPIO_Pin_13					//USART3_CTS(8)
#define BT_GPIO_NOREMAP_PIN_RST						GPIO_Pin_14					//USART3_RTS(8)

/* UART2 - Terminal Communication*/
/* Default */
/*		Pin name				Type				Main function					Alternate functions
																	(after reset)					Default
---------------------------------------------------------------------------																												
			PA0							I/O					PA0										WKUP/
																												USART2_CTS/
																												ADC12_IN0/
																												TIM2_CH1_ETR(8)
---------------------------------------------------------------------------																												
			PA1							I/O					PA1										USART2_RTS(8)/				Using the MBED PA10 is used to reset the BT Module
																												ADC12_IN1/
																												TIM2_CH2(8)
---------------------------------------------------------------------------																	
			PA2							I/O					PA2										USART2_TX(8)/
																												ADC12_IN2/
																												TIM2_CH3(8)
---------------------------------------------------------------------------																												
			PA3							I/O					PA3										USART2_RX(8)/
																												ADC12_IN3/
																												TIM2_CH4(8)			
---------------------------------------------------------------------------																												
			PA4							I/O					PA4										SPI1_NSS(8)/
																												USART2_CK(8)/
																												ADC12_IN4																													
*/
#define TERMINAL_GPIO									GPIOA	
#define TERMINAL_GPIO_PIN_CTS					GPIO_Pin_0					//USART2_CTS(8)
#define TERMINAL_GPIO_PIN_NRST				GPIO_Pin_10					//USART2_RTS(8)
#define TERMINAL_GPIO_PIN_TX					GPIO_Pin_2					//USART2_TX(8)
#define TERMINAL_GPIO_PIN_RX					GPIO_Pin_3					//USART2_RX(8)
#define TERMINAL_GPIO_PIN_CK					GPIO_Pin_4					//USART2_CK(8)

/* Types ---------------------------------------------------------------------*/
/* Function prototypes -------------------------------------------------------*/
void 			BT_configuration			(void);

void 			End_communication			(void);//Sleep mode
void 			SER_Initialize 				(void);
int 			SER_PutChar 					(int ch);
int 			SER_GetChar 					(void);

void 			TERMINAL_IRQHandler		(void);
void 			BT_IRQHandler					(void);
void			USART3_IRQHandler			(void);
void 			BT_Write							(void);
int16_t 	Temp_read							(void);

void 			Authentication_Reset 	(void);
bool 			TM_Autentified 				(void);
bool 			BT_Autentified 				(void);

void 			Clear_Flag_Wng_Pwd 		(void);
void 	 	 	BT_Clear_Flag 				(char Flag);
bool 	  	BT_GetFlagStatus 			(char Flag);

bool 			Wrong_password_colour_change 			(void);
void 			Communication_interrupt_config		(void);

/*	v3.0 Modification 	12.12.2016	*/
void			disconnect						(void);
bool 			is_Reset_BT 					(void);
void 			activate_BT 					(void);

/*	v4.0 Modification 	15.12.2016	*/
int 			SendChar 							(int c);
void 			startTransmission_BT 	(void);
bool 			is_Receiving_BT 			(void);

/*	v5.0 Modification 	16.12.2016	*/
int 			BT_PutChar 						(int ch);
void			notify_Observer				(void);

#endif /*__BT_H */

/********** (C) COPYRIGHT HECTOR ALVAREZ *****END OF FILE****/
