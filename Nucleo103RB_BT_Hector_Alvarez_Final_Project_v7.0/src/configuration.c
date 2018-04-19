/**
  ******************************************************************************
  * @file    configuration.c 
  * @author  rnm1
  * @version V1.0
  * @date    22.02.2016
  * @brief   Setup Hardware
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
#include "configuration.h"
#include "stm32f10x_spi.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address    ((u32)0x4001244C)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint16_t ADC_Value[2];
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*	Reset & Clock Control - RCC	*/
void RCC_configuration(void) {
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
}

void GPIO_configuration(void) {
 	GPIO_InitTypeDef GPIO_InitStructure;
  
	/* PA9 & PA7 - Output Blue LED for the display*/  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //push-pull
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	/* Reset - PA6 / DC - PA8) */	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //push-pull
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_8;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* SS - PB6 */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;	
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB, GPIO_Pin_6);
  
	/*SPI pins (SCK - PA5, MOSI - PA7) */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_5;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
		
	//ADC1 PINS
	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//I2C Pins
	GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE); 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* Configure PB5 as  input for button reading */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
}

void RTC_configuration(void)
{	
	//$TASK RTC
}

void SPI_configuration(void)
{
	SPI_InitTypeDef SPI_InitStructure;
	SPI_I2S_DeInit (SPI1);
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	
	/*INITIALIZE*/
	SPI_Init (SPI1, &SPI_InitStructure);
	
	/*ENABLE SPI1*/
	SPI_Cmd (SPI1, ENABLE);
}


void I2C_configuration(void)
{
	I2C_InitTypeDef  I2C_InitStructure;
	
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x3C;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 400000;
	
	I2C_Init(I2C1, &I2C_InitStructure);
	I2C_Cmd(I2C1, ENABLE);
}	

void ADC_configuration(void)
{
	ADC_InitTypeDef  ADC_InitStructure;
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	
	ADC_DeInit(ADC1);
	
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
		//For DMA
	 ADC_InitStructure.ADC_NbrOfChannel = 2;
	
	/*ENABLE ADC1*/
	ADC_Init (ADC1, &ADC_InitStructure);
	ADC_Cmd(ADC1,ENABLE);
	
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
	
	 ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	
 	 ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_71Cycles5);
	 ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_71Cycles5);  
	
 	 /* Enable ADC1 DMA */
	 ADC_DMACmd(ADC1, ENABLE); 
}

void DMA_configuration(void){


  DMA_InitTypeDef  DMA_InitStructure;
	
	/* DMA1 channel1 configuration ----------------------------------------------*/

	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) ADC1_DR_Address;			// Define the peripheral base address for DMA1 channel1
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_Value;								// Define the memory base address for DMA1 channel1
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;													// Peripheral is the source
	DMA_InitStructure.DMA_BufferSize = 2;  // 1
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  					// Peripheral address register not incremented
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;				    					// Memory address is incremented
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	// Data width = 16 bits
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;		   	
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;															// Circular buffer mode is used
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;	 															// t DMA channel not configured for memory-to memory transfer 
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	/* Enable DMA1 channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);

}

int Pot1_value(void) {
	return ADC_Value[0];
}

int Pot2_value(void) {
	return ADC_Value[1];
}

