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

/* Private typedef -----------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address    ((u32)0x4001244C)	  // Adress ADC Data Register
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
volatile uint16_t ADC_Value[2];					// Array of two values

void RCC_configuration(void)
{
	//$TASK SPI
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	//$TASK RTC

	
	//$TASK ADC
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	
	
	//$TASK I2C
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	//Achtung APB1 has to be enabeld
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
	
	//$TASK DMA double poti
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	


}

void GPIO_configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  
	//$TASK SPI
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	
	//ssPin (PB6)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//set GPIOA pin 6 to disable
	//GPIO_SetBits(GPIOA,GPIO_Pin_6); 
	GPIO_SetBits(GPIOB,GPIO_Pin_6);
	
	//Reset Pin (PA6, DC pin (PA8)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_8;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//I2C Config
		//Pin config
	//D14 SDA -> PB9
	//D15 SCL -> PB8
	//I2C1
	//Remap -> We need to tell that we use PB8 and PB9
	GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
		
			//ADC PINS
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/**RGB LED Pin on mbed Blue D8: PA9, Green D9: PC7, Red D5 PB4**/

	//PB4 Needs to be remaped (Check in the datasheet for default funtion)
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST,ENABLE);
	/****************PA9**************/
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/****************PC7**************/
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	/****************PB4**************/
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//Joustick pins Right A5: PC0 , Up A2: PA4, Center D4: PB5, Left A4: PC1, Down A3: PB0

		/****************PC0**************/
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	/****************PA4**************/
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/****************PB5**************/
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/****************PC1**************/
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	/****************PB0**************/
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
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
	
	 //ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1,ADC_SampleTime_71Cycles5);
	 ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	

	
 	 ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_71Cycles5);
	 ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_71Cycles5);  
	
 	 /* Enable ADC1 DMA */
	 ADC_DMACmd(ADC1, ENABLE); 
}

void SPI_configuration(void)
{
	
	SPI_InitTypeDef SPI_InitStructure;
	
	SPI_I2S_DeInit(SPI1);
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1,&SPI_InitStructure);
	//activate SPI
	SPI_Cmd(SPI1,ENABLE);	
}

void I2C_configuration(void)
{
	
	
	//Configuration
	I2C_InitTypeDef  I2C_InitStructure;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x3C;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed =  400000;
	I2C_Init(I2C1,&I2C_InitStructure);
	I2C_Cmd(I2C1,ENABLE);

}

void RTC_configuration(void)
{	
	//$TASK RTC
	

}

void NVIC_configuration(void)
{
  // NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	//$TASK RTC

}

void Beep_configuration(void)
{
// Variables
GPIO_InitTypeDef GPIO_InitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBase_InitStructure;
TIM_OCInitTypeDef TIM_OC_InitStructure;
 
// Clock
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
 
// GPIO
GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, ENABLE);
	
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOB, &GPIO_InitStructure);
 
// Timer
TIM_TimeBase_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;// TIM_CKD_DIV1;
TIM_TimeBase_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
TIM_TimeBase_InitStructure.TIM_Period = 1000-1; // Prescaler / 1000 -> 1ms
TIM_TimeBase_InitStructure.TIM_Prescaler = 72-1; // Assuming 72MHz clock -> 1MHz
TIM_TimeBaseInit(TIM2, &TIM_TimeBase_InitStructure);
 
// Timer Output Compare Mode PWM
TIM_OC_InitStructure.TIM_OCMode = TIM_OCMode_PWM1;
TIM_OC_InitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
TIM_OC_InitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
TIM_OC_InitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
TIM_OC_InitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
TIM_OC_InitStructure.TIM_OutputState = TIM_OutputState_Enable;
TIM_OC_InitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
TIM_OC_InitStructure.TIM_Pulse = 100; // Dutycycle = Pulse/Period -> 10%
TIM_OC3Init(TIM2, &TIM_OC_InitStructure);
 
TIM_Cmd(TIM2, ENABLE);
	
}

void DMA_configuration(void){


  DMA_InitTypeDef  DMA_InitStructure;
	
	/* DMA1 channel1 configuration ----------------------------------------------*/

	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) ADC1_DR_Address;			// Define the peripheral base address for DMA1 channel1
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_Value;		// Define the memory base address for DMA1 channel1
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;							// Peripheral is the source
	DMA_InitStructure.DMA_BufferSize = 2;  // 1
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  // Peripheral address register not incremented
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;				    // Memory address is incremented
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	// Data width = 16 bits
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;		   	
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;								// Circular buffer mode is used
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;	 							// t DMA channel not configured for memory-to memory transfer 
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	/* Enable DMA1 channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);

}

int get_Poti1(void) {
	return ADC_Value[1];
}

int get_Poti2(void) {
	return ADC_Value[0];
}
