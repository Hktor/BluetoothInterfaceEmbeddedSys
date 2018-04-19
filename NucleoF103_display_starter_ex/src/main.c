/**
  ******************************************************************************
  * @file    main.c 
  * @author  aal1
  * @version V1.0
  * @date    30.09.2015
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Yann Schaeffer - aal1 </center></h2>
  *
  ******************************************************************************
	* @information
	* This progamm allows to set the frequency and duty cycle with the terminal of
	* the computer. Furter the RGB LED can be controlled as well (toggel)
	* RGB: type into terminal: r g b -> r = red, g = green , b = blue
	* Buzzer: type into Terminal: f1200; d55; (f = frequ (Hz), d = duty (%)) 
	* Can alsow be written separatly (freq or duty)
	* Always delimiter ; behind the number to terminate
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <math.h>
#include "Serial.h"
#include "stdbool.h"
/* Fonts */
#include "font5x7.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* System */
static __IO uint32_t uwTimingDelay;
extern uint32_t ms_ticks;
RCC_ClocksTypeDef RCC_Clocks;

/* RTC 1/10 Second */
__IO uint32_t TimeDisplay = 0;


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
	
	
int mapValue(int input_start, int input_end,int output_start,int output_end,int input);
double round(double val);


#define RED_LED_PIN GPIO_Pin_4
#define RED_LED_PORT GPIOB

#define GREEN_LED_PIN GPIO_Pin_7
#define GREEN_LED_PORT GPIOC

#define BLUE_LED_PIN GPIO_Pin_9
#define BLUE_LED_PORT GPIOA

  volatile char dutyReceive[15] = "";
	volatile char frequReceive[15] = "";
	volatile bool newData = false; //indicates if there was new value for duty or freq
	volatile bool newFrequ = false;
	volatile bool newDuty = false;

	


int main(void)
{
  /* SysTick end of count event each 1ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency/1000);
	
	/* Hardware Configuration and Initialisation */
  RCC_configuration();
	GPIO_configuration();
	ADC_configuration();
	SPI_configuration();
	I2C_configuration();
	NVIC_configuration();
	Beep_configuration();
	DMA_configuration();
	UART_config();
	UART_interrupt_config();
	
	
	// glcd
	glcd_select_screen((uint8_t *)&glcd_buffer,&glcd_bbox);
	glcd_reset();
	glcd_ST7565R_init();

	glcd_tiny_set_font(Font5x7, 5,7,32,127);
	glcd_clear_buffer();
	glcd_draw_string_xy(0,0,"Good morning Yann!");
	glcd_write();
	
	// RTC
	RTC_initialisation();
	
    u8 increment = 0;

		glcd_clear();
		glcd_draw_string_xy(0,0,"Frequency:");
		glcd_draw_string_xy(110 , 0,"Hz");
		glcd_draw_string_xy(0,10,"Duty Cycle:");
		glcd_draw_string_xy(110 , 10,"%");
		glcd_draw_string_xy(0,20,"Temp:");
		glcd_draw_string_xy(110		, 20,"C");
		
	
	while(1) {
		
		delay_ms(20);	
		static double dutyCycle = 0; //in %
		
    //handel new data received trough serial port
		if(newData){
			newData = false;
			
			if(newDuty){
				newDuty = false;
				//convert duty string to number
				char *ptr;
				char *ptrTempDuty = (char*)&dutyReceive;
				int tempDuty = 0;
				tempDuty = (int) strtol(ptrTempDuty,&ptr,10);
				//write new duty cycle
				if(tempDuty <= 100 && tempDuty >= 0) {
						dutyCycle = tempDuty;
				}
			  //delet array for new data
				for(int i = 0;i<sizeof(dutyReceive);++i){
						dutyReceive[i] = ' ';
				}
			}	
			if(newFrequ){
				newFrequ = false;
				//convert frequency string to number
				char *ptr;
				char *ptrTempFrequ = (char*)&frequReceive;
				int tempFrequ = (int)strtol(ptrTempFrequ,&ptr,10);
				if(tempFrequ >= 0 && tempFrequ <= 0x0FFF){
						//write frequency
						uint16_t tempARR = 1000000/tempFrequ;//(uint16_t) round(1000000.0/tempFrequ);
						TIM2->ARR = tempARR;
				}
				//delet array for new data
				for(int i = 0;i<sizeof(frequReceive);++i){
						frequReceive[i] = ' ';
				}
			}
		}	
		//ajust duty cycle to frequ
		double tempARRCalc = TIM2->ARR;
		int freq = 1000000/TIM2->ARR ;
		TIM2->CCR3 =  dutyCycle/100*tempARRCalc;
	  //Writ it to the screen
		glcd_draw_string_xy(60, 0,"        ");
		char 	text1[12] = " ";
		//Write Frequency
		sprintf (text1, "%4d",freq);
		glcd_draw_string_xy(65, 0,text1);
		//Write Duty Cycle
		sprintf (text1, "%4d",(int)dutyCycle);
		glcd_draw_string_xy(65, 10,text1);
		//Write Temp		
		sprintf (text1, "%4f",readTemperatur());
		glcd_draw_string_xy(40, 20,text1);
		glcd_write();
	}
}


/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void delay_ms(__IO uint32_t nTime)
{ 
  uwTimingDelay = nTime;
  while(uwTimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (uwTimingDelay != 0x00)
  { 
    uwTimingDelay--;
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

	int mapValue(int input_start, int input_end, int output_start, int output_end,int input){
		double slope = 1.0 * (output_end-output_start)/(input_end-input_start);
		return output_start + round(slope*(input - input_start));
	}
	
	double round(double d){
		return floor(d + 0.5);
	}
	
	/**
  * @brief  Handels received data of the serial port
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void){
	  
	/*
  if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET) // Transmit the string in a loop
  {
    USART_SendData(USART2, StringLoop[tx_index++]);
   
    if (tx_index >= (sizeof(StringLoop) - 1))
      tx_index = 0;
  }
  
		
	
  if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) // Received characters modify string
  {
    StringLoop[rx_index++] = USART_ReceiveData(USART2);
   
    if (rx_index >= (sizeof(StringLoop) - 1))
      rx_index = 0;
  }
	
	printf(StringLoop);*/
	
	
	static u8 dutyIncrement = 0;
	static u8 frequIncrement = 0;
	static bool nextDuty = false;
  static bool nextFrequ = false;

if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET){
	//get the char that is in the input buffer
	char tempChar = SER_GetChar();
	
	if(tempChar == 'd'){
		nextDuty = true;
	}
	else if(tempChar == 'f'){
		nextFrequ = true;
	}
	else if(tempChar == ';'){
		//ending the transmition of the number with the delimiter ;
		dutyIncrement = 0;
		frequIncrement = 0;
		nextDuty = false;
		nextFrequ = false;
		newData = true;
	}
	
	if(nextDuty && tempChar >= '0' && tempChar <= '9'){
		//write to dutycycle array
		dutyReceive[dutyIncrement++] = tempChar;
		newDuty = true;
	}
	else if(nextFrequ && tempChar >= '0' && tempChar <= '9'){
		//write to frequency array
		frequReceive[frequIncrement++] = tempChar;
		newFrequ = true;
		
	}
	
	/**********toggle the rgb led with input of rgb from the computer***********************************************************/
	if((tempChar == 'r' || tempChar == 'g' || tempChar == 'b') && (!nextDuty || !nextFrequ)){
		if(tempChar == 'r'){
				//toggle red led
				GPIOB->ODR ^= (1 << 4);
		}
		else if(tempChar == 'g'){
				//toggle green led
				GPIOC->ODR ^= (1 << 7);
		}
		else if(tempChar == 'b'){
				//toggle blue
				GPIOA->ODR ^= (1 << 9);
		}
	}
}
}





/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


