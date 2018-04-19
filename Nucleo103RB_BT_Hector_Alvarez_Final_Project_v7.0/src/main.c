/**
  ******************************************************************************
  * @file    main.c 
  * @author  Hector Alvarez
  * @version V4.0
  * @date    15.12.2016
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 HECTOR ALVAREZ </center></h2>
  *
  ******************************************************************************
	* @information
	*
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** Fonts */
#include "font5x7.h"

/* Private variables ---------------------------------------------------------*/


int 				button = 0;

int16_t 		pot1;
int16_t 		pot2;
float 			ax, ay, az;
int16_t 		tmp;

bool				TMP_FLAG = false;
bool				POT_FLAG = false;
bool				POT2_FLAG = false;

int 				DATACOL = 0;
int					ROW1 = 0;
int					ROW2 = 32/3*1;
int					ROW3 = 32/3*2;

int16_t 		TX_data[2] = {116,112};
char 				text_out[12] = "Temperature:";
char 				data_out[32];

/*	v3.0 Modification 	12.12.2016	*/
//char 	*text_service[35] = {'S','E','R','V','I','C','E','_','U','U','I','D','_','B','T','_','C','O','M','M','U','N','I','C','A','T','I','O','N'};
char 	CN_SR[36];
char	RD_TM[36];
char 	RD_PT[36];
char	MV_UP[36];
char	MV_DW[36];
char	MV_RG[36];
char	MV_LF[36];

/*	v4.0 Modification 	15.12.2016	*/
char 				notify_String[35];
int16_t			data_ToSend[10];
int16_t			*ptr[35];		
int16_t			READTM[10] 	= {8,2,6,8,1,1,6,1,0,9};

/*	v8.0 Modification 	18.12.2016	*/
bool				ADVERTISE = false;
struct Service_Adv {
	char CONN_SERVICE_UUID[36];
	char RDTM_SERVICE_UUID[36];
	char RDPT_SERVICE_UUID[36];
	char MVUP_SERVICE_UUID[36];
	char MVDW_SERVICE_UUID[36];
	char MVRG_SERVICE_UUID[36];
	char MVLF_SERVICE_UUID[36];
} BTservice [1] = {
	"rn42bt01-2cba-1000-8000-0006666e2cba","rn42bt02-2cba-1000-8000-0006666e2cba","rn42bt03-2cba-1000-8000-0006666e2cba",
	"rn42bt04-2cba-1000-8000-0006666e2cba","rn42bt05-2cba-1000-8000-0006666e2cba","rn42bt06-2cba-1000-8000-0006666e2cba",
	"rn42bt07-2cba-1000-8000-0006666e2cba"
};

/* Functions Declaration -----------------------------------------------------*/
void clocks 						(void);
void idle								(void);
void sendData 					(USART_TypeDef* USARTx,	int16_t *data);
void advertise_Services (void);

/* System */
static __IO uint32_t uwTimingDelay;
extern uint32_t ms_ticks;
RCC_ClocksTypeDef RCC_Clocks;

/* RTC 1/10 Second */
__IO uint32_t TimeDisplay = 0;

void clocks (void){
	
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{	
		/* SysTick end of count event each 1ms */
		RCC_GetClocksFreq(&RCC_Clocks);
		SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);
		TimeDisplay = 0;
	
		/* Hardware Configuration and Initialisation */
		RCC_configuration();
		GPIO_configuration();
		ADC_configuration();
		SPI_configuration();
		I2C_configuration();
		DMA_configuration();
		BT_configuration();
		Communication_interrupt_config();
	
		/* Display Configuration and Initialisation */
		glcd_select_screen ((uint8_t *)&glcd_buffer, &glcd_bbox);
		glcd_reset();
		glcd_ST7565R_init();
		glcd_tiny_set_font (Font5x7,5,7,32,127);			
		
		//$TASK RTC
		RTC_initialisation();
		
		/*	Wait till Button Pressed to Start */
		
		idle();			
		
		while (1)
		{
		if (is_Reset_BT())
			idle();
		
		if (!is_Receiving_BT())
		{
			delay_ms(3000);
			if (TimeDisplay == 1 && !is_Receiving_BT())
			{
				//$TASK RTC
				/* get temperature data */
				Display(RTC_GetCounter(),readTemperatur(), ax, ay, az);	
				
				sprintf (text_out, "%4s", "Temp:");//%4s - string
				glcd_draw_string_xy (DATACOL,ROW1,text_out);			
				
				tmp = Temp_read();
				if (tmp & (1<<10) ) tmp |= 0xFC00;
				float temp = (float) tmp * 0.125;
				
				if (tmp != 0) {
					sprintf (text_out, "%4f",temp);//%4f - float
					glcd_draw_string_xy(DATACOL+40, ROW1,text_out);
					TMP_FLAG = true;
				}
				else {
					sprintf (text_out, "%4s","Tmp not read");//%4f - float
					glcd_draw_string_xy(DATACOL+40, ROW1,text_out);
					TMP_FLAG = false;
				}
				//$TASK ADC
				/* get Potentionmeters data */
				sprintf (text_out, "%4s", "Pot1:");//%4s - string
				glcd_draw_string_xy (DATACOL,ROW2,text_out);
				pot1 = Pot1_value()/41;//Divided by 41 to give a percentage 0-100%
		
				if (pot1 != 0) {
					sprintf (text_out, "%4d",pot1);//%4f - float
					glcd_draw_string_xy(DATACOL+30, ROW2,text_out);
					POT_FLAG = true;
				}
				else {
					sprintf (text_out, "%4s","Pot not read");//%4f - float
					glcd_draw_string_xy(DATACOL+30, ROW2,text_out);
					POT_FLAG = false;
				}
				sprintf (text_out, "%4s", "Pot2:");//%4s - string
				glcd_draw_string_xy (DATACOL,ROW3,text_out);
				pot2 = Pot2_value()/41;//Divided by 41 to give a percentage 0-100%
					
				if (pot2 != 0) {
					sprintf (text_out, "%4d",pot2);//%4f - float
					glcd_draw_string_xy(DATACOL+30, ROW3,text_out);
					POT2_FLAG = true;
				}
				else {
					sprintf (text_out, "%4s","Pot not read");//%4f - float
					glcd_draw_string_xy(DATACOL+30, ROW3,text_out);
					POT2_FLAG = false;
				}
			}
			
			if (TM_Autentified() && !is_Receiving_BT())
			{
				/*		TEMPERATURE TRANSMISSION & POTENTIOMETER TRANSMISSION
				*			Send character 't' - ASCI 116 to announce read temperature
				*			Send temperature value
				*			Send character 'p' - ASCI 112 to announce read potentiometer
				*			Send potentiometer value
				*/
				
				if (	BT_GetFlagStatus('u')){BT_Clear_Flag('u');delay_ms(1000);}
				else if (	BT_GetFlagStatus('d')){BT_Clear_Flag('d');delay_ms(1000);}
				else if (	BT_GetFlagStatus('r')){BT_Clear_Flag('r');delay_ms(1000);}
				else if (	BT_GetFlagStatus('l')){BT_Clear_Flag('l'); delay_ms(1000);}
				sprintf (text_out, "%4s","TM CONN");//%4f - float
				glcd_draw_string_xy(DATACOL+60, ROW2,text_out);
			}
			else {
				sprintf (text_out, "%4s","TM DISCONN");//%4f - float
				glcd_draw_string_xy(DATACOL+60, ROW2,text_out);
			}
			
			if (BT_Autentified() && !is_Receiving_BT())
			{
				ADVERTISE  = true;//Adevertise BT services
				/*		TEMPERATURE TRANSMISSION & POTENTIOMETER TRANSMISSION
				*			Send character 't' - ASCI 116 to announce read temperature
				*			Send temperature value
				*			Send character 'p' - ASCI 112 to announce read potentiometer
				*			Send potentiometer value
				*/
			
				if 			(	BT_GetFlagStatus('t')){startTransmission_BT(); printf("%d,%d",TX_data[0],tmp); BT_Clear_Flag('t'); delay_ms(2000);}
				else if (	BT_GetFlagStatus('p')){startTransmission_BT(); printf("%d,%d",TX_data[1],pot1);BT_Clear_Flag('p'); delay_ms(2000);}
				else if (	BT_GetFlagStatus('u')){BT_Clear_Flag('u'); delay_ms(2000);}
				else if (	BT_GetFlagStatus('d')){BT_Clear_Flag('d'); delay_ms(2000);}
				else if (	BT_GetFlagStatus('r')){BT_Clear_Flag('r'); delay_ms(2000);}
				else if (	BT_GetFlagStatus('l')){BT_Clear_Flag('l'); delay_ms(2000);}
				sprintf (text_out, "%4s","BT CONN");//%4f - float
				glcd_draw_string_xy(DATACOL+60, ROW3,text_out);
				
			}
			else {
				sprintf (text_out, "%4s","BT DISCONN");//%4f - float
				glcd_draw_string_xy(DATACOL+60, ROW3,text_out);
			}
			if (BT_GetFlagStatus('w'))
			{
				GPIOB->ODR ^= (1 << 4); 
				GPIOC->ODR ^= (1 << 7); 
				GPIOA->ODR ^= (1 << 9); 
				BT_Clear_Flag('w'); 
				delay_ms(2000);
			}
			glcd_write();
			TimeDisplay = 0;
			
			if (ADVERTISE) advertise_Services();
		}
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
	TimeDisplay = 1;
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (uwTimingDelay != 0x00)uwTimingDelay--;
}

/*	v3.0 Modification 	12.12.2016	*/
/*******************************************************************************
* Function Name  : idle
* Description    : Waits till Button is pressed
* Input          : None
* Output         : None
* Return         : void
*******************************************************************************/
void idle (){
	while(button==0) button = GPIOB ->IDR & (1 << 5);
	/*	Set BT Module to enable Advertising */
	activate_BT();
	button = 0;
}

/*	v3.0 Modification 	12.12.2016	*/
/*******************************************************************************
* Function Name  : advertise_Services
* Description    : Activates BT Module
* Input          : None
* Output         : None
* Return         : void
*******************************************************************************/
void 		advertise_Services (void){
	
	struct Service_Adv *service = BTservice;
	printf ("\n");
	//printf ("%s",CN_SR);
	printf ("%s",service->CONN_SERVICE_UUID);
	printf ("\n");
	printf ("Service Selected: ");
	delay_ms(10000);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
