/******************************************************************************
* @file			BT.c
* @brief    BT Routines
* @author   Hector Alvarez
* @version  V4.0
* @date     15.12.2016
* @hardware MCBSTM32C & STM32-P103 @72MHz
*******************************************************************************/
/*
  V4.0 15.12.2016
                  Add Service Adverticement to communicate with another device through BT
                  Add BT_Connection Status enum to know about communication success
                  Add BT_GetStatus function
									Add function void disconnect (void) to finish communication
*/

/* Includes ------------------------------------------------------------------*/
#include "BT.h"

/* Variables -----------------------------------------------------------------*/

uint16_t	TM_Tx_Data;
uint16_t	TM_Rx_Data;
uint16_t	BT_Rx_Data;
uint16_t	BT_Tx_Data;

bool 			connect = false;
bool 			connectBT = false;
bool 			authentication[6] = {false,false,false,false,false,false};
bool 			TM_CONNECT = false;
bool 			BT_CONNECT = false;
bool 			MOVE_RIGHT = false;
bool 			MOVE_LEFT  = false;
bool 			MOVE_UP 	 = false;
bool 			MOVE_DOWN  = false;
bool 			WRONG_PWD  = false;

/*	v3.0 Modification 	12.12.2016	*/
char			notifyString[25]; //String made of 32 characters to advertise information
char			dataReceived[4]; //String made of 10 characters to receive information
int				connectionTrialCount = 0;
u16				Communication_Pin_Reset;
bool			RSTBT = false;

/*	v4.0 Modification 	15.12.2016	*/
int		countDataReceived = 0;
bool	serviceFound		= false;
bool	notifyActionPerformed = false;
char 	RDTM_ACTION[4] 	= "RDtm";
char 	RDPT_ACTION[4] 	= "RDpt";
char	MVUP_ACTION[4] 	= "MVup";
char	MVDW_ACTION[4] 	= "MVdw";
char	MVRG_ACTION[4] 	= "MVrg";
char	MVLF_ACTION[4] 	= "MVlf";
bool	FLAG_RX						= false;
bool	FLAG_TX						= false;

volatile uint16_t RX_Value[2];
volatile uint16_t TX_Value[2];

/*	v7.0 Modification 	16.12.2016	*/
int		dataCounter = 0;
int 	result;
bool	TM_REQUEST = false;
bool	PT_REQUEST = false;

/* Hard to read configurations****/
#ifdef __DBG_ITM
volatile int ITM_RxBuffer;              /*  CMSIS Debug Input                 */
#endif

/*----------------------------------------------------------------------------
 Define  USART
 *----------------------------------------------------------------------------*/
#define BTx  								USART3
#define TERMINALx  					USART2
#define BT_IRQHandler				USART3_IRQHandler
#define TERMINAL_IRQHandler	USART2_IRQHandler
#define BT_IT_RXNE  ((uint8_t)0x0525)
#define BT_IT_Mask  ((uint8_t)0x001F)  /*!< USART Interrupt Mask */
#define BT_IT_CTS   ((uint16_t)0x096A)

#define BT_Address    ((u32)0x4000484C)

/*----------------------------------------------------------------------------
 Define  Baudrate setting (BRR) for USART
 *----------------------------------------------------------------------------*/
#define __DIV(__PCLK, __BAUD)       ((__PCLK*25)/(4*__BAUD))
#define __DIVMANT(__PCLK, __BAUD)   (__DIV(__PCLK, __BAUD)/100)
#define __DIVFRAQ(__PCLK, __BAUD)   (((__DIV(__PCLK, __BAUD) - (__DIVMANT(__PCLK, __BAUD) * 100)) * 16 + 50) / 100)
#define __USART_BRR(__PCLK, __BAUD) ((__DIVMANT(__PCLK, __BAUD) << 4)|(__DIVFRAQ(__PCLK, __BAUD) & 0x0F))

#define POT_ADDR 	 ((u32)0x4001244C)
#define LM75B_ADDR  0x90
#define LM75B_TEMP  0x00 //Temperature register (RO)
#define LM75B_CONF  0x00 //Configuration register (R/W)
#define LM75B_THYST 0x00 //Overtemperature shutdown threshold register (R/W)
#define LM75B_TOS   0x00 //Hysteresis register (R/W)

/*----------------------------------------------------------------------------
  Initialize UART pins, Baudrate
 *----------------------------------------------------------------------------*/
void SER_Initialize (void) {

#ifdef __DBG_ITM
  ITM_RxBuffer = ITM_RXBUFFER_EMPTY;       /*  CMSIS Debug Input              */
#else
  RCC->APB2ENR |=  (   1ul <<  2);         /* Enable GPIOA clock              */
  RCC->APB1ENR |=  (   1ul << 17);         /* Enable USART#2 clock            */

  /* Configure PA3 to USART2_RX, PA2 to USART2_TX */
  RCC->APB2ENR |=  (   1ul <<  0);         /* enable clock Alternate Function */
  AFIO->MAPR   &= ~(   1ul <<  3);         /* clear USART2 remap              */

  GPIOA->CRL   &= ~(0xFFul <<  8);         /* clear PA2, PA3                  */
  GPIOA->CRL   |=  (0x0Bul <<  8);         /* USART2 Tx (PA2) output push-pull*/
  GPIOA->CRL   |=  (0x04ul << 12);         /* USART2 Rx (PA3) input floating  */

  BTx->BRR  = __USART_BRR(24000000ul, 115200ul);  /* 115200 baud @ 24MHz   */
  BTx->CR3   = 0x0000;                  /* no flow control                 */
  BTx->CR2   = 0x0000;                  /* 1 stop bit                      */
  BTx->CR1   = ((   1ul <<  2) |        /* enable RX                       */
                (   1ul <<  3) |        /* enable TX                       */
                (   0ul << 12) |        /* 1 start bit, 8 data bits        */
                (   1ul << 13) );       /* enable USART                    */
#endif
}

/*******************************************************************************
* Function Name  : SER_PutChar
* Description    : Write character to Serial Port
* Input          : ASCI character (int)
* Output         : Returns ASCI character (int)
* Return         : None
*******************************************************************************/
int SER_PutChar (int ch) {
/*	Instrumentation Trace Macrocell Register (ITM). 
*
*		The ITM contains the following sub-blocks:
*		Timestamp			Generates timestamp packet.
*		Sync control	ITM synchronizer.
*		Arbiter				Arbitrates between synchronous, timestamp, and SWIT packet.
*		FIFO					First In First Out (FIFO).
*/
#ifdef __DBG_ITM
  ITM_SendChar (ch & 0xFF);
#else
  if (ch == '\n')  {
    while (!(USART2->SR & 0x0080));
    USART2->DR = 0x0D;
  }
   while (!(USART2->SR & 0x0080));
		USART2->DR = (ch & 0x1FF);
#endif

  return (ch);
}

/*******************************************************************************
* Function Name  : SER_GetChar
* Description    : Read character from Serial Port - blocking read
* Input          : None
* Output         : Returns ASCI character (int)
* Return         : None
*******************************************************************************/
int SER_GetChar (void) {

#ifdef __DBG_ITM
  if (ITM_CheckChar())
    return ITM_ReceiveChar();
#else
   while (!(USART2->SR & 0x0020));
   return (USART2->DR);
#endif
}

/*******************************************************************************
* Function Name  : BT_PutChar
* Description    : Write character to Serial Port
* Input          : ASCI character (int)
* Output         : Returns ASCI character (int)
* Return         : None
*******************************************************************************/
int BT_PutChar (int ch) {
/*	Instrumentation Trace Macrocell Register (ITM). 
*
*		The ITM contains the following sub-blocks:
*		Timestamp			Generates timestamp packet.
*		Sync control	ITM synchronizer.
*		Arbiter				Arbitrates between synchronous, timestamp, and SWIT packet.
*		FIFO					First In First Out (FIFO).
*/

#ifdef __DBG_ITM
  ITM_SendChar (ch & 0xFF);
#else
  if (ch == '\n')  {
    while (!(USART3->SR & 0x0080));
    USART3->DR = 0x0D;
  }
   while (!(USART3->SR & 0x0080));
		USART3->DR = (ch & 0x1FF);
#endif

  return (ch);

}

/*******************************************************************************
* Function Name  : BT_configuration
* Description    : Configures the BT Bus
* Input          : USART TypeDef
* Output         : None
* Return         : None
*******************************************************************************/
void BT_configuration(void)
{
	GPIO_InitTypeDef  	GPIO_InitStructure; 
	USART_InitTypeDef 	USART_InitStructure;
	u16 								BTx_PIN_TX, BTx_PIN_RX, Terminalx_PIN_TX, Terminalx_PIN_RX;
	
	/* Enable Clocks for BT */
	RCC_APB1PeriphClockCmd	 (RCC_APB1Periph_USART2, ENABLE);
	RCC_APB1PeriphClockCmd	 (RCC_APB1Periph_USART3, ENABLE);
	
	/* Configure BT pins */
	BTx_PIN_TX 				= BT_GPIO_PARTIALREMAP_PIN_TX;
	BTx_PIN_RX				= BT_GPIO_PARTIALREMAP_PIN_RX;
	
	/* Configure Terminal pins */
	Terminalx_PIN_TX 	= TERMINAL_GPIO_PIN_TX;
	Terminalx_PIN_RX	= TERMINAL_GPIO_PIN_RX;
	
	/* Configure Communication reset */
	Communication_Pin_Reset = TERMINAL_GPIO_PIN_NRST;
	
	/* USART2 NRST Configure PA10 as output Push Pull*/
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_3;	
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	/*	Reset BT Module till Button Pressed */
	GPIO_ResetBits(GPIOC, GPIO_Pin_3);
	
	/* USART2 TX Configure PA02 as alternate-function output*/
	GPIO_InitStructure.GPIO_Pin 	= Terminalx_PIN_TX;				
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/* USART2 RX Configure PA03 as input floating*/
  GPIO_InitStructure.GPIO_Pin 	= Terminalx_PIN_RX;
  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*Remap pins for USART3 */
	GPIO_PinRemapConfig(AFIO_MAPR_USART3_REMAP_PARTIALREMAP, ENABLE);
	GPIO_InitStructure.GPIO_Pin 	= BTx_PIN_TX;				
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin 	= BTx_PIN_RX;
  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
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

	/* Configure USART2 RX/TX */ 
	USART_InitStructure.USART_BaudRate 						= 115200; // 
	USART_InitStructure.USART_WordLength 					= USART_WordLength_8b;
	USART_InitStructure.USART_StopBits 						= USART_StopBits_1;
	USART_InitStructure.USART_Parity 							= USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode 								= USART_Mode_Tx | USART_Mode_Rx;
	/* Initialize USART2*/
	USART_Init(USART2, &USART_InitStructure);
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART2, ENABLE);
	
	/* Configure USART3 RX/TX */ 
	USART_InitStructure.USART_BaudRate 						= 115200; // 
	USART_InitStructure.USART_WordLength 					= USART_WordLength_8b;
	USART_InitStructure.USART_StopBits 						= USART_StopBits_1;
	USART_InitStructure.USART_Parity 							= USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode 								= USART_Mode_Tx | USART_Mode_Rx;
	/* Initialize USART3*/
	USART_Init(USART3, &USART_InitStructure);
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART3, ENABLE);

}

/*******************************************************************************
* Function Name  : BT_interrupt_config
* Description    : Configuration for BT interruptions
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Communication_interrupt_config(void){
	
	  NVIC_InitTypeDef NVIC_InitStructure;  
#ifdef  VECT_TAB_RAM  
  /* Set the Vector Table base location at 0x20000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
#else  /* VECT_TAB_FLASH  */
  /* Set the Vector Table base location at 0x08000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
#endif

  /* Configure the NVIC Preemption Priority Bits */  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);			// 0 bits for pre-emption priority
																											// 4 bits for subpriority
	
  /* Enable the USART2 Receive Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel 										= USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority 				= 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd 								= ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	/* Enable the USART1 Receive Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel 										= USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority 				= 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd 								= ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}


/*******************************************************************************
* Function Name  : Terminal_IRQHandler
* Description    : Read/Write Bytes from address via Terminal
* Input          : None
* Output         : Turn on RED    LED when char "u" or "U" is received
* Output         : Turn on BLUE   LED when char "d" or "D" is received
* Output         : Turn on YELLOW LED when char "r" or "R" is received
* Output         : Turn on GREEN  LED when char "l" or "L" is received
* Return         : None
*******************************************************************************/
void TERMINAL_IRQHandler(void){

	/* Information Received */
	if ((USART_GetITStatus(TERMINALx, USART_IT_RXNE) != RESET)
			||(USART_GetFlagStatus(TERMINALx, USART_FLAG_RXNE)))
	{
			FLAG_RX = true;
			TM_Rx_Data = USART_ReceiveData(TERMINALx);
		
			MOVE_RIGHT = false;
 			MOVE_LEFT  = false;
 			MOVE_UP 	 = false;
 			MOVE_DOWN  = false;
 			WRONG_PWD  = false;
		
		if (!RSTBT)
			if (!TM_CONNECT)
				{
					switch (TM_Rx_Data)
					{
						case 'H': {authentication[0]=true; break;}
						case 'E': {if(authentication[0]==true)authentication[1]=true; else Authentication_Reset(); break;}
						case 'C': {if(authentication[1]==true)authentication[2]=true; else Authentication_Reset(); break;}
						case 'T': {if(authentication[2]==true)authentication[3]=true; else Authentication_Reset(); break;}
						case 'O': {if(authentication[3]==true)authentication[4]=true; else Authentication_Reset(); break;}
						case 'R': {if(authentication[4]==true)authentication[5]=true; else Authentication_Reset(); break;}
						default : {Authentication_Reset(); break;}
					}
					if (authentication[5]==true)
					{
						connect = true;
						printf("Connected");
						
						if (!BT_CONNECT)
							GPIOA->ODR ^= (1 << 9);
						
						TM_CONNECT = true;
						FLAG_RX = false;
					}
					else if (connectionTrialCount>5)
						disconnect();
				}
		else 
			{
					/*
					*		Case '117' ASCII (u): Activate Motor Move up 	 	(RED LED)
					*		Case '085' ASCII (U): Activate Motor Move up 	 	(RED LED)
					*		Case '100' ASCII (d): Activate motor Move down 	(BLUE LED)
					*		Case '068' ASCII (D): Activate Motor Move up 	 	(BLUE LED)
					*		Case '114' ASCII (r): Activate Motor Move right (YELLOW LED)
					*		Case '082' ASCII (R): Activate Motor Move right (YELLOW LED)
					*		Case '108' ASCII (l): Activate Motor Move left  (GREEN LED)
					*		Case '076' ASCII (l): Activate Motor Move left  (GREEN LED)
					*/
				
				switch (TM_Rx_Data)
				{
					case 117: {MOVE_UP 		= true; GPIOB->ODR ^= (1 << 4); printf("U");break;}
					case  85: {MOVE_UP 		= true; GPIOB->ODR ^= (1 << 4); printf("U");break;}
					case 'd': {MOVE_DOWN 	= true; GPIOC->ODR ^= (1 << 7); GPIOA->ODR ^= (1 << 9); printf("D");break;}
					case 'D': {MOVE_DOWN 	= true; GPIOC->ODR ^= (1 << 7); GPIOA->ODR ^= (1 << 9); printf("D");break;}
					case 'r': {MOVE_RIGHT = true; GPIOC->ODR ^= (1 << 7); GPIOB->ODR ^= (1 << 4); GPIOA->ODR ^= (1 << 9); printf("R");break;}
					case 'R': {MOVE_RIGHT = true; GPIOC->ODR ^= (1 << 7); GPIOB->ODR ^= (1 << 4); GPIOA->ODR ^= (1 << 9); printf("R");break;}
					case 'l': {MOVE_LEFT 	= true; GPIOA->ODR ^= (1 << 9); printf("L");break;}
					case 'L': {MOVE_LEFT 	= true; GPIOA->ODR ^= (1 << 9); printf("L");break;}
					default : {TERMINALx->DR = 0; break;}
				}
				TM_Rx_Data = 0;
				FLAG_RX = false;
			}
	}
}

/*******************************************************************************
* Function Name  : BT_IRQHandler
* Description    : Read/Write Bytes from address via BT
* Input          : None
* Output         : Turn on RED    LED when char "u" or "U" is received
* Output         : Turn on BLUE   LED when char "d" or "D" is received
* Output         : Turn on YELLOW LED when char "r" or "R" is received
* Output         : Turn on GREEN  LED when char "l" or "L" is received
* Return         : None
*******************************************************************************/
void BT_IRQHandler(void){

	/* Information Received */
	if (((USART_GetITStatus(BTx, USART_IT_RXNE) != RESET)
			||(USART_GetFlagStatus(BTx, USART_FLAG_RXNE)))&&!FLAG_TX)
	{
			FLAG_RX = true;
			BT_Rx_Data = USART_ReceiveData(BTx);
		
			MOVE_RIGHT = false;
 			MOVE_LEFT  = false;
 			MOVE_UP 	 = false;
 			MOVE_DOWN  = false;
 			WRONG_PWD  = false;
		
		if (!RSTBT)
			if (!BT_CONNECT)
				{
					switch (BT_Rx_Data)
					{
						case 'H': {authentication[0]=true; break;}
						case 'E': {if(authentication[0]==true)authentication[1]=true; else Authentication_Reset(); break;}
						case 'C': {if(authentication[1]==true)authentication[2]=true; else Authentication_Reset(); break;}
						case 'T': {if(authentication[2]==true)authentication[3]=true; else Authentication_Reset(); break;}
						case 'O': {if(authentication[3]==true)authentication[4]=true; else Authentication_Reset(); break;}
						case 'R': {if(authentication[4]==true)authentication[5]=true; else Authentication_Reset(); break;}
						default : {Authentication_Reset(); break;}
					}
					if (authentication[5]==true)
					{
						connect = true;
						printf("Connected");
						strcpy (notifyString,"BT Connected "); 
						notify_Observer();
						if (!TM_CONNECT)
							GPIOA->ODR ^= (1 << 9);
						
						BT_CONNECT = true;
						FLAG_RX = false;
						BTx -> DR = 0;
					}
					else if (connectionTrialCount>5)
						disconnect();
				}
		else 
			{
				dataReceived[countDataReceived] = BT_Rx_Data;	
				countDataReceived++;
					
				if (countDataReceived>=4)
				{
									
					/*	COMPARE DATA RECEIVED
					*
					*		STRING COMPARE MVUP_ACTION: Activate Motor Move up 	 	(YELLOW LED)
					*		STRING COMPARE MVDW_ACTION: Activate motor Move down 	(BLUE LED)
					*		STRING COMPARE MVRG_ACTION: Activate Motor Move right (PURPLE LED)
					*		STRING COMPARE MVLF_ACTION: Activate Motor Move left  (CYAN LED)
					*		STRING COMPARE SECURE_ACTION: End communication
					*/
					countDataReceived = 0;
					result = strcmp(dataReceived,MVUP_ACTION);
					if (result == 0xFFFFFFB3)
					{
						GPIOB->ODR ^= (1 << 4); 
						serviceFound = true; 
						MOVE_UP = true;
						printf("\n");
						printf("%s Implemented",dataReceived);
						strcpy (notifyString,"Move up implemented "); 
						notify_Observer();
					}
					result =strcmp (dataReceived, MVDW_ACTION);
					if (result == 0xFFFFFFB3)
					{
						GPIOC->ODR ^= (1 << 7); 
						GPIOA->ODR ^= (1 << 9);
						serviceFound = true; 
						MOVE_DOWN = true;
						printf("\n");
						printf("%s Implemented",dataReceived);
						strcpy (notifyString,"Move down implemented "); 
						notify_Observer();
					}
					result = strcmp (dataReceived, MVRG_ACTION);
					if ( result == 0xFFFFFFB3)
					{
						GPIOB->ODR ^= (1 << 4); 
						GPIOA->ODR ^= (1 << 9);
						GPIOC->ODR ^= (1 << 7); 
						serviceFound = true; 
						MOVE_RIGHT = true;
						printf("\n");
						printf("%s Implemented",dataReceived);
						strcpy (notifyString,"Move right implemented "); 
						notify_Observer();
					}
					result = strcmp (dataReceived, MVLF_ACTION);
					if (result == 0xFFFFFFFF)
					{
						GPIOA->ODR ^= (1 << 9); 
						serviceFound = true;
						MOVE_LEFT = true;
						printf("\n");
						printf("%s Implemented",dataReceived);
						strcpy (notifyString,"Move left implemented "); 
						notify_Observer();
					}
					result = strcmp (dataReceived, RDTM_ACTION);
					if (result == 0xFFFFFFFF)
					{
						serviceFound = true;
						TM_REQUEST = true;
						printf("\n");
						printf("%s Implemented",dataReceived);
						strcpy (notifyString,"Read temp implemented "); 
						notify_Observer();
					}
					result = strcmp (dataReceived, RDPT_ACTION);
					if (result == 0xFFFFFFFF)
					{
						serviceFound = true;
						PT_REQUEST = true;
						printf("\n");
						printf("%s Implemented",dataReceived);
						strcpy (notifyString,"Read pot implemented "); 
						notify_Observer();
					}
					if (!serviceFound)
					{
						printf("\n");
						printf("%s Not found",dataReceived);
					}
					serviceFound = false;
					dataReceived[0] = 0; dataReceived[1] = 0; 
					dataReceived[2] = 0; dataReceived[3] = 0; 
					FLAG_RX = false;
				}
				BT_Rx_Data = 0;
			}
			
	}
	if (((USART_GetITStatus(BTx, USART_IT_TXE) != RESET)
			||(USART_GetFlagStatus(BTx, USART_IT_TXE) != RESET)) && !FLAG_RX)
	{
			FLAG_TX = true;
			
			USART_SendData(BTx,'t');
			while(USART_GetFlagStatus(BTx, USART_FLAG_TC) == RESET);
			USART_ClearFlag	(BTx,USART_FLAG_TXE);
			USART_ClearFlag	(BTx,USART_FLAG_TC);
			USART_ITConfig	(BTx, USART_IT_TXE, DISABLE);
		
			USART_SendData(BTx,Temp_read());
			while(USART_GetFlagStatus(BTx, USART_FLAG_TC) == RESET);
			USART_ClearFlag	(BTx,USART_FLAG_TXE);
			USART_ClearFlag	(BTx,USART_FLAG_TC);
			USART_ITConfig	(BTx, USART_IT_TXE, DISABLE);
			FLAG_TX = false;
		
			USART_SendData(BTx,'p');
			while(USART_GetFlagStatus(BTx, USART_FLAG_TC) == RESET);
			USART_ClearFlag	(BTx,USART_FLAG_TXE);
			USART_ClearFlag	(BTx,USART_FLAG_TC);
			USART_ITConfig	(BTx, USART_IT_TXE, DISABLE);
		
			USART_SendData(BTx,'p');
			while(USART_GetFlagStatus(BTx, USART_FLAG_TC) == RESET);
			USART_ClearFlag	(BTx,USART_FLAG_TXE);
			USART_ClearFlag	(BTx,USART_FLAG_TC);
			USART_ITConfig	(BTx, USART_IT_TXE, DISABLE);
	}
	
}

/*******************************************************************************
* Function Name  : Authetication_Reset
* Description    : Validates the host connected to the system
* Input          : None
* Output         : Bool True if host is recognized
* Output         : Bool False if host is  not recognized
* Return         : Bool
*******************************************************************************/
void Authentication_Reset (void){
	
	authentication[0] = false;
	authentication[1] = false;
	authentication[2] = false;
	authentication[3] = false;
	authentication[4] = false;
	authentication[5] = false;
	GPIOB->ODR ^= (1 << 4); 
	GPIOC->ODR ^= (1 << 7);
	GPIOA->ODR ^= (1 << 9);
	TM_CONNECT = false;
	BT_CONNECT = false;
	WRONG_PWD = true;
	connectionTrialCount++;
	//printf("%s","Wrong");
	FLAG_RX = false;
	FLAG_TX = false;
}

/*******************************************************************************
* Function Name  : Authetication_Reset
* Description    : Validates the host connected to the system
* Input          : None
* Output         : Bool True if host is recognized
* Output         : Bool False if host is  not recognized
* Return         : Bool
*******************************************************************************/
bool 		TM_Autentified (void){
		return TM_CONNECT;
}

/*	v4.0 Modification 	15.12.2016	*/
/*******************************************************************************
* Function Name  : is_Receiving_BT
* Description    : Checks flag receiving data to wait till it's completed
* Input          : None
* Output         : FLAG_RX - flag receiving BT
* Return         : bool
*******************************************************************************/
bool 		is_Receiving_BT (void){
	return	FLAG_RX;
}

/*	v4.0 Modification 	15.12.2016	*/
/*******************************************************************************
* Function Name  : startTransmission_BT
* Description    : Enables transmission and thus not execute any other function
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void 		startTransmission_BT (void){
	FLAG_TX = true;
}

/*	v3.0 Modification 	12.12.2016	*/
/*******************************************************************************
* Function Name  : Disconnect
* Description    : Finishes the Serial communication (BT and Terminal)
* Input          : None
* Output         : None
* Return         : void
*******************************************************************************/
void 		disconnect (void){
	TM_CONNECT = false;
	BT_CONNECT = false;
	connect = false;
	/*	Include reset button for BT */
	GPIO_ResetBits(GPIOC, GPIO_Pin_3);
	RSTBT = true;
	FLAG_RX = true;
	FLAG_TX = true;
}

/*	v3.0 Modification 	12.12.2016	*/
/*******************************************************************************
* Function Name  : is_Reset_BT
* Description    : Checks flag restart BT Module Finishes
* Input          : None
* Output         : RSTBT - flag reset BT
* Return         : bool
*******************************************************************************/
bool 		is_Reset_BT (void){
	return	RSTBT;
}

/*	v3.0 Modification 	12.12.2016	*/
/*******************************************************************************
* Function Name  : activate_BT
* Description    : Activates BT Module
* Input          : None
* Output         : None
* Return         : void
*******************************************************************************/
void 		activate_BT (void){
	connectionTrialCount = 0;
	RSTBT = false;
	GPIO_SetBits(GPIOC, GPIO_Pin_3);
}

/*******************************************************************************
* Function Name  : Authetication_Reset
* Description    : Validates the host connected to the system
* Input          : None
* Output         : Bool True if host is recognized
* Output         : Bool False if host is  not recognized
* Return         : Bool
*******************************************************************************/
bool 		BT_Autentified (void){
		return BT_CONNECT;
}

/*******************************************************************************
* Function Name  : GetFlagStatus
* Description    : Checks if flag has been activated
* Input          : char Flag
*									 @arg: 'u' MOVE_UP
*									 @arg: 'd' MOVE_DOWN
*									 @arg: 'r' MOVE_RIGHT
*									 @arg: 'l' MOVE_LEFT
*									 @arg: 'w' WRONG_PWD
* Output         : Bool True if flag is set
* Output         : Bool False if flat is not set
* Return         : Bool
*******************************************************************************/
bool 	BT_GetFlagStatus (char Flag){
	
	switch (Flag)
	{
		case 'u': return MOVE_UP;
		case 'd': return MOVE_DOWN;
		case 'r': return MOVE_RIGHT;
		case 'l': return MOVE_LEFT;
		case 'w': return WRONG_PWD;
		case 'p': return PT_REQUEST;
		case 't': return TM_REQUEST;
		default : return false;
	}
}


/*******************************************************************************
* Function Name  : Clear_Flag
* Description    : Disables flag to read again anothe value
* Input          : char Flag
*									 @arg: 'u' MOVE_UP
*									 @arg: 'd' MOVE_DOWN
*									 @arg: 'r' MOVE_RIGHT
*									 @arg: 'l' MOVE_LEFT
*									 @arg: 'w' WRONG_PWD
* Output         : None
* Return         : Void
*******************************************************************************/
void 	BT_Clear_Flag (char Flag){
	
	switch (Flag)
	{
		case 'u': {MOVE_UP 		= false; GPIOB->ODR ^= (1 << 4); break;}
		case 'd': {MOVE_DOWN 	= false; GPIOC->ODR ^= (1 << 7); GPIOA->ODR ^= (1 << 9); break;}
		case 'r': {MOVE_RIGHT = false; GPIOC->ODR ^= (1 << 7); GPIOB->ODR ^= (1 << 4); GPIOA->ODR ^= (1 << 9); break;}
		case 'l': {MOVE_LEFT 	= false; GPIOA->ODR ^= (1 << 9); break;}
		case 'w': WRONG_PWD 	= false; break;
		case 't': TM_REQUEST 	= false; break;
		case 'p': PT_REQUEST 	= false; break;
		default	: break;
	}
}

/*******************************************************************************
* Function Name  : set_Data
* Description    : Prepares the String to be send Using BT
* Input          : uint16_t data
* Output         : None
* Return         : Void
*******************************************************************************/
void 	set_Data (uint16_t *data){
	for (int i=0; i<=sizeof(data); i++)
			notifyString[i] = data[i];
}

/*******************************************************************************
* Function Name  : Temp_read
* Description    : Reads temperature from sensor LM75B
* Input          : None
* Output         : Temperature value
* Return         : int16_t 
*******************************************************************************/
int16_t Temp_read(void)
{
	int16_t 	tmp;
	//$TASK LM75B              
	tmp = I2C_Write_1_Read_2_byte(I2C1, LM75B_ADDR, LM75B_TEMP) >> 5;
	return tmp;
}

/*******************************************************************************
* Function Name  : notify_Observer
* Description    : Sends a acknowledge to the terminal after an acion is executed
* Input          : char
* Output         : none
* Return         : void 
*******************************************************************************/
void notify_Observer(){
	int time;
	for (int i = 0; i < sizeof(notifyString); i++){
		time = 2000;
		if (notifyString[i]!=0)
		USART_SendData(TERMINALx,notifyString[i]);
		while(time != 0) time--;
		notifyString[i]=0;
	}
}

