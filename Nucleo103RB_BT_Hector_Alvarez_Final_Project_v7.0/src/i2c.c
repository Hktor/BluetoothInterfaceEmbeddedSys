/******************************************************************************
* @file			I2C.c
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

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* Variables -----------------------------------------------------------------*/
enum I2CState I2C1_state = NONE, I2C2_state = NONE;
const enum I2CState TxError[4] = {B1NT, B2NT, B3NT, B4NT};
const enum I2CState RxError[4] = {B1NR, B2NR, B3NR, B4NR};

/*******************************************************************************
* Function Name  : I2C_Configuration
* Description    : Configures the I2C Bus
* Input          : I2C TypeDef
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_Configuration(I2C_TypeDef * I2Cx)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
	I2C_InitTypeDef  I2C_InitStructure;
	GPIO_TypeDef * I2Cx_GPIO;
	u16 I2Cx_PIN_SCL, I2Cx_PIN_SDA;
	
	/* Enable Clocks for I2C */
	if(I2Cx == I2C1)
	{
		RCC_APBPeriphClockCmd_I2C1(RCC_APB2Periph_I2C1_IO, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
#if defined(USE_I2C1_Remap)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
#endif	
	}
	else if(I2Cx == I2C2)
	{
		RCC_APBPeriphClockCmd_I2C2(RCC_APB2Periph_I2C2_IO, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	}
	
	/* Configure I2C pins and remap if necessary */
	if(I2Cx == I2C1)
	{
		I2Cx_GPIO = I2C1_GPIO;
#if defined(USE_I2C1_Remap)
		I2Cx_PIN_SCL = I2C1_REMAP_GPIO_PIN_SCL;
		I2Cx_PIN_SDA = I2C1_REMAP_GPIO_PIN_SDA;
		
		// Remap and Configure I2C1 pins
		GPIO_PinRemapConfig(Remap_I2C1, ENABLE);
#else
		I2Cx_PIN_SCL = I2C1_GPIO_PIN_SCL;
		I2Cx_PIN_SDA = I2C1_GPIO_PIN_SDA;
#endif		
	}
	else if(I2Cx == I2C2)
	{
		I2Cx_GPIO = I2C2_GPIO;
		I2Cx_PIN_SCL = I2C2_GPIO_PIN_SCL;
		I2Cx_PIN_SDA = I2C2_GPIO_PIN_SDA;
	}
	
	GPIO_InitStructure.GPIO_Pin = I2Cx_PIN_SCL | I2Cx_PIN_SDA;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(I2Cx_GPIO, &GPIO_InitStructure);
	
  /* I2C configuration */
	// I2C mode selection: I2C_Mode_I2C = default, I2C_Mode_SMBusDevice, I2C_Mode_SMBusHost 
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;  
	
	// Counter for I2C: I2C_DutyCycle_16_9, I2C_DutyCycle_2 = default 
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	
	// Own slave address: here 0x3C because of 7 Bit address 0x1E 
	I2C_InitStructure.I2C_OwnAddress1 = 0x3C;
	
	// Acknowledge Enable/Disbale: I2C_Ack_Disable, I2C_Ack_Enable = default 
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable; 
	
	// Length of address: I2C_AcknowledgedAddress_10bit, I2C_AcknowledgedAddress_7bit = default
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	 
	// Clock selection for SCL: 100kHz standard mode = default, 400kHz fast mode 
	I2C_InitStructure.I2C_ClockSpeed = I2C_CLOCK_FREQ;  
	
	/* I2C Peripheral Enable */
	I2C_Cmd(I2Cx, ENABLE);
	
	/* Apply I2C configuration after enabling it */
	I2C_Init(I2Cx, &I2C_InitStructure);
}

/*******************************************************************************
* Function Name  : I2C_Read_1_byte
* Description    : Read 1 Byte from address via the I2C Bus
* Input          : I2C TypeDef, I2C address
* Output         : None
* Return         : 1 Byte of data received over I2C
*******************************************************************************/
u8 I2C_Read_1_byte(I2C_TypeDef * I2Cx, u8 address)
{
	u8 output = 0;
	
	// Read 1 byte
	I2C_Read_n_byte(I2Cx, address, 1, &output);
	
	return output;
}

/*******************************************************************************
* Function Name  : I2C_Read_2_byte
* Description    : Read 2 Bytes from address via the I2C Bus
* Input          : I2C TypeDef, I2C address
* Output         : None
* Return         : 2 Bytes of data received over I2C
*******************************************************************************/
u16 I2C_Read_2_byte(I2C_TypeDef * I2Cx, u8 address)
{
	u16 output = 0;
	u8 i;
	u8 buffer[2] = {0, 0};
	
	// Read 2 bytes
	I2C_Read_n_byte(I2Cx, address, 2, &buffer[0]);

	// Shift received bytes
	for(i = 0; i < 2; i++) output |= buffer[i] << (1 - i) * 8;
	
	return output;
}

/*******************************************************************************
* Function Name  : I2C_Read_3_byte
* Description    : Read 3 Bytes from address via the I2C Bus
* Input          : I2C TypeDef, I2C address
* Output         : None
* Return         : 3 Bytes of data received over I2C
*******************************************************************************/
u32 I2C_Read_3_byte(I2C_TypeDef * I2Cx, u8 address)
{
	u32 output = 0;
	u8 i;
	u8 buffer[3] = {0, 0, 0};
	
	// Read 3 bytes
	I2C_Read_n_byte(I2Cx, address, 3, &buffer[0]);

	// Shift received bytes
	for(i = 0; i < 3; i++) output |= buffer[i] << (2 - i) * 8;
	
	return output;
}

/*******************************************************************************
* Function Name  : I2C_Read_n_byte
* Description    : Read n Bytes from address via the I2C Bus
* Input          : I2C TypeDef, I2C address, nummber of Bytes
* Input          : pointer to array of n Databytes
* Output         : n Bytes of data received over I2C
* Return         : None
*******************************************************************************/
void I2C_Read_n_byte(I2C_TypeDef * I2Cx, u8 address, u8 n, u8 * buffer)
{
	ErrorStatus state;
	u32 timeout;
	u8 ByteToRead = n, ByteRead = 0;
	
	// Send START condition for receiving
	I2C_GenerateSTART(I2Cx, ENABLE);

	// Test on I2Cx EV5 and clear it
	timeout = TIMEOUT;
	state = ERROR;
	while(!state && timeout--) state = I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT);
	if(state == ERROR)
	{
		// Test on I2Cx Busy Flag
		if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
		{
			// Send STOP Condition 
			I2C_GenerateSTOP(I2Cx, ENABLE);
		}
		
		// SCF: Start condition failure
		I2C_SetState(I2Cx, SCF);
		
		return;
	}

	// Send EEPROM address for read 
	I2C_Send7bitAddress(I2Cx, address, I2C_Direction_Receiver);

	// Test on I2Cx EV6 and clear it
	timeout = TIMEOUT;
	state = ERROR;
	while(!state && timeout--) state = I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
	if(state == ERROR)
	{
		// Test on I2Cx Busy Flag
		if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
		{
			// Send STOP Condition 
			I2C_GenerateSTOP(I2Cx, ENABLE);
		}
		
		// ADNM: Address not matched to any slave
		I2C_SetState(I2Cx, ADNM);
		return;
	}
	
	while(0 < ByteToRead)
	{
		switch(ByteToRead)
		{
			case 1:
				// Disable Acknowledgement 
				I2C_AcknowledgeConfig(I2Cx, DISABLE);

				// Send STOP Condition 
				I2C_GenerateSTOP(I2Cx, ENABLE);
			
				// Continue with regular receiving also for case 1
			default:
				// Test on EV7
				timeout = TIMEOUT;
				state = ERROR;
				while(!state && timeout--) state = I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED);
				if(state == ERROR)
				{
					// Test on I2Cx Busy Flag
					if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
					{
						// Send STOP Condition 
						I2C_GenerateSTOP(I2Cx, ENABLE);
					}
					
					// B*NR: Byte * not received (sets state to B1NR, B2NR, B3NR, B4NR or BxNR depending on ByteRead)
					I2C_SetState(I2Cx, ByteRead < 4 ? RxError[ByteRead] : BxNR);
					return;
				}

				// Read a byte from the EEPROM, decrease ByteToRead and increase ByteRead
				*(buffer + ByteRead++) = I2C_ReceiveData(I2Cx);
				ByteToRead--;
				break;
		}
	}

	// Enable Acknowledgement to be ready for another reception 
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	
	// BRF: Byte receive finished
	I2C_SetState(I2Cx, BRF);
}

/*******************************************************************************
* Function Name  : I2C_Write_1_byte
* Description    : Write 1 Byte to address via the I2C Bus
* Input          : I2C TypeDef, I2C address, 1 Databyte
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_Write_1_byte(I2C_TypeDef * I2Cx, u8 address, u8 data1)
{
	// Write 1 Databyte
	I2C_Write_n_byte(I2Cx, address, 1, &data1);
}

/*******************************************************************************
* Function Name  : I2C_Write_2_byte
* Description    : Write 2 Bytes to address via the I2C Bus
* Input          : I2C TypeDef, I2C address, 2 Databytes
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_Write_2_byte(I2C_TypeDef * I2Cx, u8 address, u8 data1, u8 data2)
{
	u8 data[2];
	data[0] = data1;
	data[1] = data2;

	// Write 2 Databytes
	I2C_Write_n_byte(I2Cx, address, 2, &data[0]);
}

/*******************************************************************************
* Function Name  : I2C_Write_3_byte
* Description    : Write 3 Bytes to address via the I2C Bus
* Input          : I2C TypeDef, I2C address, 3 Databytes
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_Write_3_byte(I2C_TypeDef * I2Cx, u8 address, u8 data1, u8 data2, u8 data3)
{
	u8 data[3] = {0, 0, 0};
	data[0] = data1;
	data[1] = data2;
	data[2] = data3;

	// Write 3 bytes
	I2C_Write_n_byte(I2Cx, address, 3, &data[0]);
}

/*******************************************************************************
* Function Name  : I2C_Write_n_byteNS
* Description    : Write n Bytes to address via the I2C Bus without generating stop
* Input          : I2C TypeDef, I2C address, pointer to array of n Databytes
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_Write_n_byteNS(I2C_TypeDef * I2Cx, u8 address, u8 n, u8 * data)
{
	ErrorStatus state;
	u32 timeout;
	u8 ByteToWrite = n, ByteWrite = 0;
	
	// Send START condition for receiving
	I2C_GenerateSTART(I2Cx, ENABLE);

	// Test on I2Cx EV5 and clear it
	timeout = TIMEOUT;
	state = ERROR;
	while(!state && timeout--) state = I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT);
	if(state == ERROR)
	{
		// Test on I2Cx Busy Flag
		if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
		{
			// Send STOP Condition 
			I2C_GenerateSTOP(I2Cx, ENABLE);
		}
		
		// SCF: Start condition failure
		I2C_SetState(I2Cx, SCF);
		return;
	}

	// Send EEPROM address for write 
	I2C_Send7bitAddress(I2Cx, address, I2C_Direction_Transmitter);

	// Test on I2Cx EV6 and clear it
	timeout = TIMEOUT;
	state = ERROR;
	while(!state && timeout--) state = I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	if(state == ERROR)
	{
		// Test on I2Cx Busy Flag
		if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
		{
			// Send STOP Condition 
			I2C_GenerateSTOP(I2Cx, ENABLE);
		}
		
		// ADNM: Address not matched to any slave
		I2C_SetState(I2Cx, ADNM);
		return;
	}
	
	while(0 < ByteToWrite)
	{
		// Write n Databytes
		I2C_SendData(I2Cx, *(data + ByteWrite++));
		
		// Decrease Number of byte to send
		ByteToWrite--;

		// Test on I2Cx EV8 and clear it
		timeout = TIMEOUT;
		state = ERROR;
		while(!state && timeout--) state = I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
		if(state == ERROR)
		{
			// Test on I2Cx Busy Flag
			if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
			{
				// Send STOP Condition 
				I2C_GenerateSTOP(I2Cx, ENABLE);
			}
					
			// B*NT: Byte * not transmitted (sets state to B1NT, B2NT, B3NT, B4NT or BxNT depending on ByteWrite)
			I2C_SetState(I2Cx, ByteWrite < 5 ? TxError[ByteWrite - 1] : BxNT);
			return;
		}
	}
	
	// BTF: Byte transfer finished
	I2C_SetState(I2Cx, BTF);
}

/*******************************************************************************
* Function Name  : I2C_Write_n_byte
* Description    : Write n Bytes to address via the I2C Bus
* Input          : I2C TypeDef, I2C address, nummber of Bytes
* Input          : pointer to array of n Databytes
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_Write_n_byte(I2C_TypeDef * I2Cx, u8 address, u8 n, u8 * data)
{
	I2C_Write_n_byteNS(I2Cx, address, n, data);
	
	// If BTF is set (Byte transfer finished)
	if(I2C_GetState(I2Cx) == BTF)
	{
		// Send STOP Condition 
		I2C_GenerateSTOP(I2Cx, ENABLE);
	}
}

/*******************************************************************************
* Function Name  : I2C_Write_1_Read_1_byte
* Description    : Write 1 Byte and Read 1 Byte to/from address via the I2C Bus
* Input          : I2C TypeDef, I2C address, 1 Databyte
* Output         : None
* Return         : 1 Byte of data received over I2C
*******************************************************************************/
u8 I2C_Write_1_Read_1_byte(I2C_TypeDef * I2Cx, u8 address, u8 data1)
{
	u8 output = 0;

	// Write 1 Databyte, don't generate stop
	I2C_Write_n_byteNS(I2Cx, address, 1, &data1);
	
	// If BTF is set (Byte transfer finished)
	if(I2C_GetState(I2Cx) == BTF)
	{
		// Read 1 Databyte
		I2C_Read_n_byte(I2Cx, address, 1, &output);
	}
	
	return output;
}

/*******************************************************************************
* Function Name  : I2C_Write_1_Read_2_byte
* Description    : Write 1 Byte and Read 2 Bytes to/from address via the I2C Bus
* Input          : I2C TypeDef, I2C address, 1 Databyte
* Output         : None
* Return         : 2 Bytes of data received over I2C
*******************************************************************************/
u16 I2C_Write_1_Read_2_byte(I2C_TypeDef * I2Cx, u8 address, u8 data1)
{
	u16 output = 0;
	u8 i;
	u8 buffer[2] = {0, 0};

	// Write 1 Databyte, don't generate stop
	I2C_Write_n_byteNS(I2Cx, address, 1, &data1);
	
	// If BTF is set (Byte transfer finished)
	if(I2C_GetState(I2Cx) == BTF)
	{
		// Read 2 Databytes
		I2C_Read_n_byte(I2Cx, address, 2, &buffer[0]);

		// Shift received Databytes
		for(i = 0; i < 2; i++) output |= buffer[i] << (1 - i) * 8;
	}
	
	return output;
}

/*******************************************************************************
* Function Name  : I2C_Write_1_Read_n_byte
* Description    : Write 1 Byte and Read n Bytes to/from address via the I2C Bus
* Input          : I2C TypeDef, I2C address, 1 Databyte, nummber of Bytes
* Input          : pointer to array of n Databytes
* Output         : n Bytes of data received over I2C
* Return         : None
*******************************************************************************/
void I2C_Write_1_Read_n_byte(I2C_TypeDef * I2Cx, u8 address, u8 data1, u8 n, u8 * buffer)
{
	// Write 1 Databyte, don't generate stop
	I2C_Write_n_byteNS(I2Cx, address, 1, &data1);
	
	// If BTF is set (Byte transfer finished)
	if(I2C_GetState(I2Cx) == BTF)
	{
		// Read n Databytes
		I2C_Read_n_byte(I2Cx, address, n, buffer);
	}
}

/*******************************************************************************
* Function Name  : I2C_SetState
* Description    : Set state of I2Cx and try to solve problem after errors
* Input          : I2C TypeDef, new state
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_SetState(I2C_TypeDef* I2Cx, enum I2CState state)
{
	static u8 countFault = 0;
	u32 timeout;
	GPIO_TypeDef * I2Cx_GPIO;
	u16 I2Cx_PIN_SDA, I2Cx_PIN_SCL;
	
	// Set new state
	if(I2Cx == I2C1)
	{
		I2C1_state = state;
	}
	else if(I2Cx == I2C2)
	{
		I2C2_state = state;
	}
	
	// count faults
	switch(state)
	{
		case NONE:
		case ADNM:
			countFault = 0;
			break;
		case SCF:
		case B1NT:
		case B2NT:
		case B3NT:
		case B4NT:
		case BxNT:
		case B1NR:
		case B2NR:
		case B3NR:
		case B4NR:
		case BxNR:
			countFault++;
			break;
		case BTF:
		case BRF:
			countFault = 0;
			break;
	}
	
	// If 5 or more faults occurred
	if(5 < countFault)
	{
		// Enable I2Cx Software Reset
		I2C_SoftwareResetCmd(I2Cx, ENABLE);
		
		// Define GPIO and Pins for I2Cx
		if(I2Cx == I2C1)
		{
			I2Cx_GPIO = I2C1_GPIO;
#if defined(USE_I2C1_Remap)
			I2Cx_PIN_SCL = I2C1_REMAP_GPIO_PIN_SCL;
			I2Cx_PIN_SDA = I2C1_REMAP_GPIO_PIN_SDA;
			// Configure SCL and SDA as open drain output 
			I2Cx_GPIO->CRH &= ~0x00000088;
			I2Cx_GPIO->CRH |= 0x00000077;
#else
			I2Cx_PIN_SCL = I2C1_GPIO_PIN_SCL;
			I2Cx_PIN_SDA = I2C1_GPIO_PIN_SDA;
			// Configure SCL and SDA as open drain output 
			I2Cx_GPIO->CRL &= ~0x88000000;
			I2Cx_GPIO->CRL |= 0x77000000;
#endif		
		}
		else if(I2Cx == I2C2)
		{
			I2Cx_GPIO = I2C2_GPIO;
			I2Cx_PIN_SCL = I2C2_GPIO_PIN_SCL;
			I2Cx_PIN_SDA = I2C2_GPIO_PIN_SDA;
			// Configure SCL and SDA as open drain output 
			I2Cx_GPIO->CRH &= ~0x00008800;
			I2Cx_GPIO->CRH |= 0x00007700;
		}
		
		// Release SDA
		I2Cx_GPIO->ODR |= I2Cx_PIN_SDA;
		
		// Toggle CLK until SDA and SCL released or timeout exceeded
		timeout = TIMEOUT;
		while((I2Cx_GPIO->IDR ^ (I2Cx_PIN_SDA | I2Cx_PIN_SCL)) && timeout--)
		{
			I2Cx_GPIO->ODR ^= I2Cx_PIN_SCL;
		}
		
		// Configure SCL and SDA as alternate function open drain
		if(I2Cx == I2C1)
		{
#if defined(USE_I2C1_Remap)
			I2Cx_GPIO->CRH |= 0x000000FF;
#else
			I2Cx_GPIO->CRL |= 0xFF000000;
#endif		
		}
		else if(I2Cx == I2C2)
		{
			I2Cx_GPIO->CRH |= 0x0000FF00;
		}
		
		// If SDA released
		if(0 < (I2Cx_GPIO->IDR & I2Cx_PIN_SDA))
		{
			// Close the I2C Bus
			I2C_DeInit(I2Cx);
			
			//Configure the I2C Bus
			I2C_Configuration(I2Cx);
		}
		
		// Reset countFault
		countFault = 0;
	}
}

/*******************************************************************************
* Function Name  : I2C_GetState
* Description    : Read state of I2Cx
* Input          : I2C TypeDef
* Output         : None
* Return         : State of I2Cx
*******************************************************************************/
enum I2CState I2C_GetState(I2C_TypeDef* I2Cx)
{
	enum I2CState state = NONE;
	
	// Read state of I2Cx
	if(I2Cx == I2C1)
	{
		state = I2C1_state;
	}
	else if(I2Cx == I2C2)
	{
		state = I2C2_state;
	}

	// Return state of I2Cx
	return state;
}
/******END OF FILE****/
