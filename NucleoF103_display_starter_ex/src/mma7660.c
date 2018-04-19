/**
  ******************************************************************************
  * @file    MMA7660.c 
  * @author  rnm1
  * @version V1.0
  * @date    23.02.2016
  * @brief   MMA7660 3-Axis Orientation/Motion Detection Sensor library functions
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
#include "mma7660.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//$TASK MMA7669
#define MMA7660_ADDR 0x00
#define MMA7660_SENSITIVITY 0

#define MMA7660_X 0x00
#define MMA7660_Y 0x00
#define MMA7660_Z 0x00
#define MMA7660_TILT 0x00
#define MMA7660_SRST 0x00
#define MMA7660_SPCNT 0x00
#define MMA7660_INTSU 0x00
#define MMA7660_MODE 0x00
#define MMA7660_STAND_BY 0x00
#define MMA7660_ACTIVE 0x00
#define MMA7660_SR 0x00 //sample rate register
#define AUTO_SLEEP_120 0x00//120 sample per second
#define AUTO_SLEEP_64 0x00
#define AUTO_SLEEP_32 0x00
#define AUTO_SLEEP_16 0x00
#define AUTO_SLEEP_8 0x00
#define AUTO_SLEEP_4 0x00
#define AUTO_SLEEP_2 0x00
#define AUTO_SLEEP_1 0x00
#define MMA7660_PDET 0x00
#define MMA7660_PD 0x00
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initialize MMA7660
  * @param  None
  * @retval None
  */
void MMA7660init(void)
{
	MMA7660setMode(MMA7660_STAND_BY);
	MMA7660setSampleRate(AUTO_SLEEP_64);
	MMA7660setMode(MMA7660_ACTIVE);
}

/**
  * @brief  set mode of MMA7660
	* @param  uint8_t mode: MMA7660_STAND_BY or MMA7660_ACTIVE
  * @retval None
  */
void MMA7660setMode(uint8_t mode)
{
	//$TASK MMA7660
	/*set mode by writing MMA7660_MODE and mode over I2C*/

}

/**
  * @brief  set samplerate of MMA7660
	* @param  uint8_t rate: AUTO_SLEEP_1 to AUTO_SLEEP_120 (120 sample per second)
  * @retval None
  */
void MMA7660setSampleRate(uint8_t rate)
{
	//$TASK MMA7660
	/*set sample rate by writing MMA7660_SR and rate over I2C*/

}

/**
  * @brief  get x,y and z-acceleration of MMA7660
	* @param  float *x, float *y, float *z: give x,y and z variable as reference
  * @retval None
  */
void MMA7660getAcceleration(float *x, float *y, float *z)
{
	// u8 buffer[3] = {64, 64, 64};

	//$TASK MMA7660
	/* while alert bit is set: read all 3 data byte */

	/* calculate accelerations */

}
