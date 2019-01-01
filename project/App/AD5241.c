/**
  ******************************************************************************
  * @file    Project/src/AD5241.c 
  * @author  
  * @version V0.00
  * @date  
  * @brief   AD5241.c
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "app.h"

/* Private macro -------------------------------------------------------------*/
#define AD5241_ID	    0x58
#define MIDSCALE_RESET 	0b01000000
#define SHUTDOWN		0b00100000
#define O1				0b00010000
#define	O2				0b00001000
/* Private typedef -----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Code begin ----------------------------------------------------------------*/
HAL_StatusTypeDef
AD5241_Write(I2C_HandleTypeDef *hi2c, uint8_t rheostat_data)
{
	uint8_t i2c_data[2] = {0, rheostat_data};
	return HAL_I2C_Master_Transmit(hi2c, AD5241_ID, i2c_data, 2, 5);
}

// HAL_StatusTypeDef
// AD5241_Read(I2C_HandleTypeDef *hi2c, uint8_t *pdata)
// {
// 	return HAL_I2C_Master_Receive(hi2c, AD5241_ID, pdata, 1);
// }