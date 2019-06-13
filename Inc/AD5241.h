/**
  ******************************************************************************
  * @file    Project/src/AD5241.h
  * @author
  * @version V0.00
  * @date
  * @brief   AD5241.h
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AD5241_H
#define __AD5241_H
//#ifdef __cplusplus
// extern "C" {
//#endif


/* Includes ------------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported variables ------------------------------------------------------- */

/* Exported functions ------------------------------------------------------- */
HAL_StatusTypeDef AD5241_Write(I2C_HandleTypeDef *hi2c, uint8_t data, osMutexId mutex);

//#ifdef __cplusplus
//}
//#endif
#endif /*__AD5241_H */