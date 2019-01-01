/**
  ******************************************************************************
  * @file    Project/src/tim_app.h
  * @author
  * @version V0.00
  * @date
  * @brief   tim_app.h
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SYSTICK_H
#define __SYSTICK_H
//#ifdef __cplusplus
// extern "C" {
//#endif


/* Includes ------------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported variables ------------------------------------------------------- */


/* Exported functions ------------------------------------------------------- */
void delay_us(float us);
void delay_ms(uint16_t ms);
uint32_t GetMinute(void);
uint32_t GetSecond(void);
void Seconds_Handler(void);


//#ifdef __cplusplus
//}
//#endif
#endif /*__SYSTICK_H */


