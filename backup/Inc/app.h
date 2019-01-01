/**
  ******************************************************************************
  * @file    Project/src/app.h
  * @author
  * @version V0.00
  * @date
  * @brief   app.h
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __app_H
#define __app_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "main.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "math.h"
#include "stdio.h"
#include "string.h"
#include "stdbool.h"
#include "stdint.h"

#include "AD5241.h"
#include "Adaptor.h"
#include "adc.h"
#include "adc_app.h"
#include "Battery.h"
#include "can.h"
//#include "can_app.h"
#include "dma.h"
#include "eeprom.h"
#include "FieldCase.h"
#include "gpio.h"
#include "i2c.h"
#include "iwdg.h"
#include "load.h"
#include "LTC2943.h"
#include "sdadc.h"
#include "sdadc_app.h"
#include "stm32f3xx_it.h"
#include "SysTick.h"
#include "tim.h"
#include "usart.h"


/* Exported macro ------------------------------------------------------------*/
#define DEBUG       0
#define APP_ADDRESS 0x08008000
#define BL_ADDRESS  0x08000000
//#define BEEP_EN

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
extern const float fw_version;
extern uint32_t run_cycle;

extern osThreadId tid_fieldcase;
extern osThreadId tid_adaptor;

extern SemaphoreHandle_t mutex_i2c0;
extern SemaphoreHandle_t mutex_i2c1;
extern SemaphoreHandle_t mutex_printf;
extern QueueHandle_t q_canmsg;


/* Exported variables ------------------------------------------------------- */
__no_init uint32_t update_request @0x20000000;


/* Exported functions ------------------------------------------------------- */


#endif /*__app_H */

