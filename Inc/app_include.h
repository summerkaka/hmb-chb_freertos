/**
  ******************************************************************************
  * @file    Project/src/app_include.h
  * @author
  * @version V0.00
  * @date
  * @brief   app.h
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_INCLUDE_H
#define __APP_INCLUDE_H

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
#include "FreeRTOSConfig.h"

#include "AD5241.h"
#include "Adaptor.h"
#include "adc.h"
#include "adc_app.h"
#include "Battery.h"
#include "can.h"
#include "can_app.h"
#include "command.h"
#include "dma.h"
#include "eeprom.h"
#include "FieldCase.h"
#include "gpio.h"
#include "Heater.h"
#include "i2c.h"
#include "iwdg.h"
#include "Load.h"
#include "LTC2943.h"
#include "sdadc.h"
#include "sdadc_app.h"
#include "stm32f3xx_it.h"
#include "tim.h"
#include "usart.h"



/* Exported macro ------------------------------------------------------------*/
#define APP_ADDRESS 0x08008000
#define BL_ADDRESS  0x08000000
#define ON          1
#define OFF         0

//#define offsetof(type, member) ((size_t) &((type *)0)->member)
#define container_of(ptr, type, member) ({                      \
        const typeof( ((type *)0)->member ) *__mptr = (ptr);    \
        (type *)( (char *)__mptr - offsetof(type,member) );})

// #define vprintf(mutex, arg...)                                                  \
// do {                                                                            \
//     osMutexWait(mutex, osWaitForever);                                          \
//     printf(##arg);                                                              \
//     osMutexRelease(mutex);                                                      \
// } while (0)

#define xprintf(arg...)                                                         \
do {                                                                            \
    osMutexWait(mutex_printfHandle, osWaitForever);                             \
    printf(##arg);                                                              \
    osMutexRelease(mutex_printfHandle);                                         \
} while (0)

#define pr_debug(fmt,arg...) \
printf(KERN_DEBUG fmt, ##arg)

#define debug(...)                                                              \
do {                                                                            \
    osThreadSuspendAll();                                                       \
    printf(__VA_ARGS__);                                                        \
    osThreadResumeAll();                                                        \
} while (0)

#define fdebug(format, ...) fprintf(stderr, format, ##__VA_ARGS__)

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
extern const float fw_version;
extern uint32_t run_cycle;

extern osMutexId mutex_iic0Handle;
extern osMutexId mutex_iic1Handle;
extern osMutexId mutex_printfHandle;
extern osSemaphoreId sem_consoleHandle;
extern osSemaphoreId csemHandle;
extern QueueHandle_t q_canmsg;
extern TimerHandle_t timer_pump1_start;
extern TimerHandle_t timer_pump1_stop;
extern TimerHandle_t timer_pump2_start;
extern TimerHandle_t timer_pump2_stop;
extern TimerHandle_t timer_pvalve_start;
extern TimerHandle_t timer_pvalve_stop;


/* Exported variables ------------------------------------------------------- */
__no_init uint32_t update_request @0x20000000;
extern const float fw_version;


/* Exported functions ------------------------------------------------------- */
uint32_t GetSecond(void);
uint32_t GetMinute(void);




#endif /*__APP_H */
