/**
  ******************************************************************************
  * @file    Project/src/Load.h
  * @author
  * @version V0.00
  * @date
  * @brief   Load.h
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __Load_H
#define __Load_H


/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
#include "tim.h"


/* Exported macro ------------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/
typedef enum {
    kLoadOff = 0,
    kLoadOn,    // running
    kLoadError
} eLoadStatus;

typedef struct
{
    GPIO_TypeDef        *out_port;
    uint16_t const      out_pin;
    GPIO_TypeDef        *detect_port;
    uint16_t const      detect_pin;
    uint8_t             dev_id;
    uint16_t            err_code;
    eLoadStatus         status;
    int32_t             connect_time;
    int32_t             disconnect_time;
    uint32_t            turnoff_time;
} DigitalOut;

typedef struct
{
    uint8_t             dev_id;
    TIM_HandleTypeDef   *tim;
	const uint32_t      tim_channel;
    eLoadStatus         status;
    const uint32_t      period;
    float               duty;                                                   // 0 or 1 for dio type, 0~1 float value for pwm type
} PwmOut;

typedef struct
{
    //uint32_t origin;
    //uint32_t start_time;
    //uint32_t stop_time;
    TimerHandle_t timer_start;
    TimerHandle_t timer_stop;
    //int8_t sm;
    DigitalOut dio;
} PulseOut;

/* Exported constants --------------------------------------------------------*/


/* Exported variables ------------------------------------------------------- */
extern DigitalOut MB_Pwr;
extern DigitalOut MB_Fan;
extern DigitalOut CCB_Pwr;
extern DigitalOut Valve_1;
extern DigitalOut Valve_2;
extern DigitalOut Fan;
extern PulseOut Pump_1;
extern PulseOut Pump_2;
extern PulseOut PValve;


/* Exported functions ------------------------------------------------------- */
void DioSetTo(DigitalOut *dio, bool value);
void PwmSetTo(PwmOut *pwm, float duty);
void PwmSetDuty(PwmOut *pwm, float value);
void PulseOutSetStartTime(PulseOut *p, uint32_t time);
void PulseOutSetStopTime(PulseOut *p, uint32_t time);
void PulseOutStartRun(PulseOut *p);
void PulseOutStopRun(PulseOut *p);
void PulseOutRun(PulseOut *p);
void PulseOutOn(TimerHandle_t xtimer);
void PulseOutOff(TimerHandle_t xtimer);





#endif /*__Load_H */

