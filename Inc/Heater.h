/**
  ******************************************************************************
  * @file    Project/src/HeaterControl.h
  * @author
  * @version V0.00
  * @date
  * @brief   HeaterControl.h
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HEATERCONTROL_H
#define __HEATERCONTROL_H

/* Includes ------------------------------------------------------------------*/
#include "Load.h"
#include "sdadc_app.h"

/* Exported macro ------------------------------------------------------------*/
#define DEFAULT_KP      2000
#define DEFAULT_KI      4
#define DEFAULT_KD      0


/* Exported types ------------------------------------------------------------*/
typedef enum {
    kHeaterOff = 0,
    kHeaterPID = 1,
    kHeaterFixPwm
} eHeaterMode;

typedef struct {
    PwmOut pwm;
    float setpoint;
    float temperature;
    bool faulty;
    eHeaterMode mode;
    int16_t kp;
    int16_t ki;
    int16_t kd;
    int16_t *adc_code;
    float oldactual;
    float olderror;
    float error;
    float derror;
    float abserror;
    float actual;
    bool  UsePID;
    float integral;
    float modeled_integral;
    float integral_error;
    float iterm;
    float power;
    bool off_and_lock;           // this flag is set by CANbus 'Contact_Off', initial value is '0'. when 'Contact_off' set a value '0', this flag is set to '1' and heater is set off

} Heater_t;


/* Exported constants --------------------------------------------------------*/

/* Exported variables ------------------------------------------------------- */
extern Heater_t heater;
extern bool gheater_off_and_lock;


/* Exported functions ------------------------------------------------------- */
void HeaterSetPID(Heater_t *heater, const int16_t (*pid)[3]);
void HeaterGetPID(Heater_t *heater, int16_t (*pid)[3]);
void HeaterContactHandler(uint8_t value);
void HeaterContactRelease(void);
void HeaterSetFixPwm(Heater_t *h, float duty_cycle);
void TimerCallBack_Heater(TimerHandle_t xtimer);



#endif /*__HEATERCONTROL_H */


