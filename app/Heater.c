/**
  ******************************************************************************
  * @file    Project/src/HeaterControl.c
  * @author
  * @version V0.00
  * @date
  * @brief   HeaterControl.c
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_include.h"
#include "math.h"

/* Private macro -------------------------------------------------------------*/
#define MIN_INTEGRAL    0
#define TEMPER_UNIT     1
#define MAX_POWER       10000
#define INTEGRAL_SCALE  1
#define CONTROL_BAND    5
#define KPRESET         2
#define MAX_INTEGRAL    MAX_POWER*INTEGRAL_SCALE*TEMPER_UNIT
#define MIN_POWER       0


/* Private typedef -----------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
Heater_t heater = {
    .pwm = {
        .dev_id = 20,
        .tim = &htim3,
        .tim_channel = TIM_CHANNEL_2,
        .status = kLoadOff,
        .period = 10000,
        .duty = 0,
    },
    .kp = DEFAULT_KP,
    .ki = DEFAULT_KI,
    .kd = DEFAULT_KD,
    .mode = kHeaterOff,
    .setpoint = 0,
    .faulty = false,
    .temperature = 25,
    .off_and_lock = 0,
};


/* Code begin ----------------------------------------------------------------*/
void HeaterSetPID(Heater_t *heater, const int16_t (*pid)[3])
{
    heater->kp = ((int16_t *)pid)[0];
    heater->ki = ((int16_t *)pid)[1];
    heater->kd = ((int16_t *)pid)[2];
}

void HeaterGetPID(Heater_t *heater, int16_t (*pid)[3])
{
    ((int16_t *)pid)[0] = heater->kp;
    ((int16_t *)pid)[1] = heater->ki;
    ((int16_t *)pid)[2] = heater->kd;
}

static void CalcTemperature(Heater_t *heater, int16_t adc_code)
{
    float code = sd1_gain_coe * (adc_code + 32767);
    float vadc_new = code / 65535 * 3.3;                                        // for debug

    heater->temperature = 20.712 * vadc_new * vadc_new + 124.36 * vadc_new - 233.07;

    if (heater->temperature <= -100) {
        heater->faulty = true;
    }
}

void HeaterContactHandler(Heater_t *heater, uint8_t value)
{
    heater->off_and_lock = value ? 0 : 1;
}

void TimerCallBack_Heater(TimerHandle_t xtimer)
{
    Heater_t *h = pvTimerGetTimerID(xtimer);

    CalcTemperature(h, *h->adc_code);

    if (h->off_and_lock == 1) {
        if (h->mode != kHeaterOff || h->pwm.status != kLoadOff) {
            h->mode = kHeaterOff;
            PwmSetTo(&h->pwm, 0);
        }
        return;
    }

    if (h->faulty == true) {
        h->mode = kHeaterOff;
        PwmSetTo(&h->pwm, 0);
    } else if (h->mode == kHeaterOff && h->pwm.status != kLoadOff) {                // off mode
        PwmSetTo(&h->pwm, 0);
        h->integral = 0;
        h->setpoint = 0;
    } else if (h->mode == kHeaterFixPwm && h->pwm.status == kLoadOff) {             // fix-pwm mode
        PwmSetTo(&h->pwm, h->pwm.duty);
    } else if (h->mode == kHeaterPID) {                                             // pid mode
        h->oldactual = h->actual;
        h->actual = h->temperature > 0 ? h->temperature : -h->temperature;
        //actual = floor(actual);
        h->olderror = h->setpoint - h->oldactual;
        h->error = h->setpoint - h->actual;
        h->derror = h->error - h->olderror;
        h->abserror = h->error > 0 ? h->error : -h->error;
        h->power = 0;

        if (!h->UsePID && h->abserror <= CONTROL_BAND) {
            h->UsePID = true;
            h->integral = h->modeled_integral;
        } else if (h->abserror > CONTROL_BAND && h->UsePID) {
            h->UsePID = false;
        }

        if (h->UsePID) {
            h->integral_error = h->ki * h->error;
            h->integral += h->integral_error;

            if (h->integral > MAX_INTEGRAL)
                h->integral = MAX_INTEGRAL;
            if (h->integral < MIN_INTEGRAL)
                h->integral = MIN_INTEGRAL;

            h->iterm = h->integral / INTEGRAL_SCALE;
            h->power = (h->kp*h->error + h->iterm + h->kd*h->derror)/TEMPER_UNIT;
            //printf("error: %.2f, integral: %.2f, power: %.2f\n\r", error, integral, power);
            if (h->power > MAX_POWER)
                h->power = MAX_POWER;
            if (h->power < MIN_POWER)
                h->power = MIN_POWER;

            if (h->integral < 0)
                h->integral = 0;
        } else {
            if (h->error > 0)
                h->power = MAX_POWER;
            else
                h->power = MIN_POWER;
        }
        PwmSetTo(&h->pwm, h->power/MAX_POWER);
    }
    //xprintf("heater runtime %d\n\r", osKernelSysTick());
}

void HeaterSetFixPwm(Heater_t *h, float duty_cycle)
{
    if (duty_cycle < 0 || duty_cycle > 1)
        return;
    h->pwm.duty = duty_cycle;
    h->mode = kHeaterFixPwm;
    PwmSetTo(&h->pwm, duty_cycle);
}


// void HeaterThread(void *p)
// {
//     while (1)
//     {
//         if (MB_Pwr.status == kLoadOff) {
//             heater.mode = kHeaterOff;
//         }

//         HeaterRun(&heater);

//         osDelay(20);
//         xprintf("heater runtime %d\n\r", osKernelSysTick());
//     }

// }



