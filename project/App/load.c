/**
  ******************************************************************************
  * @file    Project/src/Load.c 
  * @author  
  * @version V0.00
  * @date    
  * @brief   
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "app.h"

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
DioDevice_t MB_Pwr = {
	.status = kLoadOff,
    .port = VGC_EN_GPIO_Port,
    .pin = VGC_EN_Pin,
};
DioDevice_t CCB_Pwr = {
	.status = kLoadOff,
    .port = VCHANNEL_EN_GPIO_Port,
    .pin = VCHANNEL_EN_Pin,
};
DioDevice_t Fan = {
    .status = kLoadOff,
    .port = GCFAN_EN_GPIO_Port,
    .pin = GCFAN_EN_Pin,
};
DioDevice_t Valve_1 = {
    .status = kLoadOff,
    .port = VALVE_1_EN_GPIO_Port,
    .pin = VALVE_1_EN_Pin,
};
DioDevice_t Valve_2 = {
    .status = kLoadOff,
    .port = VALVE_2_EN_GPIO_Port,
    .pin = VALVE_2_EN_Pin,
};
PulseDevice_t Pump_1 = {
	.dio.status = kLoadOff,
    .dio.port = PUMP_EN_GPIO_Port,
    .dio.pin = PUMP_EN_Pin,
    .timer_on = &tmr_pump_on,
    .timer_off = &tmr_pump_off
};
PulseDevice_t Pump_2 = {
	.dio.status = kLoadOff,
    .dio.port = PUMP_EN_GPIO_Port,
    .dio.pin = PUMP_EN_Pin,
    .timer_on = &tmr_pump_on,
    .timer_off = &tmr_pump_off
};
PulseDevice_t PValve = {
    .dio.status = kLoadOff,
    .dio.port = PVALVE_EN_GPIO_Port,
    .dio.pin = PVALVE_EN_Pin,
    .timer_on = &tmr_pvalve_on,
    .timer_off = &tmr_pvalve_off
};
Heater_t Heater = {
    .pwm.status = kLoadOff,
    .pwm.duty = 0,
    .pwm.period = 10000,
    .pwm.tim = &htim3,
    .pwm.tim_channel = TIM_CHANNEL_2,
    .adc_code = &sdadc1_code[SDADC1_CHNL_PT100]
};



/* Code begin ----------------------------------------------------------------*/
void DioSetTo(DioDevice_t *this, uint8_t val)
{
    if (this->status == kLoadError)
        return;
    
    if (val > 0) {
        HAL_GPIO_WritePin(this->port, this->pin, GPIO_PIN_SET);
        this->status = kLoadOn;
    } else {
        HAL_GPIO_WritePin(this->port, this->pin, GPIO_PIN_RESET);
        this->status = kLoadOff;
    }
    return;
}

void PwmSetTo(PwmDevice_t *this, float duty)
{
    if (duty == 0) {
        HAL_TIM_PWM_Stop(this->tim, this->tim_channel);
        this->duty = 0;
        this->status = kLoadOff;
    } else {
        this->duty = duty;
        TIM_OC_InitTypeDef sConfigOC;

        sConfigOC.OCMode = TIM_OCMODE_PWM1;
        sConfigOC.Pulse = (uint32_t)(this->duty * this->period);
        sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
        sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
        sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
        sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
        if (HAL_TIM_PWM_ConfigChannel(this->tim, &sConfigOC, this->tim_channel) != HAL_OK) {
            Error_Handler();
        }
        HAL_TIM_PWM_Start(this->tim, this->tim_channel);
        this->status = kLoadOn;
    }
    return;
}

bool OpenDetect(DioDevice_t *load)
{
    if (load->open_detect_port != NULL && load->open_detect_pin != NULL) {
        if (HAL_GPIO_ReadPin(load->open_detect_port, load->open_detect_pin)) {
            load->status = kLoadOff;
            return false;
        } else {
            load->status = kLoadError;
            return true;
        }
    }
    return false;
}

void timer_pulse_on_callback(TimerHandle_t id)
{
    DioSetTo((DioDevice_t *)id, 1);
}

void timer_pulse_off_callback(TimerHandle_t id)
{
    DioSetTo((DioDevice_t *)id, 0);
}

void Thread_Heater(const void *arg)
{
    float code;
    float vadc_new;
    Heater_t *heater = (Heater_t *)arg;
    TickType_t tick;
    
    while (1) 
    {
        xSemaphoreTake(sem_heater, portMAX_DELAY);
        tick = xTaskGetTickCount();
        vprintf("tick: %d\n\r", tick);
        code = sd1_gain_coe * (*heater->adc_code + 32767);
        vadc_new = code / 65535 * 3.3;                                        // for debug

        heater->temperature = 20.712 * vadc_new * vadc_new + 124.36 * vadc_new - 233.07;
        
        if (heater->temperature <= -100) {
            heater->err_code = -1;
        }
                
        if (heater->off_and_lock == 1) {
            if (heater->mode != kHeaterOff || heater->pwm.status != kLoadOff) {
                heater->mode = kHeaterOff;
                PwmSetTo(&heater->pwm, 0);
            }
        } else if (heater->err_code == -1) {
            heater->mode = kHeaterOff;
            PwmSetTo(&heater->pwm, 0);
        } else if (heater->mode == kHeaterOff && heater->pwm.status != kLoadOff) {                 // off mode
            PwmSetTo(&heater->pwm, 0);
            heater->integral = 0;
            heater->setpoint = 0;
        } else if (heater->mode == kHeaterFixPWM && heater->pwm.status == kLoadOff) {              // fix-pwm mode
            PwmSetTo(&heater->pwm, heater->duty);
        } else if (heater->mode == kHeaterPID) {                                            // pid mode
            heater->oldactual = heater->actual;
            heater->actual = heater->temperature > 0 ? heater->temperature : -heater->temperature;
            //actual = floor(actual);
            heater->olderror = heater->setpoint - heater->oldactual;
            heater->error = heater->setpoint - heater->actual;
            heater->derror = heater->error - heater->olderror;
            heater->abserror = heater->error > 0 ? heater->error : -heater->error;
            heater->power = 0;

            if (!heater->use_pid && heater->abserror <= CONTROL_BAND) {
                heater->use_pid = true;
                heater->integral = heater->modeled_integral;
            } else if (heater->abserror > CONTROL_BAND && heater->use_pid) {
                heater->use_pid = false;
            }

            if (heater->use_pid) {
                heater->integral_error = heater->ki * heater->error;
                heater->integral += heater->integral_error;

                if (heater->integral > MAX_INTEGRAL)
                    heater->integral = MAX_INTEGRAL;
                if (heater->integral < MIN_INTEGRAL)
                    heater->integral = MIN_INTEGRAL;

                heater->iterm = heater->integral / INTEGRAL_SCALE;
                heater->power = (heater->kp * heater->error + heater->iterm + heater->kd * heater->derror)/TEMPER_UNIT;
                //printf("error: %.2f, integral: %.2f, power: %.2f\n\r", error, integral, power);
                if (heater->power > MAX_POWER)
                    heater->power = MAX_POWER;
                if (heater->power < MIN_POWER)
                    heater->power = MIN_POWER;

                if (heater->integral < 0)
                    heater->integral = 0;
            } else {
                if (heater->error > 0)
                    heater->power = MAX_POWER;
                else
                    heater->power = MIN_POWER;
            }
            PwmSetTo(&heater->pwm, heater->power/MAX_POWER);
        }
    }
}







