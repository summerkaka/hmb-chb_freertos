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

//void PulseRun(PulseDevice_t *pulse)
//{
//    if (pulse->sm_state == 0) {
//        return;
//    } else if (pulse->sm_state == 1) {
//        if (xTaskGetTickCount() - pulse->origin >= pulse->start_time) {
//            pulse->sm_state = 2;
//            DioSetTo(&pulse->dio, 1);
//            printf("PulseRun(): enter sm_2\n\r");
//        }
//    } else if (pulse->sm_state == 2) {
//        if (xTaskGetTickCount() - pulse->origin >= pulse->stop_time) {
//            pulse->sm_state = 0;
//            DioSetTo(&pulse->dio, 0);
//            printf("PulseRun(): enter sm_0\n\r");
//        }
//    }
//}

void timer_pulse_on_callback(TimerHandle_t id)
{
    DioSetTo((DioDevice_t *)id, 1);
}

void timer_pulse_off_callback(TimerHandle_t id)
{
    DioSetTo((DioDevice_t *)id, 0);
}









