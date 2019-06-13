/**
  ******************************************************************************
  * @file    Project/src/Loadcpp.cpp
  * @author
  * @version V0.00
  * @date
  * @brief
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_include.h"

/* Private macro -------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
DigitalOut MB_Pwr = {
    .dev_id = 0,
    .out_port = VGC_EN_GPIO_Port,
    .out_pin = VGC_EN_Pin,
    .detect_port = MBFAN_OPEN_GPIO_Port,
    .detect_pin= MBFAN_OPEN_Pin,
    .err_code = 0,
    .status = kLoadOff,
    .connect_time = 0,
    .disconnect_time = 0,
    .turnoff_time = 0
};

DigitalOut CCB_Pwr = {
    .dev_id = 1,
    .out_port = VCHANNEL_EN_GPIO_Port,
    .out_pin = VCHANNEL_EN_Pin,
    .detect_port = NULL,
    .detect_pin = NULL,
    .err_code = 0,
    .status = kLoadOff,
    .connect_time = 0,
    .disconnect_time = 0,
    .turnoff_time = 0
};

DigitalOut Valve_1 = {
    .dev_id = 2,
    .out_port = VALVE_1_EN_GPIO_Port,
    .out_pin = VALVE_1_EN_Pin,
    .detect_port = VALVE_1_OPEN_GPIO_Port,
    .detect_pin= VALVE_1_OPEN_Pin,
    .err_code = 0,
    .status = kLoadOff,
    .connect_time = 0,
    .disconnect_time = 0,
    .turnoff_time = 0
};

DigitalOut Valve_2 = {
    .dev_id = 3,
    .out_port = VALVE_2_EN_GPIO_Port,
    .out_pin = VALVE_2_EN_Pin,
    .detect_port = VALVE_2_OPEN_GPIO_Port,
    .detect_pin= VALVE_2_OPEN_Pin,
    .err_code = 0,
    .status = kLoadOff,
    .connect_time = 0,
    .disconnect_time = 0,
    .turnoff_time = 0
};

DigitalOut Fan = {
    .dev_id = 4,
    .out_port = GCFAN_EN_GPIO_Port,
    .out_pin = GCFAN_EN_Pin,
    .detect_port = GCFAN_OPEN_GPIO_Port,
    .detect_pin= GCFAN_OPEN_Pin,
    .err_code = 0,
    .status = kLoadOff,
    .connect_time = 0,
    .disconnect_time = 0,
    .turnoff_time = 0
};

PulseOut Pump_1 = {
    .dio = {
        .dev_id = 10,
        .out_port = PUMP_EN_GPIO_Port,
        .out_pin = PUMP_EN_Pin,
        .detect_port = PUMP_1_OPEN_GPIO_Port,
        .detect_pin= PUMP_1_OPEN_Pin,
        .err_code = 0,
        .status = kLoadOff,
        .connect_time = 0,
        .disconnect_time = 0,
        .turnoff_time = 0
    },
    // .origin = 0,
    // .start_time = 0,
    // .stop_time = 0,
    // .sm = 0
};

PulseOut Pump_2 = {
    .dio = {
        .dev_id = 11,
        .out_port = PUMP_EN_GPIO_Port,
        .out_pin = PUMP_EN_Pin,
        .detect_port = PUMP_2_OPEN_GPIO_Port,
        .detect_pin= PUMP_2_OPEN_Pin,
        .err_code = 0,
        .status = kLoadOff,
        .connect_time = 0,
        .disconnect_time = 0,
        .turnoff_time = 0
    },
    // .origin = 0,
    // .start_time = 0,
    // .stop_time = 0,
    // .sm = 0,
};

PulseOut PValve = {
    .dio = {
        .dev_id = 12,
        .out_port = PVALVE_EN_GPIO_Port,
        .out_pin = PVALVE_EN_Pin,
        .detect_port = NULL,
        .detect_pin= NULL,
        .err_code = 0,
        .status = kLoadOff,
        .connect_time = 0,
        .disconnect_time = 0,
        .turnoff_time = 0
    },
    // .origin = 0,
    // .start_time = 0,
    // .stop_time = 0,
    // .sm = 0
};


/* Code begin ----------------------------------------------------------------*/
void DioSetTo(DigitalOut *dio, bool value)
{
    if (value == 0) {
        HAL_GPIO_WritePin(dio->out_port, dio->out_pin, GPIO_PIN_RESET);
        dio->status = dio->status == kLoadError ? kLoadError : kLoadOff;
        dio->turnoff_time = GetSecond();
    } else if (value == 1) {
        if (dio->status != kLoadError) {
            HAL_GPIO_WritePin(dio->out_port, dio->out_pin, GPIO_PIN_SET);
            dio->status = kLoadOn;
        }
    }
    xprintf("dio device %d set to: %d\n\r", dio->dev_id, value);
}

uint8_t DioCheckFaulty(DigitalOut *dio)
{
    if (dio->detect_port != NULL && dio->detect_pin != NULL) {
        if (dio->status == kLoadOff && GetSecond() - dio->turnoff_time >= 2) {
            if (!HAL_GPIO_ReadPin(dio->detect_port, dio->detect_pin)) {
                if (dio->disconnect_time == 0)
                    dio->disconnect_time = GetSecond();
                else if (GetSecond() - dio->disconnect_time >= 5) {
                    dio->err_code = 1;
                    dio->status = kLoadError;
                    dio->disconnect_time = GetSecond();
                    dio->connect_time = 0;
                    xprintf("dev_id %d open detect\n\r", dio->dev_id);
                }
            } else {
                dio->disconnect_time = 0;
                dio->err_code = 0;
            }
        } else if (dio->status == kLoadError) {
            if (HAL_GPIO_ReadPin(dio->detect_port, dio->detect_pin)) {
                if (dio->connect_time == 0)
                    dio->connect_time = GetSecond();
                else if (GetSecond() - dio->connect_time >= 5) {
                    dio->err_code = 0;
                    dio->status = kLoadOff;
                    dio->connect_time = GetSecond();
                    dio->disconnect_time = 0;
                    xprintf("dev_id %d connecting\n\r", dio->dev_id);
                }
            }
        }
    }
    return dio->err_code;
}

void PwmSetTo(PwmOut *pwm, float value)
{
    if (value < 0 || value > 1)
        return;

    pwm->duty = value;

    if (pwm->duty == 0) {
        HAL_TIM_PWM_Stop(pwm->tim, pwm->tim_channel);
        pwm->status = kLoadOff;
    } else {
        TIM_OC_InitTypeDef sConfigOC;

        sConfigOC.OCMode = TIM_OCMODE_PWM1;
        sConfigOC.Pulse = (uint32_t)(pwm->duty * pwm->period);
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
        sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
        sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
        if (HAL_TIM_PWM_ConfigChannel(pwm->tim, &sConfigOC, pwm->tim_channel) != HAL_OK) {
            Error_Handler();
        }
        HAL_TIM_PWM_Start(pwm->tim, pwm->tim_channel);
        pwm->status = kLoadOn;
    }
}

void PwmSetDuty(PwmOut *pwm, float value)
{
    pwm->duty = value > 1 ? 1 : value;
    pwm->duty = pwm->duty  < 0 ? 0 : pwm->duty;
}

void PulseOutSetStartTime(PulseOut *p, uint32_t time)
{
    // p->start_time = time;
    if (xTimerChangePeriod(p->timer_start, time/portTICK_PERIOD_MS, 20/portTICK_PERIOD_MS) == pdTRUE)
        xprintf("PulseOut::SetStartTime(): dev %d set %d\n\r", p->dio.dev_id, time);
    xTimerStop(p->timer_start, 1000/portTICK_PERIOD_MS);
}

void PulseOutSetStopTime(PulseOut *p, uint32_t time)
{
    // p->stop_time = time;
    if (xTimerChangePeriod(p->timer_stop, time / portTICK_PERIOD_MS, 20/portTICK_PERIOD_MS) == pdTRUE)
        xprintf("PulseOut::SetStopTime(): dev %d set %d\n\r", p->dio.dev_id, time);
    xTimerStop(p->timer_stop, 1000/portTICK_PERIOD_MS);
}

void PulseOutStartRun(PulseOut *p)
{
    // p->sm = 1;
    // p->origin = HAL_GetTick();
    uint32_t start = xTimerGetPeriod(p->timer_start);
    uint32_t stop = xTimerGetPeriod(p->timer_stop);

    if (start != 0 && stop != 0)
        if (xTimerStart(p->timer_start, 20))
            xprintf("PulseOut StartTimer: dev %d\n\r", p->dio.dev_id);
        else
            xprintf("PulseOut fail to StartTimer dev %d\n\r", p->dio.dev_id);
    else
        xprintf("PulseOut StartTimer: dev %d invalid period, %d, %d\n\r", p->dio.dev_id, start, stop);
}

void PulseOutStopRun(PulseOut *p)
{
    // DioSetTo(&p->dio, 0);
    // p->sm = 0;
    if (xTimerIsTimerActive(p->timer_start))
        xTimerStop(p->timer_start, 1000/portTICK_PERIOD_MS);
    if (xTimerIsTimerActive(p->timer_stop))
        xTimerStop(p->timer_stop, 1000/portTICK_PERIOD_MS);
    xprintf("PulseOut StopTimer: dev %d stop\n\r", p->dio.dev_id);
}

//static void PulseOutRun(PulseOut *p)
//{
//    p->dio.err_code = DioCheckFaulty(&p->dio);
//
//    if (p->dio.err_code != 0)
//        return;
//
//    if (p->sm == 0) {
//        return;
//    } else if (p->sm == 1) {
//        if (HAL_GetTick() - p->origin >= p->start_time) {
//            p->sm = 2;
//            DioSetTo(&p->dio, 1);
//            xprintf("PulseOut::Run(): dev %d enter sm_2\n\r", p->dio.dev_id);
//        }
//    } else if (p->sm == 2) {
//        if (HAL_GetTick() - p->origin >= p->start_time + p->stop_time) {
//            p->sm = 0;
//            DioSetTo(&p->dio, 0);
//            xprintf("PulseOut::Run(): dev %d enter sm_0\n\r", p->dio.dev_id);
//        }
//    }
//}

void PulseOutOn(TimerHandle_t xtimer)
{
    PulseOut *p = pvTimerGetTimerID(xtimer);
    DioSetTo(&p->dio, 1);
    xTimerStart(p->timer_stop, portMAX_DELAY);
}

void PulseOutOff(TimerHandle_t xtimer)
{
    PulseOut *p = pvTimerGetTimerID(xtimer);
    DioSetTo(&p->dio, 0);
    xTimerChangePeriod(p->timer_start, 0, 10 / portTICK_PERIOD_MS);
    xTimerChangePeriod(p->timer_stop, 0, 10 / portTICK_PERIOD_MS);
}





