/**
  ******************************************************************************
  * @file    Project/src/TimeDelay.c
  * @author
  * @version V0.00
  * @date
  * @brief   timer control
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app.h"

/* Private macro -------------------------------------------------------------*/
//#define CNT_1US 64

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static bool tim18_tick = false;
static uint32_t seconds_tick = 0;


/* Private function prototypes -----------------------------------------------*/


/* Code begin ----------------------------------------------------------------*/
uint32_t GetMinute(void)
{
    return seconds_tick / 60;
}

uint32_t GetSecond(void)
{
    return seconds_tick;
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM18) {
        tim18_tick = true;
        seconds_tick++;
    }
    if (htim->Instance == TIM17) {
        HAL_IncTick();
    }
}

void Seconds_Handler(void)
{
    if (tim18_tick == true) {

        tim18_tick = false;
        
#if DEBUG == 0
        HAL_IWDG_Refresh(&hiwdg);
#endif

        sd1_gain_coe = 54612.5F / (sdadc1_code[SDADC1_CHNL_REF] + 32767);

        //HAL_GPIO_TogglePin(LED_WHITE_GPIO_Port, LED_WHITE_Pin);

//        if (led_white_lock_time == 0) {
//            if ((Battery_1.status >= kstsPreCharge && Battery_1.status <= kstsTrickle) ||
//                    Battery_2.status >= kstsPreCharge && Battery_2.status <= kstsTrickle) {
//                HAL_GPIO_TogglePin(LED_WHITE_GPIO_Port, LED_WHITE_Pin);
//            } else if (Battery_1.status == kstsFinish && Battery_2.status == kstsFinish) {
//                HAL_GPIO_WritePin(LED_WHITE_GPIO_Port, LED_WHITE_Pin, GPIO_PIN_SET);
//            } else {
//                HAL_GPIO_WritePin(LED_WHITE_GPIO_Port, LED_WHITE_Pin, GPIO_PIN_RESET);
//            }
//        } else if (GetSecond() - led_white_lock_time >= 5) {
//            led_white_lock_time = 0;
//        }
//
//        if (led_red_lock_time == 0) {
//            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, (GPIO_PinState)(Battery_1.status == kstsError || Battery_2.status == kstsError));
//        } else if (GetSecond() - led_red_lock_time >= 5) {
//            led_red_lock_time = 0;
//        }
    }
}

