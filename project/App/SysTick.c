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
static uint8_t tim18_tick = false;
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
    BaseType_t higher_woken;
    if (htim->Instance == TIM18) {
        tim18_tick++;
        if (tim18_tick >= 50) {
            seconds_tick++;
            tim18_tick = 0;
            sd1_gain_coe = 54612.5F / (sdadc1_code[SDADC1_CHNL_REF] + 32767);
            #if DEBUG == 0
                HAL_IWDG_Refresh(&hiwdg);
            #endif
        }
        xSemaphoreGiveFromISR(sem_heater, &higher_woken);
        if (higher_woken == pdTRUE)
            portYIELD_FROM_ISR(pdTRUE);
            
    }
    if (htim->Instance == TIM17) {
        HAL_IncTick();
    }
}





