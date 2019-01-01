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

    }
}




