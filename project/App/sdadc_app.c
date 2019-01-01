/**
  ******************************************************************************
  * @file    Project/src/sdadc_app.c
  * @author
  * @version V0.00
  * @date
  * @brief   application for sdadc
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app.h"

/* Private macro -------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
int16_t sdadc1_code[SDADC1_CHN_NUM] = {24, 24, 24, 24};
float sd1_gain_coe = 0;

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Code begin ----------------------------------------------------------------*/
void
Sdadc_Config(void)
{
    if (HAL_SDADC_CalibrationStart(&hsdadc1, SDADC_CALIBRATION_SEQ_2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_SDADC_PollForCalibEvent(&hsdadc1, HAL_MAX_DELAY) != HAL_OK) {
        Error_Handler();
    }

    HAL_Delay(2);

    HAL_SDADC_Start(&hsdadc1);

    // HAL_SDADC_PollForConversion(&hsdadc1, 1);

    // code = HAL_SDADC_GetValue(&hsdadc1);

    // sd1_gain_coe = (float)32767 / (code + 32767);

    if (HAL_SDADC_InjectedStart_DMA(&hsdadc1, (uint32_t *)sdadc1_code, SDADC1_CHN_NUM) != HAL_OK){   // if set length to 5, and channel num is 1
        Error_Handler();                                                        // then 5 adc results would be saved to array time after time
    }
}
