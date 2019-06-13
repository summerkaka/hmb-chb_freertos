/**
  ******************************************************************************
  * @file    Project/src/adc_app.c
  * @author  XiaTian
  * @version V0.00
  * @date    06-April-2017
  * @brief   adc work mode configuration
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_include.h"

/* Private macro -------------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
uint16_t ADCvalue[ADCDMA_BUF_SIZE] = {0};			    // final result


/* Private function prototypes -----------------------------------------------*/


/* Code begin ----------------------------------------------------------------*/
void ADC_Config(void)
{
	if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }
    /* Start ADC conversion on regular group with transfer by DMA */
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADCvalue, ADCDMA_BUF_SIZE) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  AdcHandle : AdcHandle handle
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
{

}

/**
  * @brief  Conversion DMA half-transfer callback in non blocking mode
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    //todo set flag
}

/**
  * @brief  ADC error callback in non blocking mode
  *        (ADC conversion with interruption or transfer by DMA)
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    Error_Handler();
}

















