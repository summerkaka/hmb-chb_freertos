/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MBFAN_OPEN_Pin GPIO_PIN_13
#define MBFAN_OPEN_GPIO_Port GPIOC
#define BOOT_MODE_Pin GPIO_PIN_14
#define BOOT_MODE_GPIO_Port GPIOC
#define VALVE_2_OPEN_Pin GPIO_PIN_15
#define VALVE_2_OPEN_GPIO_Port GPIOC
#define BAT_1_INTR_Pin GPIO_PIN_0
#define BAT_1_INTR_GPIO_Port GPIOC
#define BAT_2_INTR_Pin GPIO_PIN_1
#define BAT_2_INTR_GPIO_Port GPIOC
#define HALL_INTR_Pin GPIO_PIN_2
#define HALL_INTR_GPIO_Port GPIOC
#define GCFAN_EN_Pin GPIO_PIN_3
#define GCFAN_EN_GPIO_Port GPIOC
#define GCFAN_OPEN_Pin GPIO_PIN_0
#define GCFAN_OPEN_GPIO_Port GPIOA
#define LTC4236_IMON_Pin GPIO_PIN_1
#define LTC4236_IMON_GPIO_Port GPIOA
#define PUMP_2_OPEN_Pin GPIO_PIN_2
#define PUMP_2_OPEN_GPIO_Port GPIOA
#define NTC_1_ADC_Pin GPIO_PIN_3
#define NTC_1_ADC_GPIO_Port GPIOA
#define NTC_2_ADC_Pin GPIO_PIN_4
#define NTC_2_ADC_GPIO_Port GPIOA
#define VADAPTOR_ADC_Pin GPIO_PIN_5
#define VADAPTOR_ADC_GPIO_Port GPIOA
#define VCC12_ADC_Pin GPIO_PIN_6
#define VCC12_ADC_GPIO_Port GPIOA
#define ZERO_1_Pin GPIO_PIN_7
#define ZERO_1_GPIO_Port GPIOA
#define ZERO_2_Pin GPIO_PIN_4
#define ZERO_2_GPIO_Port GPIOC
#define ZERO_3_Pin GPIO_PIN_5
#define ZERO_3_GPIO_Port GPIOC
#define PT100_ADC_Pin GPIO_PIN_1
#define PT100_ADC_GPIO_Port GPIOB
#define VREF_2V75_Pin GPIO_PIN_2
#define VREF_2V75_GPIO_Port GPIOB
#define GAS_2_ADC_Pin GPIO_PIN_8
#define GAS_2_ADC_GPIO_Port GPIOE
#define GAS_1_ADC_Pin GPIO_PIN_9
#define GAS_1_ADC_GPIO_Port GPIOE
#define VALVE_1_OPEN_Pin GPIO_PIN_14
#define VALVE_1_OPEN_GPIO_Port GPIOB
#define PUMP_1_OPEN_Pin GPIO_PIN_15
#define PUMP_1_OPEN_GPIO_Port GPIOB
#define PUMP_EN_Pin GPIO_PIN_8
#define PUMP_EN_GPIO_Port GPIOD
#define VCHANNEL_EN_Pin GPIO_PIN_7
#define VCHANNEL_EN_GPIO_Port GPIOC
#define VALVE_1_EN_Pin GPIO_PIN_8
#define VALVE_1_EN_GPIO_Port GPIOC
#define VALVE_2_EN_Pin GPIO_PIN_9
#define VALVE_2_EN_GPIO_Port GPIOC
#define VGC_EN_Pin GPIO_PIN_8
#define VGC_EN_GPIO_Port GPIOA
#define I2C1_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOF
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOF
#define BAT_SW_MCU_Pin GPIO_PIN_15
#define BAT_SW_MCU_GPIO_Port GPIOA
#define PVALVE_EN_Pin GPIO_PIN_10
#define PVALVE_EN_GPIO_Port GPIOC
#define LED_WHITE_Pin GPIO_PIN_11
#define LED_WHITE_GPIO_Port GPIOC
#define LED_RED_Pin GPIO_PIN_12
#define LED_RED_GPIO_Port GPIOC
#define BAT_SW_MON_Pin GPIO_PIN_2
#define BAT_SW_MON_GPIO_Port GPIOD
#define CHARGE_1_EN_Pin GPIO_PIN_3
#define CHARGE_1_EN_GPIO_Port GPIOB
#define CHARGE_2_EN_Pin GPIO_PIN_4
#define CHARGE_2_EN_GPIO_Port GPIOB
#define HEATER_PWM_Pin GPIO_PIN_5
#define HEATER_PWM_GPIO_Port GPIOB
#define I2C0_SCL_Pin GPIO_PIN_6
#define I2C0_SCL_GPIO_Port GPIOB
#define I2C0_SDA_Pin GPIO_PIN_7
#define I2C0_SDA_GPIO_Port GPIOB
#define BAT_1_EN_Pin GPIO_PIN_8
#define BAT_1_EN_GPIO_Port GPIOB
#define BAT_2_EN_Pin GPIO_PIN_9
#define BAT_2_EN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
