/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

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
#define SW1_Pin GPIO_PIN_0
#define SW1_GPIO_Port GPIOA
#define SW2_Pin GPIO_PIN_1
#define SW2_GPIO_Port GPIOA
#define TX_BLE_Pin GPIO_PIN_2
#define TX_BLE_GPIO_Port GPIOA
#define RX_BLE_Pin GPIO_PIN_3
#define RX_BLE_GPIO_Port GPIOA
#define PWR_BLT_Pin GPIO_PIN_7
#define PWR_BLT_GPIO_Port GPIOA
#define ADC_NTC_Pin GPIO_PIN_0
#define ADC_NTC_GPIO_Port GPIOB
#define PWR_SpO2_Pin GPIO_PIN_1
#define PWR_SpO2_GPIO_Port GPIOB
#define SCL_SPO2_Pin GPIO_PIN_10
#define SCL_SPO2_GPIO_Port GPIOB
#define SDA_SPO2_Pin GPIO_PIN_11
#define SDA_SPO2_GPIO_Port GPIOB
#define OLED_RES_Pin GPIO_PIN_15
#define OLED_RES_GPIO_Port GPIOB
#define TX_DEBUG_Pin GPIO_PIN_9
#define TX_DEBUG_GPIO_Port GPIOA
#define RX_DEBUG_Pin GPIO_PIN_10
#define RX_DEBUG_GPIO_Port GPIOA
#define SCL_OLED_Pin GPIO_PIN_8
#define SCL_OLED_GPIO_Port GPIOB
#define SDA_OLED_Pin GPIO_PIN_9
#define SDA_OLED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
