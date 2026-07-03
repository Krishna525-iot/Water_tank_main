/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#define RF_connector_Pin GPIO_PIN_1
#define RF_connector_GPIO_Port GPIOD
#define Relay1_Pin GPIO_PIN_0
#define Relay1_GPIO_Port GPIOB
#define Relay2_Pin GPIO_PIN_1
#define Relay2_GPIO_Port GPIOB
#define Relay3_Pin GPIO_PIN_2
#define Relay3_GPIO_Port GPIOB
#define SWITCH1_Pin GPIO_PIN_12
#define SWITCH1_GPIO_Port GPIOB
#define SWITCH2_Pin GPIO_PIN_13
#define SWITCH2_GPIO_Port GPIOB
#define SWITCH3_Pin GPIO_PIN_14
#define SWITCH3_GPIO_Port GPIOB
#define SWITCH4_Pin GPIO_PIN_15
#define SWITCH4_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_11
#define LED2_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_12
#define LED3_GPIO_Port GPIOA
#define LORA_SELECT_Pin GPIO_PIN_15
#define LORA_SELECT_GPIO_Port GPIOA
#define LORA_STATUS_Pin GPIO_PIN_6
#define LORA_STATUS_GPIO_Port GPIOB
#define RF_DATA_Pin GPIO_PIN_7
#define RF_DATA_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_8
#define LED4_GPIO_Port GPIOB
#define LED5_Pin GPIO_PIN_9
#define LED5_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
