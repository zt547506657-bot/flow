/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define YFS401_Pin GPIO_PIN_1
#define YFS401_GPIO_Port GPIOA
#define YFS401_EXTI_IRQn EXTI1_IRQn
#define YFS401A2_Pin GPIO_PIN_2
#define YFS401A2_GPIO_Port GPIOA
#define YFS401A2_EXTI_IRQn EXTI2_IRQn
#define YFS401A3_Pin GPIO_PIN_3
#define YFS401A3_GPIO_Port GPIOA
#define YFS401A3_EXTI_IRQn EXTI3_IRQn
#define SCL_Pin GPIO_PIN_5
#define SCL_GPIO_Port GPIOA
#define SDA_Pin GPIO_PIN_7
#define SDA_GPIO_Port GPIOA
#define RES_Pin GPIO_PIN_12
#define RES_GPIO_Port GPIOB
#define DC_Pin GPIO_PIN_13
#define DC_GPIO_Port GPIOB
#define CS_Pin GPIO_PIN_14
#define CS_GPIO_Port GPIOB
#define RELAY1_Pin GPIO_PIN_15
#define RELAY1_GPIO_Port GPIOB
#define RELAY2_Pin GPIO_PIN_8
#define RELAY2_GPIO_Port GPIOA
#define RELAY3_Pin GPIO_PIN_11
#define RELAY3_GPIO_Port GPIOA
#define KEY2_Pin GPIO_PIN_12
#define KEY2_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
