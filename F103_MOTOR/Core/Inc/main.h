/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HB_Pin GPIO_PIN_13
#define HB_GPIO_Port GPIOC
#define ENC12_Pin GPIO_PIN_0
#define ENC12_GPIO_Port GPIOA
#define ENC11_Pin GPIO_PIN_1
#define ENC11_GPIO_Port GPIOA
#define ENC2_1_Pin GPIO_PIN_6
#define ENC2_1_GPIO_Port GPIOA
#define ENC2_2_Pin GPIO_PIN_7
#define ENC2_2_GPIO_Port GPIOA
#define USER_Pin GPIO_PIN_5
#define USER_GPIO_Port GPIOC
#define VBAT_Pin GPIO_PIN_1
#define VBAT_GPIO_Port GPIOB
#define FAULT2_Pin GPIO_PIN_8
#define FAULT2_GPIO_Port GPIOC
#define EN2_Pin GPIO_PIN_9
#define EN2_GPIO_Port GPIOC
#define MTR22_Pin GPIO_PIN_8
#define MTR22_GPIO_Port GPIOA
#define MTR21_Pin GPIO_PIN_9
#define MTR21_GPIO_Port GPIOA
#define MTR12_Pin GPIO_PIN_10
#define MTR12_GPIO_Port GPIOA
#define MTR11_Pin GPIO_PIN_11
#define MTR11_GPIO_Port GPIOA
#define FAULT1_Pin GPIO_PIN_3
#define FAULT1_GPIO_Port GPIOB
#define EN1_Pin GPIO_PIN_4
#define EN1_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
