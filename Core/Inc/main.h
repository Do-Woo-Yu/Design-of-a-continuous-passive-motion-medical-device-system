/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define FA_RESET_Pin GPIO_PIN_1
#define FA_RESET_GPIO_Port GPIOA
#define dummy5_Pin GPIO_PIN_8
#define dummy5_GPIO_Port GPIOE
#define RLY_M_SEL_Pin GPIO_PIN_10
#define RLY_M_SEL_GPIO_Port GPIOE
#define START_SIG_Pin GPIO_PIN_12
#define START_SIG_GPIO_Port GPIOE
#define STOP_SIG_Pin GPIO_PIN_13
#define STOP_SIG_GPIO_Port GPIOE
#define EMS1_SIG_Pin GPIO_PIN_14
#define EMS1_SIG_GPIO_Port GPIOE
#define RUN_LED_Pin GPIO_PIN_12
#define RUN_LED_GPIO_Port GPIOB
#define dummy1_Pin GPIO_PIN_13
#define dummy1_GPIO_Port GPIOB
#define dummy2_Pin GPIO_PIN_14
#define dummy2_GPIO_Port GPIOB
#define dummy3_Pin GPIO_PIN_11
#define dummy3_GPIO_Port GPIOD
#define dummy4_Pin GPIO_PIN_14
#define dummy4_GPIO_Port GPIOD
#define BUZ_OUT_Pin GPIO_PIN_7
#define BUZ_OUT_GPIO_Port GPIOC
#define PWM2_H_Pin GPIO_PIN_8
#define PWM2_H_GPIO_Port GPIOA
#define PWM1_H_Pin GPIO_PIN_9
#define PWM1_H_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
