/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define FAULT2_Pin GPIO_PIN_13
#define FAULT2_GPIO_Port GPIOC
#define FAULT2_EXTI_IRQn EXTI15_10_IRQn
#define OCTWarn1_Pin GPIO_PIN_14
#define OCTWarn1_GPIO_Port GPIOC
#define OCTWarn1_EXTI_IRQn EXTI15_10_IRQn
#define OCTWarn2_Pin GPIO_PIN_15
#define OCTWarn2_GPIO_Port GPIOC
#define OCTWarn2_EXTI_IRQn EXTI15_10_IRQn
#define ENC_DIR_Pin GPIO_PIN_6
#define ENC_DIR_GPIO_Port GPIOA
#define DC_CAL1_Pin GPIO_PIN_2
#define DC_CAL1_GPIO_Port GPIOB
#define DC_CAL2_Pin GPIO_PIN_9
#define DC_CAL2_GPIO_Port GPIOC
#define ENC_PG0_Pin GPIO_PIN_15
#define ENC_PG0_GPIO_Port GPIOA
#define Gain_EN_Pin GPIO_PIN_12
#define Gain_EN_GPIO_Port GPIOC
#define nLED_Green_Pin GPIO_PIN_2
#define nLED_Green_GPIO_Port GPIOD
#define EN_Gate_Pin GPIO_PIN_3
#define EN_Gate_GPIO_Port GPIOB
#define Key_Pin GPIO_PIN_5
#define Key_GPIO_Port GPIOB
#define Key_EXTI_IRQn EXTI9_5_IRQn
#define Encoder2_SCL_Pin GPIO_PIN_6
#define Encoder2_SCL_GPIO_Port GPIOB
#define FAULT1_Pin GPIO_PIN_9
#define FAULT1_GPIO_Port GPIOB
#define FAULT1_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
