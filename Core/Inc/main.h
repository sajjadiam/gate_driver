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
#define LED1_Pin GPIO_PIN_14
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_15
#define LED2_GPIO_Port GPIOC
#define POT_Pin GPIO_PIN_0
#define POT_GPIO_Port GPIOA
#define NTC1_Pin GPIO_PIN_1
#define NTC1_GPIO_Port GPIOA
#define NTC2_Pin GPIO_PIN_2
#define NTC2_GPIO_Port GPIOA
#define CS1_Pin GPIO_PIN_3
#define CS1_GPIO_Port GPIOA
#define CS2_Pin GPIO_PIN_4
#define CS2_GPIO_Port GPIOA
#define HC165_CLK_Pin GPIO_PIN_5
#define HC165_CLK_GPIO_Port GPIOA
#define LEDX_Pin GPIO_PIN_6
#define LEDX_GPIO_Port GPIOA
#define LEDO_Pin GPIO_PIN_7
#define LEDO_GPIO_Port GPIOA
#define HC165_DATA_Pin GPIO_PIN_0
#define HC165_DATA_GPIO_Port GPIOB
#define HC165_PL_Pin GPIO_PIN_1
#define HC165_PL_GPIO_Port GPIOB
#define PH1_Pin GPIO_PIN_12
#define PH1_GPIO_Port GPIOB
#define PH2_Pin GPIO_PIN_13
#define PH2_GPIO_Port GPIOB
#define NFAULT1_Pin GPIO_PIN_14
#define NFAULT1_GPIO_Port GPIOB
#define NFAULT1B15_Pin GPIO_PIN_15
#define NFAULT1B15_GPIO_Port GPIOB
#define NSLEEP_Pin GPIO_PIN_10
#define NSLEEP_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
