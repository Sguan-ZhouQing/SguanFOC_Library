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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
uint16_t Encoder_GetCount(void);
float SS_Encoder_GetRad(void);
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Blue_Pin GPIO_PIN_13
#define LED_Blue_GPIO_Port GPIOC
#define LED_Green_Pin GPIO_PIN_14
#define LED_Green_GPIO_Port GPIOC
#define LED_Red_Pin GPIO_PIN_15
#define LED_Red_GPIO_Port GPIOC
#define PORT_Pin GPIO_PIN_3
#define PORT_GPIO_Port GPIOC
#define SD4_Pin GPIO_PIN_0
#define SD4_GPIO_Port GPIOA
#define SD2_Pin GPIO_PIN_1
#define SD2_GPIO_Port GPIOA
#define SD3_Pin GPIO_PIN_2
#define SD3_GPIO_Port GPIOA
#define SD1_Pin GPIO_PIN_0
#define SD1_GPIO_Port GPIOB
#define Key2_Pin GPIO_PIN_12
#define Key2_GPIO_Port GPIOB
#define Key1_Pin GPIO_PIN_9
#define Key1_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
