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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Laser_Pin GPIO_PIN_13
#define Laser_GPIO_Port GPIOC
#define RST_PANTALLA_Pin GPIO_PIN_8
#define RST_PANTALLA_GPIO_Port GPIOD
#define DC_PANTALLA_Pin GPIO_PIN_9
#define DC_PANTALLA_GPIO_Port GPIOD
#define CS_PANTALLA_Pin GPIO_PIN_10
#define CS_PANTALLA_GPIO_Port GPIOD
#define Btn_RESET_Pin GPIO_PIN_7
#define Btn_RESET_GPIO_Port GPIOC
#define Btn_1_Pin GPIO_PIN_8
#define Btn_1_GPIO_Port GPIOC
#define Btn_2_Pin GPIO_PIN_9
#define Btn_2_GPIO_Port GPIOC
#define Btn_3_Pin GPIO_PIN_8
#define Btn_3_GPIO_Port GPIOA
#define Btn_4_Pin GPIO_PIN_9
#define Btn_4_GPIO_Port GPIOA
#define Lidar_xshutdown_Pin GPIO_PIN_5
#define Lidar_xshutdown_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
