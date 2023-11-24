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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define A_Pin GPIO_PIN_1
#define A_GPIO_Port GPIOA
#define B_Pin GPIO_PIN_2
#define B_GPIO_Port GPIOA
#define C_Pin GPIO_PIN_3
#define C_GPIO_Port GPIOA
#define D_Pin GPIO_PIN_4
#define D_GPIO_Port GPIOA
#define E_Pin GPIO_PIN_5
#define E_GPIO_Port GPIOA
#define F_Pin GPIO_PIN_6
#define F_GPIO_Port GPIOA
#define G_Pin GPIO_PIN_7
#define G_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define SA_SET HAL_GPIO_WritePin(GPIOA, A_Pin, GPIO_PIN_SET)

#define SA_RESET HAL_GPIO_WritePin(GPIOA, A_Pin, GPIO_PIN_RESET)

#define SB_SET HAL_GPIO_WritePin(GPIOA, B_Pin, GPIO_PIN_SET)

#define SB_RESET HAL_GPIO_WritePin(GPIOA, B_Pin, GPIO_PIN_RESET)

#define SC_SET HAL_GPIO_WritePin(GPIOA, C_Pin, GPIO_PIN_SET)

#define SC_RESET HAL_GPIO_WritePin(GPIOA, C_Pin, GPIO_PIN_RESET)

#define SD_SET HAL_GPIO_WritePin(GPIOA, D_Pin, GPIO_PIN_SET)

#define SD_RESET HAL_GPIO_WritePin(GPIOA, D_Pin, GPIO_PIN_RESET)

#define SE_SET HAL_GPIO_WritePin(GPIOA, E_Pin, GPIO_PIN_SET)

#define SE_RESET HAL_GPIO_WritePin(GPIOA, E_Pin, GPIO_PIN_RESET)

#define SF_SET HAL_GPIO_WritePin(GPIOA, F_Pin, GPIO_PIN_SET)

#define SF_RESET HAL_GPIO_WritePin(GPIOA, F_Pin, GPIO_PIN_RESET)

#define SG_SET HAL_GPIO_WritePin(GPIOA, G_Pin, GPIO_PIN_SET)

#define SG_RESET HAL_GPIO_WritePin(GPIOA, G_Pin, GPIO_PIN_RESET)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
