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
#define SA_SET GPIOA->BSRR = GPIO_PIN_1

#define SA_RESET GPIOA->BSRR = (uint32_t)GPIO_PIN_1 << 16u

#define SB_SET GPIOA->BSRR = GPIO_PIN_2

#define SB_RESET GPIOA->BSRR = (uint32_t)GPIO_PIN_2 << 16u

#define SC_SET GPIOA->BSRR = GPIO_PIN_3

#define SC_RESET GPIOA->BSRR = (uint32_t)GPIO_PIN_3 << 16u

#define SD_SET GPIOA->BSRR = GPIO_PIN_4

#define SD_RESET GPIOA->BSRR = (uint32_t)GPIO_PIN_4 << 16u

#define SE_SET GPIOA->BSRR = GPIO_PIN_5

#define SE_RESET GPIOA->BSRR = (uint32_t)GPIO_PIN_5 << 16u

#define SF_SET GPIOA->BSRR = GPIO_PIN_6

#define SF_RESET GPIOA->BSRR = (uint32_t)GPIO_PIN_6 << 16u

#define SG_SET GPIOA->BSRR = GPIO_PIN_7

#define SG_RESET GPIOA->BSRR = (uint32_t)GPIO_PIN_7 << 16u

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
