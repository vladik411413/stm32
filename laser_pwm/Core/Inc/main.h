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

#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_gpio.h"

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
#define LATCH_Pin LL_GPIO_PIN_1
#define LATCH_GPIO_Port GPIOA
#define LATCH_EXTI_IRQn EXTI1_IRQn
#define EM_Pin LL_GPIO_PIN_2
#define EM_GPIO_Port GPIOA
#define EM_EXTI_IRQn EXTI2_IRQn
#define SYNC_Pin LL_GPIO_PIN_6
#define SYNC_GPIO_Port GPIOA
#define D2_Pin LL_GPIO_PIN_10
#define D2_GPIO_Port GPIOB
#define D3_Pin LL_GPIO_PIN_11
#define D3_GPIO_Port GPIOB
#define D4_Pin LL_GPIO_PIN_12
#define D4_GPIO_Port GPIOB
#define D5_Pin LL_GPIO_PIN_13
#define D5_GPIO_Port GPIOB
#define D6_Pin LL_GPIO_PIN_14
#define D6_GPIO_Port GPIOB
#define D7_Pin LL_GPIO_PIN_15
#define D7_GPIO_Port GPIOB
#define LD4_Pin LL_GPIO_PIN_8
#define LD4_GPIO_Port GPIOC
#define LD3_Pin LL_GPIO_PIN_9
#define LD3_GPIO_Port GPIOC
#define TMS_SWDIO_Pin LL_GPIO_PIN_13
#define TMS_SWDIO_GPIO_Port GPIOA
#define TCK_SWCLK_Pin LL_GPIO_PIN_14
#define TCK_SWCLK_GPIO_Port GPIOA
#define PWM_Pin LL_GPIO_PIN_15
#define PWM_GPIO_Port GPIOA
#define D0_Pin LL_GPIO_PIN_8
#define D0_GPIO_Port GPIOB
#define D1_Pin LL_GPIO_PIN_9
#define D1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
