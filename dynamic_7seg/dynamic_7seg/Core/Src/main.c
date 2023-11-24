/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "rtc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void segchar (uint8_t seg){
        switch (seg)
        {
                case 1:

                        SA_RESET;SB_SET;SC_SET;SD_RESET;SE_RESET;SF_RESET;SG_RESET;

                        break;

                case 2:

                        SA_SET;SB_SET;SC_RESET;SD_SET;SE_SET;SF_RESET;SG_SET;

                        break;

                case 3:

                        SA_SET;SB_SET;SC_SET;SD_SET;SE_RESET;SF_RESET;SG_SET;

                        break;

                case 4:

                        SA_RESET;SB_SET;SC_SET;SD_RESET;SE_RESET;SF_SET;SG_SET;

                        break;

                case 5:

                        SA_SET;SB_RESET;SC_SET;SD_SET;SE_RESET;SF_SET;SG_SET;

                        break;

                case 6:

                        SA_SET;SB_RESET;SC_SET;SD_SET;SE_SET;SF_SET;SG_SET;

                        break;

                case 7:

                        SA_SET;SB_SET;SC_SET;SD_RESET;SE_RESET;SF_RESET;SG_RESET;

                        break;

                case 8:

                        SA_SET;SB_SET;SC_SET;SD_SET;SE_SET;SF_SET;SG_SET;

                        break;

                case 9:

                        SA_SET;SB_SET;SC_SET;SD_SET;SE_RESET;SF_SET;SG_SET;

                        break;

                case 0:

                        SA_SET;SB_SET;SC_SET;SD_SET;SE_SET;SF_SET;SG_RESET;

                        break;
        }
}
void segment_show(uint8_t n){
	switch(n){
		case 1:
			segchar(time%10);
		break;
		case 2:
			segchar((time/10)%10);
		break;
		case 3:
			segchar((time/100)%10);
		break;
		case 4:
			segchar((time/1000)%10);
		break;
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_RTC_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  //interrupt handling start
  HAL_TIM_OC_Start_IT(&htim4,TIM_CHANNEL_1);
  HAL_TIM_OC_Start_IT(&htim4,TIM_CHANNEL_2);
  HAL_TIM_OC_Start_IT(&htim4,TIM_CHANNEL_3);
  HAL_TIM_OC_Start_IT(&htim4,TIM_CHANNEL_4);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */
	  HAL_Delay(100);
	  time++;
    /* USER CODE BEGIN 3 */
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USER CODE BEGIN 4 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{

	//if ( htim->Instance == TIM4){}
		uint16_t pulse;

		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			pulse = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			/* Set the Capture Compare Register value */

			if(pulse == 0)
			{
				segment_show(1);
				__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (pulse + 25));
			}
			else{
				SA_RESET;SB_RESET;SC_RESET;SD_RESET;SE_RESET;SF_RESET;SG_RESET;
				__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 0);
			}

		}


		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)

		{
			pulse = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			/* Set the Capture Compare Register value */

			if(pulse == 31)
			{
				segment_show(2);
				__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (pulse + 25));
			}
			else{
				SA_RESET;SB_RESET;SC_RESET;SD_RESET;SE_RESET;SF_RESET;SG_RESET;
				__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, 31);
			}
		}

		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)

		{
			pulse = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
			/* Set the Capture Compare Register value */

			if(pulse == 63)
			{
				segment_show(3);
				__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, (pulse + 25));
			}
			else{
				SA_RESET;SB_RESET;SC_RESET;SD_RESET;SE_RESET;SF_RESET;SG_RESET;
				__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, 63);
			}
		}
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)

		{
			pulse = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
			/* Set the Capture Compare Register value */

			if(pulse == 95)
			{
				segment_show(4);
				__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, (pulse + 25));
			}
			else{
				SA_RESET;SB_RESET;SC_RESET;SD_RESET;SE_RESET;SF_RESET;SG_RESET;
				__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, 95);
			}
		}

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
