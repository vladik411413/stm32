/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
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
uint32_t i = 0;
uint32_t sum = 0;
uint32_t t = 0;
uint32_t g = 0;
uint16_t k = 40;
uint16_t x = 0;
uint8_t direction = 0;
uint32_t delayDIR0 = 10U; //delayDIR<delayIRQn!!!!
uint32_t delayIRQn0 = 20U;
//delayDIR<delayIRQn!!!!
uint32_t delayIRQn = 20U;
uint16_t IRQn_PIN = BUT_Pin;
GPIO_TypeDef* IRQn_PIN_Port = BUT_GPIO_Port;
uint16_t IRQn_PIN_IDLE_POLARITY = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	TIM2->CR1&=~TIM_CR1_UDIS;
	

	LD4_GPIO_Port->ODR|=LD4_Pin; 
	LD3_GPIO_Port->ODR&=~LD3_Pin;
	direction=0;
	//DIR_GPIO_Port->ODR&=~DIR_Pin;
	ENA_GPIO_Port->ODR|=ENA_Pin;
	
	HAL_Delay(1000);
	
	EXTI->IMR|=EXTI_IMR_IM0; //warning! what if interrupt at start because of gpio hal set
	HAL_NVIC_EnableIRQ(EXTI0_IRQn); //warning! what if interrupt at start because of gpio hal set
	EXTI->IMR|=EXTI_IMR_IM1; //warning! what if interrupt at start because of gpio hal set
	HAL_NVIC_EnableIRQ(EXTI1_IRQn); //warning! what if interrupt at start because of gpio hal set
	EXTI->IMR|=EXTI_IMR_IM2; //warning! what if interrupt at start because of gpio hal set
	HAL_NVIC_EnableIRQ(EXTI2_IRQn); //warning! what if interrupt at start because of gpio hal set
	EXTI->IMR|=EXTI_IMR_IM3; //warning! what if interrupt at start because of gpio hal set
	HAL_NVIC_EnableIRQ(EXTI3_IRQn); //warning! what if interrupt at start because of gpio hal set
	
	HAL_Delay(1000);
	
	  //ADC calibration
  HAL_ADC_Stop(&hadc1);
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start(&hadc1);
	
	//TIM3->CR1|=TIM_CR1_CEN; //counter enable
	//TIM3->CCER|=TIM_CCER_CC1E;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if((EXTI->IMR)&(EXTI_IMR_IM0|EXTI_IMR_IM1|EXTI_IMR_IM2|EXTI_IMR_IM3)){
			
		if(i >= 20){

		x = sum/20;
		k = x/8+1;
		delayDIR0 = 2+200/k; //delayDIR<delayIRQn!!!!
		delayIRQn0 = 4+400/k;
		delayIRQn = delayIRQn0;
		TIM2->ARR=80*k+250;
		TIM2->CCR1=40*k+125;
		TIM2->EGR|=TIM_EGR_UG;
		sum = 0;
		i = 0;
		}
		else{
			ADC1->CR2|=ADC_CR2_SWSTART;
			sum+= ADC1->DR;
			i++;
			}
		}
		
		HAL_Delay(10);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	
	TIM2->CCER&=~TIM_CCER_CC1E; //pwm disable output
	
	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	EXTI->IMR&=~EXTI_IMR_IM0;
	HAL_NVIC_DisableIRQ(EXTI1_IRQn);
	EXTI->IMR&=~EXTI_IMR_IM1;
	HAL_NVIC_DisableIRQ(EXTI2_IRQn);
	EXTI->IMR&=~EXTI_IMR_IM2;
	HAL_NVIC_DisableIRQ(EXTI3_IRQn);
	EXTI->IMR&=~EXTI_IMR_IM3;
	
	
	switch(GPIO_Pin){
		
		case BUT_Pin:
			
			//for BUT_Pin
			EXTI->PR|=EXTI_PR_PR0;   //clear interrupt pending
			//
		
			if(direction==1){
			LD4_GPIO_Port->ODR|=LD4_Pin; 
			LD3_GPIO_Port->ODR&=~LD3_Pin; 
			DIR_GPIO_Port->ODR&=~DIR_Pin;
			direction=0;
			}
			else{
			LD3_GPIO_Port->ODR|=LD3_Pin; 
			LD4_GPIO_Port->ODR&=~LD4_Pin; 
			DIR_GPIO_Port->ODR|=DIR_Pin;
			direction=1;
			}
			IRQn_PIN = BUT_Pin;
			IRQn_PIN_Port = BUT_GPIO_Port;
			IRQn_PIN_IDLE_POLARITY = 0U;
			
			TIM2->DIER|=TIM_DIER_UIE; //update interrupt enable
			//TIM2->CR1&=~TIM_CR1_UDIS; //update ev.gen enable
			TIM2->CR1|=TIM_CR1_CEN; //counter enable
			break;
			
		case L_Pin:
			
			//for L_Pin
			EXTI->PR|=EXTI_PR_PR1;
			//
			LD4_GPIO_Port->ODR|=LD4_Pin; 
			LD3_GPIO_Port->ODR&=~LD3_Pin;
			direction=0;
			DIR_GPIO_Port->ODR&=~DIR_Pin;
		
			IRQn_PIN = L_Pin;
			IRQn_PIN_Port = L_GPIO_Port;
			IRQn_PIN_IDLE_POLARITY = L_Pin;
		
			TIM2->DIER|=TIM_DIER_UIE; //update interrupt enable
			//TIM2->CR1&=~TIM_CR1_UDIS; //update ev.gen enable
			TIM2->CR1|=TIM_CR1_CEN; //counter enable
			break;
		
		case R_Pin:
			
			//for R_Pin
			EXTI->PR|=EXTI_PR_PR2;
			//
			LD3_GPIO_Port->ODR|=LD3_Pin; 
			LD4_GPIO_Port->ODR&=~LD4_Pin;
			direction=1;
			DIR_GPIO_Port->ODR|=DIR_Pin;
		
			IRQn_PIN = R_Pin;
			IRQn_PIN_Port = R_GPIO_Port;
			IRQn_PIN_IDLE_POLARITY = R_Pin;
		
			TIM2->DIER|=TIM_DIER_UIE; //update interrupt enable
			//TIM2->CR1&=~TIM_CR1_UDIS; //update ev.gen enable
			TIM2->CR1|=TIM_CR1_CEN; //counter enable
			break;
		
		case LR_Pin:
						
			//for BUT_Pin
			EXTI->PR|=EXTI_PR_PR3;   //clear interrupt pending
			//
		
			if(direction==1){
			LD4_GPIO_Port->ODR|=LD4_Pin; 
			LD3_GPIO_Port->ODR&=~LD3_Pin; 
			DIR_GPIO_Port->ODR&=~DIR_Pin;
			direction=0;
			}
			else{
			LD3_GPIO_Port->ODR|=LD3_Pin; 
			LD4_GPIO_Port->ODR&=~LD4_Pin; 
			DIR_GPIO_Port->ODR|=DIR_Pin;
			direction=1;
			}
			IRQn_PIN = LR_Pin;
			IRQn_PIN_Port = LR_GPIO_Port;
			IRQn_PIN_IDLE_POLARITY = LR_Pin;
			
			TIM2->DIER|=TIM_DIER_UIE; //update interrupt enable
			//TIM2->CR1&=~TIM_CR1_UDIS; //update ev.gen enable
			TIM2->CR1|=TIM_CR1_CEN; //counter enable
			break;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	
	if(IRQn_PIN_IDLE_POLARITY == ((IRQn_PIN_Port->IDR) & IRQn_PIN)){
			t++;
	}
	
	if(g>=delayDIR0){
		TIM2->CCER|=TIM_CCER_CC1E; //pwm enable output
	}else{
			g++;
	}
	
	if(t>=delayIRQn){
			if(IRQn_PIN_IDLE_POLARITY ^ ((IRQn_PIN_Port->IDR) & IRQn_PIN)){
				delayIRQn+=delayIRQn0;
			}
			else{
			t=0;
			g=0;
			delayIRQn=delayIRQn0;
			TIM2->DIER&=~TIM_DIER_UIE; //update interrupt disable
			//TIM2->CR1|=TIM_CR1_UDIS; //update ev.gen disable
					EXTI->IMR|=EXTI_IMR_IM0;
					HAL_NVIC_EnableIRQ(EXTI0_IRQn);
					EXTI->IMR|=EXTI_IMR_IM1;
					HAL_NVIC_EnableIRQ(EXTI1_IRQn);
					EXTI->IMR|=EXTI_IMR_IM2;
					HAL_NVIC_EnableIRQ(EXTI2_IRQn);
					EXTI->IMR|=EXTI_IMR_IM3;
					HAL_NVIC_EnableIRQ(EXTI3_IRQn);
			}
		}
	__HAL_TIM_CLEAR_IT(htim,TIM2_IRQn);
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
