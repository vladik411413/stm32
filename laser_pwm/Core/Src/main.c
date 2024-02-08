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
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "st7735.h"
#include <string.h>
#include "fonts.h"
#include <stdlib.h>
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
extern uint32_t CCR1_IRQ_Data;

extern uint16_t idrdata;
uint16_t display_data = 0;
uint32_t t = 0;
uint32_t info;
uint16_t x=0;
uint16_t y=0;
uint16_t dotx=60;
uint16_t doty=60;
char tstr[32];
char str[32];
char cr1[3]="CR1";
char cr2[3]="CR2";
char sr[3]= " SR";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint16_t ReadData_D0D7(void);
void reverse(char s[]);
void Display_defaults(void);
void Display_update(void);
void itoa(uint16_t n, char s[]);
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
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  
  
  //FREQ capture
  LL_mDelay(1000);
  NVIC_EnableIRQ(TIM3_IRQn);
  NVIC_SetPriority(TIM3_IRQn,1);
  TIM3->CCER|=TIM_CCER_CC1E;
  TIM3->DIER|=TIM_DIER_CC1IE;
  TIM3->CR1|=TIM_CR1_CEN;
  //
  

	NVIC_EnableIRQ(EXTI1_IRQn); //LATCH
  NVIC_SetPriority(EXTI1_IRQn,0);
  EXTI->IMR|=EXTI_IMR_IM1;
  
  NVIC_EnableIRQ(EXTI2_IRQn); //EM
  NVIC_SetPriority(EXTI2_IRQn,0);
  EXTI->IMR|=EXTI_IMR_IM2;
  
  Display_defaults();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

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
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_6);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(24000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* set graphical display */
void Display_defaults(void){
  ST7735_Init();
  ST7735_FillScreen(ST7735_BLACK);
  ST7735_FillRectangle(0, 5, 160, 2, ST7735_WHITE);
  ST7735_WriteString(10, 17, "LASER CONTROL STATUS", Font_7x10, ST7735_WHITE, ST7735_BLACK);
  ST7735_FillRectangle(0, 35, 160, 2, ST7735_WHITE);
  ST7735_WriteString(0, 45, "FREQ,kHz" , Font_11x18, ST7735_WHITE, ST7735_BLACK);
  ST7735_WriteString(0, 65, "DUTY,%", Font_11x18, ST7735_WHITE, ST7735_BLACK);
  ST7735_WriteString(0, 85, "LATCH", Font_11x18, ST7735_WHITE, ST7735_BLACK);
  ST7735_WriteString(0, 105, "LASER", Font_11x18, ST7735_WHITE, ST7735_BLACK);
  ST7735_FillRectangle(95, 35, 2, 93, ST7735_WHITE);
  ST7735_WriteString(100, 45, "WAIT" , Font_11x18, ST7735_WHITE, ST7735_BLACK);
  ST7735_WriteString(100, 65, "WAIT", Font_11x18, ST7735_WHITE, ST7735_BLACK);
  ST7735_WriteString(100, 85, "WAIT", Font_11x18, ST7735_WHITE, ST7735_BLACK);
  ST7735_WriteString(100, 105, "OFF", Font_11x18, ST7735_WHITE, ST7735_BLACK);
}
/* update graphical display */
void Display_update(void){

  if(CCR1_IRQ_Data){
    display_data = 24000/CCR1_IRQ_Data;
    itoa(display_data,tstr);
    ST7735_WriteString(100, 45, tstr , Font_7x10, ST7735_WHITE, ST7735_BLACK);
  }
  if(idrdata){
    display_data = (100*idrdata)/0xFFFF;
    itoa(display_data,tstr);
    ST7735_WriteString(100, 65, tstr, Font_7x10, ST7735_WHITE, ST7735_BLACK);
  }
  if((EM_GPIO_Port->IDR) & GPIO_IDR_IDR2){
      //ST7735_FillRectangle(65, 105, 33, 18, ST7735_BLACK);
    ST7735_WriteString(100, 105, "ON ", Font_7x10, ST7735_WHITE, ST7735_BLACK);
  }
  else{
      //ST7735_FillRectangle(65, 105, 33, 18, ST7735_BLACK);
    ST7735_WriteString(100, 105, "OFF", Font_7x10, ST7735_WHITE, ST7735_BLACK);
  }
}

 /* itoa:  convert n to characters in s */
 void itoa(uint16_t n, char s[])
 {
     uint16_t i;
 

     i = 0;
     do {       /* generate digits in reverse order */
         s[i++] = n % 10 + '0';   /* get next digit */
     } while ((n /= 10) > 0);     /* delete it */
     s[i] = '\0';
     reverse(s);
 }
 
 void reverse(char s[])
 {
     uint16_t i, j;
     char c;
 
     for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
         c = s[i];
         s[i] = s[j];
         s[j] = c;
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
