/*
 *  RFDA_protocol.c
 * 	Control your RFDA4005 attenuator with ease
 *
 *	Timer settings:
 *	TIM1->CR2|=TIM_CR1_URS; //Only counter overflow/underflow generates an update interrupt or DMA request
 *	TIM1->DIER|=TIM_DIER_BIE;//Break iterrupt enable
 *	TIM1->DIER|=TIM_DIER_UIE;//Update interrupt enable
 *
 *  Created on: Dec 25, 2023
 *  Author: Vlad
 */

#include "RFDA_protocol.h"


uint8_t bit_data=0; //6-bit data in RFDA_protocol
uint8_t msg=42; //message in RFDA_protocol
uint8_t i=0; //counter in RFDA_protocol

void RFDA(uint8_t bit_data){
	//zeroing
	i=0;
	TIM1->SR &= ~TIM_SR_UIF; //CLEAR FLAG
	TIM1->SR &= ~TIM_SR_BIF; //CLEAR FLAG
	TIM1->CCER&=~TIM_CCER_CC1E; //CLK OFF
	GPIOA->ODR&=~LE_Pin; //LE OFF

	//data to send
	msg=bit_data << 1U;
	msg&=0x7E; //01111110
	//0xxxxx0 message

	//start process
	TIM1->CR1|=TIM_CR1_CEN;// Enable the counter by setting the CEN bit
	TIM1->EGR|=TIM_EGR_UG;
}
/**
  * @brief This function handles TIM1 BRK interrupt.
  */

void TIM1_BRK_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_IRQn 0 */
	//MOE == 0 automaticaly
	TIM1->CR1&=~TIM_CR1_CEN;// Disable the counter by resetting the CEN bit
	i = 0;
	bit_data = 0;
  /* USER CODE END TIM1_BRK_IRQn 0 */
  /* USER CODE BEGIN TIM1_BRK_IRQn 1 */
	TIM1->SR&=~TIM_SR_UIF; //CLEAR Update interrupt FLAG
	TIM1->SR&=~TIM_SR_BIF; //сброс флага
  /* USER CODE END TIM1_BRK_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */

void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

	switch(i)
	    {
	        case 0U:
	        	TIM1->BDTR|=TIM_BDTR_MOE;//OUTPUT ENABLE
	        	TIM1->CCER|=LL_TIM_CHANNEL_CH2; //LE ON
	            break;
	        case 1U:
	        	TIM1->CCER&=~LL_TIM_CHANNEL_CH2; //LE OFF
	    		TIM1->CCER|=LL_TIM_CHANNEL_CH1; //CLK ON
	    		if(msg & (0x1U<<i)){
	    			GPIOA->ODR|=DATA_Pin; //DATA i BIT SET GPIO_BSRR_BS2 (DATA_PORT)
	    		}
	    		else{
	    			GPIOA->ODR&=~DATA_Pin; //DATA i BIT RESET GPIO_BSRR_BR2 (DATA_PORT)
	    		}
	            break;
	        case 7U:
	        	GPIOA->ODR&=~DATA_Pin; //DATA OFF
	        	TIM1->CCER&=~LL_TIM_CHANNEL_CH1; //CLK OFF
	        	TIM1->CCER|=LL_TIM_CHANNEL_CH2; //LE ON
	        	break;
	        case 8U:
	        	TIM1->CCER&=~LL_TIM_CHANNEL_CH2; //LE OFF
	        	TIM1->EGR|=TIM_EGR_BG; //A break event is generated. MOE bit is cleared and BIF flag is set
	        	break;
	        default:
	        	if(msg & (1U<<i)){
	        		   GPIOA->ODR|=DATA_Pin; //DATA i BIT SET GPIO_BSRR_BS2 (DATA_PORT)
	        	}
	        	else{
	        		   GPIOA->ODR&=~DATA_Pin; //DATA i BIT RESET GPIO_BSRR_BR2 (DATA_PORT)
	        	}
	    }

	i++;
  /* USER CODE END TIM1_UP_IRQn 0 */
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */
	TIM1->SR &= ~TIM_SR_UIF; //сброс флага
  /* USER CODE END TIM1_UP_IRQn 1 */
}
