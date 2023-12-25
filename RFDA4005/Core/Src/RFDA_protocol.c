/*
 * RFDA_protocol.c
 *
 *  Created on: Dec 22, 2023
 *      Author: Vlad
 */

#include "RFDA_protocol.h"


uint8_t bit_data=0; //6-bit data in RFDA_protocol
uint8_t msg=0; //message in RFDA_protocol
uint8_t i=0; //counter in RFDA_protocol

void RFDA(uint8_t bit_data){
	//zeroing
	i=0;
	TIM1->SR &= ~TIM_SR_UIF; //CLEAR FLAG
	TIM1->SR &= ~TIM_SR_BIF; //CLEAR FLAG
	TIM1->CCER&=~TIM_CCER_CC1E; //CLK OFF
	TIM1->CCER&=~TIM_CCER_CC2E; //LE OFF
	TIM1->CNT = 0; //COUNTER 0
	TIM1->BDTR|= TIM_BDTR_MOE;//OUTPUT ENABLE

	//data to send
	if(bit_data > 0x3F) bit_data = 0; //if idata > 111111
	msg=bit_data << 1U;
	msg&=0x7E; //01111110
	//0xxxxx0 message

	//start process
	TIM1->CR1|=TIM_CR1_CEN;// Enable the counter by setting the CEN bit
}
/**
  * @brief This function handles TIM1 BRK interrupt.
  */

void TIM1_BRK_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_IRQn 0 */
	//MOE == 0 automaticaly
	TIM1->CR1&=~TIM_CR1_CEN;// Disable the counter by resetting the CEN bit
	TIM1->CCER&=~TIM_CCER_CC1E; //CLK OFF
	TIM1->CCER&=~TIM_CCER_CC2E; //LE OFF
	i = 0;
	bit_data = 0;
	GPIOA->BSRR|=GPIO_BSRR_BS2;
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
	if(i==8U){
		TIM1->EGR|=TIM_EGR_BG; //A break event is generated. MOE bit is cleared and BIF flag is set
	}
	if((msg & (1U<<i))){
		GPIOA->BSRR|=GPIO_BSRR_BS2; //DATA i BIT SET GPIO_BSRR_BS2 (DATA_PORT)
	}
	else{
		GPIOA->BSRR|=GPIO_BSRR_BR2; //DATA i BIT RESET GPIO_BSRR_BR2 (DATA_PORT)
	}
	//LE
	if(i==0 || i==7){
		TIM1->CCER|=TIM_CCER_CC2E; //LE ON
		TIM1->CCER&=~TIM_CCER_CC1E; //CLK OFF
	}
	if(i==1){
		TIM1->CCER&=~TIM_CCER_CC2E; //LE OFF
		TIM1->CCER|=TIM_CCER_CC1E; //CLK ON
	}

	i++;
  /* USER CODE END TIM1_UP_IRQn 0 */
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */
	TIM1->SR &= ~TIM_SR_UIF; //сброс флага
  /* USER CODE END TIM1_UP_IRQn 1 */
}
