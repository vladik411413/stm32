#include "main.h"

extern uint8_t bit_data; //6-bit data in RFDA_protocol
extern uint8_t msg; //message in RFDA_protocol
extern uint8_t i; //counter in RFDA_protocol

void TIM1_BRK_IRQHandler(void);
void TIM1_UP_IRQHandler(void);
void RFDA(uint8_t bit_data);


