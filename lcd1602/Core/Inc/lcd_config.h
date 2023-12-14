#include "main.h"

#ifndef INC_LCD_CONFIG_H_
#define INC_LCD_CONFIG_H_

/* CONFIG FOR LIBRARY USER */
#define GPIO_PORT GPIOA
#define GPIO_PORT578 GPIOB

//4 pin mode -> pins
#define DATA5_Pin d4_Pin
#define DATA6_Pin d5_Pin
#define DATA7_Pin d6_Pin
#define DATA8_Pin d7_Pin

#define RS_Pin rs_Pin
#define E_Pin  e_Pin
//RW Pin not used,connect to GND

//if you want to work with 8 bit mode uncomment the area which is given below

/*
#define LCD8Bit
#define DATA1_Pin GPIO_PIN_1
#define DATA2_Pin GPIO_PIN_2
#define DATA3_Pin GPIO_PIN_3
#define DATA4_Pin GPIO_PIN_4
*/


#endif /* INC_LCD_CONFIG_H_ */
