/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : HanMinung
Created          : 05-03-2021
Modified         : 11-02-2022
Language/ver     : C++ in Keil uVision
/----------------------------------------------------------------*/

#include "stm32f411xe.h"
#include "ecRCC.h"

#ifndef __ECGPIO_H
#define __ECGPIO_H

#define INPUT  0x00
#define OUTPUT 0x01
#define AF     0x02
#define ANALOG 0x03

#define NONE 0x00
#define EC_PU 0x01
#define EC_PD 0x02
#define EC_RE 0x03

#define LOW_SPEED 0x00
#define MEDIUM_SPEED 0x01
#define FAST_SPEED 0x10
#define HIGH_SPEED 0x11

#define PUSH_PULL 0x00
#define	OPEN_DRAIN 0x01

#define	NO_PUPD 0x00
#define	PULL_UP 0x01
#define	PULL_DOWN 0x10
#define	RESERVED 0x11

#define HIGH 1
#define LOW  0

#define LED_PIN 	5

// 7 segement display
#define LED_PA0   0
#define LED_PA1 	1
#define LED_PA2		2
#define LED_PA5 	5
#define LED_PA6 	6
#define LED_PA7 	7
#define LED_PA8 	8
#define LED_PA9 	9
#define LED_PA10 	10

#define LED_PB0 	0
#define LED_PB3 	3
#define LED_PB4 	4
#define LED_PB5 	5
#define LED_PB6 	6
#define LED_PB9   9
#define LED_PB10 	10

#define LED_PC1 	1
#define LED_PC7 	7

#define BUTTON_PIN 13

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

typedef struct{
   
	GPIO_TypeDef *port;
  int pin;
	
} _Pin;
	 
extern unsigned int state[10][8];

void GPIO_init(GPIO_TypeDef *Port, int pin, int mode);

void GPIO_write(GPIO_TypeDef *Port, int pin, int Output);

int  GPIO_read(GPIO_TypeDef *Port, int pin);

void GPIO_mode(GPIO_TypeDef* Port, int pin, int mode);

void GPIO_ospeed(GPIO_TypeDef* Port, int pin, int speed);

void GPIO_otype(GPIO_TypeDef* Port, int pin, int type);

void GPIO_pupd(GPIO_TypeDef* Port, int pin, int pupd);

void bittoggle(GPIO_TypeDef *Port, int pin);

void sevensegment_init();

void sevensegment_decoder(uint8_t num);

void indiv_init(GPIO_TypeDef *Port1,int pin,int TYPE,int OTYPE,int PUPD,int OSPEED);

void LED_init(GPIO_TypeDef *Port1, int pin1, GPIO_TypeDef *Port2, int pin2, GPIO_TypeDef *Port3, int pin3, GPIO_TypeDef *Port4, int pin4);

void LED_toggle(uint8_t cnt);
 
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
