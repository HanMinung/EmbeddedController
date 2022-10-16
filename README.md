# Embedded Controller - STM32f411 HAL API

Documentation for HAL functions

Written by : Han Minung 

Course: Embedded Controller

Program: C/C++

IDE/Compiler: Keil uVision 5

OS: WIn10

MCU: STM32F411RE (Nucleo-64)



## GPIO Digital I/O

* ecGPIO.h

```c
#include "stm32f411xe.h"
#include "ecRCC.h"

#ifndef __ECGPIO_H
#define __ECGPIO_H

// MODER
#define INPUT  0x00
#define OUTPUT 0x01
#define AF     0x02
#define ANALOG 0x03

// PUDD
#define NONE 0x00
#define EC_PU 0x01
#define EC_PD 0x02
#define EC_RE 0x03

// OSPEED
#define LOW_SPEED 0x00
#define MEDIUM_SPEED 0x01
#define FAST_SPEED 0x10
#define HIGH_SPEED 0x11

// OTYPE
#define PUSH_PULL 0x00
#define	OPEN_DRAIN 0x01

#define	NO_PULL_UP 0x00
#define	PULL_UP 0x01
#define	PULL_DOWN 0x10
#define	RESERVED 0x11

// IDR & ODR
#define HIGH 1
#define LOW  0

#define LED_PIN 	5

// 7 segement display
#define LED_PA5 	5
#define LED_PA6 	6
#define LED_PA7 	7
#define LED_PB6 	6
#define LED_PC7 	7
#define LED_PA8 	8
#define LED_PA9 	9
#define LED_PB10 	10
#define BUTTON_PIN  13

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

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
void LED_init();
void LED_toggle(uint8_t cnt);
 
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
```



## ecRCC.h

```c
#ifndef __EC_RCC_H
#define __EC_RCC_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

//#include "stm32f411xe.h"

void RCC_HSI_init(void);				// Internal clock init
void RCC_PLL_init(void);				// PLL clock init
void RCC_GPIOA_enable(void);
void RCC_GPIOB_enable(void);
void RCC_GPIOC_enable(void);
void RCC_GPIOD_enable(void);
void RCC_GPIOE_enable(void);
void RCC_GPIOH_enable(void);
// void RCC_GPIO_enable(GPIO_TypeDef * GPIOx);

extern int EC_SYSCL;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
```

##### void RCC_GPIOA_enable(void)

- RCC peripheral clock enable register
- internal structure 

```c
/********************  Bit definition for RCC_AHB1ENR register  ***************/
#define RCC_AHB1ENR_GPIOAEN_Pos            (0U)                                
#define RCC_AHB1ENR_GPIOAEN_Msk            (0x1UL << RCC_AHB1ENR_GPIOAEN_Pos)   /*!< 0x00000001 */
#define RCC_AHB1ENR_GPIOAEN                RCC_AHB1ENR_GPIOAEN_Msk  

void RCC_GPIOA_enable()
{
	// HSI is used as system clock         
	// RCC_HSI_init();
	// RCC Peripheral Clock Enable Register 
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
}
```

* Structure of RCC-->AHB1 enable register 

<img src="https://user-images.githubusercontent.com/99113269/196023402-3e0c8d58-44e4-4fd8-a16f-672243692333.png" alt="화면 캡처 2022-10-16 161830" style="zoom: 50%;" />