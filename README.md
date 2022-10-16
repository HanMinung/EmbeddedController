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



## ecEXTI.h

```c
#ifndef __EC_EXTI_H
#define __EC_EXTI_H

#include "stm32f411xe.h"

// trgger type : falling / rising / both
#define FALL 0
#define RISE 1
#define BOTH 2

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

void EXTI_init(GPIO_TypeDef *Port, int pin, int trig, int priority);
void EXTI_enable(uint32_t pin);
void EXTI_disable(uint32_t pin);
uint32_t is_pending_EXTI(uint32_t pin);
void clear_pending_EXTI(uint32_t pin);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
```

#### void EXTI_init(GPIO_TypeDef *Port, int pin, int trig, int priority)

* Enable peripheral clock
* Connect external lien to the GPIO
  * e.g. Button pin : GPIOC  
  * EXTICR_port = 2;  ( A : 0 , B : 1 , C : 2 , D : 3 )
* Clear 4 bits and set
* Configure Trigger edge , Interrupt mask , NVIC setting
* Set priority , Enable IRQ
* Parameter : Port , Pin , Trigger type , Priority
* Code structure :

```c
void EXTI_init(GPIO_TypeDef *Port, int Pin, int trig_type,int priority){

	// SYSCFG peripheral clock enable	
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;		
	
	// Connect External Line to the GPIO
	int EXTICR_port;
	if		(Port == GPIOA) EXTICR_port = 0;
	else if	(Port == GPIOB) EXTICR_port = 1;
	else if	(Port == GPIOC) EXTICR_port = 2;
	else if	(Port == GPIOD) EXTICR_port = 3;
	else 					EXTICR_port = 4;
	
	SYSCFG->EXTICR[Pin/4] &= ~15<<(4*(Pin%4));	 				// clear 4 bits
	SYSCFG->EXTICR[Pin/4] |= EXTICR_port<<(4*(Pin%4));			// set 4 bits
	
	// Configure Trigger edge
	if (trig_type == FALL) EXTI->FTSR |= 1<<Pin;   		// Falling trigger enable 
	else if	(trig_type == RISE) EXTI->RTSR |= 1<<Pin;   // Rising trigger enable 
	else if	(trig_type == BOTH) {						// Both falling/rising trigger enable
		EXTI->RTSR |= 1<<Pin; 
		EXTI->FTSR |= 1<<Pin;
	} 
	
	// Configure Interrupt Mask (Interrupt enabled)
	EXTI->IMR  |= 1<<Pin;    	 // not masked
	
	
	// NVIC(IRQ) Setting
	int EXTI_IRQn = 0;			// initialization

	if(Pin < 5) 		EXTI_IRQn = (Pin + 6);
	else if(Pin < 10) 	EXTI_IRQn = EXTI9_5_IRQn;
	else 				EXTI_IRQn = EXTI15_10_IRQn;
	
								
	NVIC_SetPriority(EXTI_IRQn, priority);	// EXTI priority
	NVIC_EnableIRQ(EXTI_IRQn); 				// EXTI IRQ enable
}
```

```c
void EXTI_enable(uint32_t pin) {
	EXTI->IMR |= 1<<pin;    			 // not masked (i.e., Interrupt enabled)
}

void EXTI_disable(uint32_t pin) {
	EXTI->IMR &= ~1<<pin;       		 // masked (i.e., Interrupt disabled)
}

uint32_t is_pending_EXTI(uint32_t pin){
	uint32_t EXTI_PRx = 1 << pin;     	// check  EXTI pending 	
	return ((EXTI->PR & EXTI_PRx) == EXTI_PRx);	
}

void clear_pending_EXTI(uint32_t pin){
	EXTI->PR |= 1<<pin;     // clear EXTI pending : essential point
	// This bit is cleared by programming it to '1'
}
```

* Structure of EXTI-->IMR : Interrupt mask regisgter

  <img src="https://user-images.githubusercontent.com/99113269/196026004-65cc208d-8cda-499e-afc6-c820d63507da.png" alt="image" style="zoom:67%;" />

  

* Structure of EXTI-->PR : Pending register

<img src="https://user-images.githubusercontent.com/99113269/196025934-b89eb2eb-91df-4ca4-9045-471c8b5884f4.png" alt="image" style="zoom: 67%;" />