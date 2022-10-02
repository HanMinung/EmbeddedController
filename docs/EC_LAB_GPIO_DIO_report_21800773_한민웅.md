# LAB: GPIO Digital InOut

**Date:** 2022-09-26

**Author/Partner:** Han,Minung / Nam,Hyunwoo

**Github:** https://github.com/HanMinung/EC-muhan-773

**Demo Video:** https://youtu.be/UcWN2vmCtk8

# Introduction

​		In this lab, we are required to create a simple program that toggle multiple LEDs with a push-button input. Create HAL drivers for GPIO digital in and out control and use your library. In priority, a function that toggles a state of a specific GPIO pin need to be defined. The function operates in the form of approaching a specific pin and toggling through xor logic. Using the above pre-defined function, multi LEDs can be controlled in a concise form using STM32. Finally, it aims to operate the four LEDs in the desired opeation through configuration for multiple GPIO pins.

## Requirement

### Hardware

- MCU
  - NUCLEO-F401RE
- Actuator/Sensor/Others:
  - LEDs x 4
  - Resistor 330 ohm x 3, breadboard

### Software

- Keil uVision, CMSIS, EC_HAL library



# Problem 1: Create EC_HAL library

## Procedure

​	We need to define below functions to set efficiently depending on the situation.

* GPIO.h   | GPIO.c

  Function declaration

  ```c
  /* GPIO.h */
  void GPIO_init(GPIO_TypeDef *Port, int pin, int mode);
  void GPIO_write(GPIO_TypeDef *Port, int pin, int Output);	
  int  GPIO_read(GPIO_TypeDef *Port, int pin);
  void GPIO_mode(GPIO_TypeDef* Port, int pin, int mode);
  void GPIO_ospeed(GPIO_TypeDef* Port, int pin, int speed);
  void GPIO_otype(GPIO_TypeDef* Port, int pin, int type);
  void GPIO_pupd(GPIO_TypeDef* Port, int pin, int pupd);
  ```
  
  ```c
  /* GPIO.c */
  void GPIO_init(GPIO_TypeDef *Port, int pin, int mode){     
  	// mode  : Input(0), Output(1), AlterFunc(2), Analog(3)   
  	if (Port == GPIOA)		RCC_GPIOA_enable();
  	if (Port == GPIOB)		RCC_GPIOB_enable();
  	if (Port == GPIOC)		RCC_GPIOC_enable();
  	if (Port == GPIOD)		RCC_GPIOD_enable();
  	if (Port == GPIOE)		RCC_GPIOE_enable();
  	if (Port == GPIOH)		RCC_GPIOH_enable();
      
  	GPIO_mode(Port, pin, mode);
  }
  
  
  // GPOI WRITE				: Ouput(LOW), Output(HIGH)
  void GPIO_write(GPIO_TypeDef *Port, int pin, int Output){
  	Port->ODR &= ~(1UL<<pin);
  	Port->ODR |= Output<<pin;
  }
  
  // GPIO Mode       		   : Input(00), Output(01), AlterFunc(10), Analog(11)
  void GPIO_mode(GPIO_TypeDef *Port, int pin, int mode){
    	Port->MODER &= ~(3UL<<(2*pin));    		// Initialization
    	Port->MODER |= mode<<(2*pin);    			
  }
  
     
  // GPIO Speed       : Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
  void GPIO_ospeed(GPIO_TypeDef *Port, int pin, int speed){
  	Port->OSPEEDR &= ~(3UL<<(2*pin));				// Initialization
  	Port->OSPEEDR |= speed<<(2*pin);
  }
  
  // GPIO Output Type: Output push-pull (0, reset), Output open drain (1)
  void GPIO_otype(GPIO_TypeDef *Port, int pin, int type){
    	Port->OTYPER |= (type<< pin);
  }
  
  // GPIO Push-Pull    : No pull-up (00), Pull-up (01), Pull-down (10), Reserved (11)
  void GPIO_pupd(GPIO_TypeDef *Port, int pin, int pupd){
  	Port->PUPDR &= ~(3UL<<(2*pin));
  	Port->PUPDR |= pupd<<(2*pin);
  }
  
  
  int GPIO_read(GPIO_TypeDef *Port, int pin){
  	// return the value of present state
  	int bitVal = ((Port->IDR)>>pin)&1; 
  	
  	return bitVal;
  }
  ```
  
  
  

# Problem 2: Toggle LED with Button

## Procedure

- First, we need to define a function that toggles state of a specific pin.
- When the button is pressed, the function operates and the status of the LED used as hardware changes.
- At the beginning of the code, it is most important to set up according to the required configurations.

## Configuration

|    Button (B1)     |                LED                |
| :----------------: | :-------------------------------: |
| Digital in (Input) |       Digital out (Output)        |
|   GPIOC, pin 13    |           GPIOA, pin 5            |
|      PULL-UP       | OPEN DRAIN, PULL-UP, MEDIUM SPEED |

code : **code 첨부**!!!

```c
/* LED_GPIO_DIO_LED.c */

#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"
#include "ecSysTick.h"

#define LED_PIN 	5
#define BUTTON_PIN 13

void setup(void);
	
int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		
		if(GPIO_read(GPIOC,BUTTON_PIN) == 0){		// Button pressed
			
			delay_ms(30);							// Need delay
			bittoggle(GPIOA,LED_PIN);				// Toggle a specific pin (GPIOA -> LED_PIN)
		}
	}
}
	
// Initialiization 
void setup(void){
	
	RCC_HSI_init();
	SysTick_init();
	// led pin configuration
	GPIO_init(GPIOA, LED_PIN, OUTPUT);  		  // calls RCC_GPIOA_enable()
	GPIO_pupd(GPIOA, LED_PIN, EC_PU);			  // pull up
	GPIO_otype(GPIOA, LED_PIN, OPEN_DRAIN);		  // open drain
	GPIO_write(GPIOA, LED_PIN, LOW);			
	GPIO_ospeed(GPIOA, LED_PIN, MEDIUM_SPEED);	  // set output speed : Medium speed
	// button pin configuration
	GPIO_init(GPIOC, BUTTON_PIN, INPUT); 		  // Calls RCC_GPIOC_enable()
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);	      // Pull up
	GPIO_otype(GPIOC, BUTTON_PIN, 1);

}
```

```c
/* GPIO.c */
void bittoggle(GPIO_TypeDef *Port, int pin){
	
	Port->ODR ^= (1<<pin);						  // Toggle current state with XOR logic
}
```

## Discussion

1. If button is pressed (LOW to HIGH or HIGH to LOW),  it is short but takes time for the LED to reach a stable state. For that short period of time, oscillation occurs around the state to be reached. For that reason, in the floating state, unknown behavior of hardware (LED) occurs, and a solution about that is required.  The solution is software delay. After button is pressed, a change due to oscillation is prevented through a delay. The function 'delay_ms' was used in the code with prerequisite files like (ecSysTick.c, ecSysTick.h).
2. Hardware debouncing : In order to solve the switch bouncing phenomenon, hardware connects the capacitor in parallel with the switch. Then, the capacitor prevents the bouncing phenomenon absorbing a rapid voltage change caused by bouncing. Capacitors charge when current flows and discharge when polarity changes. When the switch is closed, the capacitor delays the voltage drop while discharging. In conclusion, it is to create an effect of filtering a waveform vibrating in a specific interval generated by bouncing. In addition, there is also a method of debouncing by configuring a logic circuit using SR latch.
3. In addition, we need to understand the difference of the output type (push pull, open drain). First, push-pull refers to determining the output port using the internal power of the MCU and IC. In contrast, Open-drain uses an external terminal. The reason for using an external power source is that if the MCU has a power supply of 3.3V and an external device of 5V, the voltage difference may occur some problems when connected directly. Therfore, it is used to match the voltage  with the external device connected to the outside.



# Problem 3: Toggle LED with Button

## Problem

* First state : All LEDs are LOW state (off)

* When button is pressed once, first LED (LED 0) toggles its state. 

* As the button is pressed once, each LED is lit by one sequentially, and the previous one is toggled. 

  ( LED 0 --> LED 1 --> LED 2 --> LED 3 --> LED 0  ... repetition )

## Configuration

|    Button (B1)     |               LED                |
| :----------------: | :------------------------------: |
| Digital in (Input) |       Digital out (Output)       |
|   GPIOC, pin 13    |        PA5, PA6, PA7, PB6        |
|      PULL-UP       | PUSH-PULL, PULL-UP, MEDIUM-SPEED |

## Circuit Diagram

**Circuit diagram (image)**

![multiLED_2](https://user-images.githubusercontent.com/99113269/193262987-256bc475-c102-427a-881f-48f2f02275d6.png)





## Code

Code  : [https://github.com/HanMinung/EC-muhan-773/blob/main/lab/LAB_GPIO_DIO_multiLED.c](https://github.com/ykkimhgu/EC-student/)

​	This code targets four LEDs to be toggled and turned on one by one.

```c
/* LAB_GPIO_DIO_multiLED.c */
#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"
#include "ecSysTick.h"

#define LED_PA5 	5			// PA5
#define LED_PA6 	6			// PA6
#define LED_PA7 	7			// PA7
#define LED_PB6 	6			// PB6
#define BUTTON_PIN 13			// Button pin : PC13

void setup(void);
	
int main(void) { 
	
	// Initialiization --------------------------------------------------------
	setup();
	int flag = 0;
	int cnt = 0;
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		if(GPIO_read(GPIOC,BUTTON_PIN) == 0)	{
			cnt ++;
			flag = cnt%4;
		
			switch(flag){
				
				case 1 :
					bittoggle(GPIOA,LED_PA5);
					if(cnt > 4){
						bittoggle(GPIOB,LED_PB6);
					}
					break;
				
				case 2 :
					bittoggle(GPIOA,LED_PA5);
					bittoggle(GPIOA,LED_PA6);
					break;
				
				case 3 :
					bittoggle(GPIOA,LED_PA6);
					bittoggle(GPIOA,LED_PA7);
					break;
				
				default : 
					bittoggle(GPIOA,LED_PA7);	
					bittoggle(GPIOB,LED_PB6);
            }
		} 
			delay_ms(60);
	}
	
}


// Initialiization 
void setup(void)
{
	
	RCC_HSI_init();	
	SysTick_init();
	
	GPIO_init(GPIOC, BUTTON_PIN, INPUT); 		 // calls RCC_GPIOC_enable()
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU); 		 // set button pin pull up
	
	GPIO_init(GPIOA, LED_PA5, OUTPUT);    		 // calls RCC_GPIOA_enable()
	GPIO_pupd(GPIOA, LED_PA5, EC_PU);  	  		 // set LED PA5 pin pull up
	GPIO_otype(GPIOA, LED_PA5, 0);		  		 // set output type : push-pull
	GPIO_write(GPIOA, LED_PA5, LOW);	  		 // set output value to pin PA5
	GPIO_ospeed(GPIOA , LED_PA5, MEDIUM_SPEED);  // set output speed : medium
	
	GPIO_init(GPIOA, LED_PA6, OUTPUT);   		 // calls RCC_GPIOA_enable()
	GPIO_pupd(GPIOA, LED_PA6, EC_PU);
	GPIO_otype(GPIOA, LED_PA6, 0);
	GPIO_write(GPIOA, LED_PA6, LOW);
	GPIO_ospeed(GPIOA , LED_PA6, MEDIUM_SPEED);

	GPIO_init(GPIOA, LED_PA7, OUTPUT);  		  // calls RCC_GPIOA_enable()
	GPIO_pupd(GPIOA, LED_PA7, EC_PU);
	GPIO_otype(GPIOA, LED_PA7, 0);
	GPIO_write(GPIOA, LED_PA7, LOW);
	GPIO_ospeed(GPIOA , LED_PA7, MEDIUM_SPEED);

	GPIO_init(GPIOB, LED_PB6, OUTPUT);   		  // calls RCC_GPIOA_enable()
	GPIO_pupd(GPIOB, LED_PB6, EC_PU);
	GPIO_otype(GPIOB, LED_PB6, 0);
	GPIO_write(GPIOB, LED_PB6, LOW);
	GPIO_ospeed(GPIOA , LED_PB6, MEDIUM_SPEED);
					  
}
```

## Results

Experimental result :

[![IU(아이유) _ Into the I-LAND](http://img.youtube.com/vi/UcWN2vmCtk8/0.jpg)](https://www.youtube.com/watch?v=UcWN2vmCtk8) 

Demo video link : https://youtu.be/UcWN2vmCtk8

## Discussion

​	Above all, it is important to define and use a toggle function using XOR logic. Based on the function, it was possible to control the multi LEDs with a relatively  simple code. In addition, it was necessary to use the delay function defined in ecSysTick.c as a software method for debouncing the switch. In other words, it is important to allow hardware such as LEDs to operate normally by delaying the appropriate time after pressing the switch.

 

# Reference

* Github : https://github.com/ykkimhgu/EC-student