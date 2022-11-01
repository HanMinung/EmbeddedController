#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecSysTick.h"
			
#ifndef __EC_STEPPER_H
#define __EC_STEPPER_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

//State mode
#define HALF 0
#define FULL 1	 

#define CW 0
#define CCW 1

#define GEAR_RATIO 32
#define FULL_ANGLE 64
#define HALF_ANGLE 128

/* Stepper Motor */
//stepper motor function

typedef struct{
   GPIO_TypeDef *port1;
   int pin1;
	 GPIO_TypeDef *port2;
   int pin2;
	 GPIO_TypeDef *port3;
   int pin3;
	 GPIO_TypeDef *port4;
   int pin4;
	 int _step_num;
	
} Stepper_t;

	 
void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4);

void Stepper_setSpeed(long whatSpeed,int mode);

void Stepper_pinOut (uint32_t state, int mode);

void Stepper_step(int steps, int dir, int mode, long rpm);

void Stepper_stop(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
