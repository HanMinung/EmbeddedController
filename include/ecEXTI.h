/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : HanMinung
Created          : 05-03-2021
Modified         : 10-29-2022
Language/ver     : C++ in Keil uVision
/----------------------------------------------------------------*/

#ifndef __EC_EXTI_H
#define __EC_EXTI_H

#include "stm32f411xe.h"

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
	 
void EXTI_button(uint8_t trig_type, uint8_t push_pull,uint8_t priority);

#ifdef __cplusplus
}
#endif /* __cplusplus */
	 
#endif