/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : HanMinung
Created          : 05-03-2021
Modified         : 10-29-2022
Language/ver     : C++ in Keil uVision
/----------------------------------------------------------------*/

#ifndef __EC_TIM_H 
#define __EC_TIM_H
#include "stm32f411xe.h"

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */


/* Timer Configuration */
void TIM_init(TIM_TypeDef *timerx, uint32_t msec);  
void TIM_period_us(TIM_TypeDef* timx, uint32_t usec);  
void TIM_period_ms(TIM_TypeDef* timx, uint32_t msec);

void TIM_INT_init(TIM_TypeDef* timerx, uint32_t msec); 
void TIM_INT_enable(TIM_TypeDef* timx);
void TIM_INT_disable(TIM_TypeDef* timx);

uint32_t is_UIF(TIM_TypeDef *TIMx);
void clear_UIF(TIM_TypeDef *TIMx);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
