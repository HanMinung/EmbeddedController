/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : HanMinung
Created          : 05-03-2021
Modified         : 10-29-2022
Language/ver     : C++ in Keil uVision
/----------------------------------------------------------------*/
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecTIM.h" 

#ifndef __EC_PWM_H
#define __EC_PWM_H

#define PA1		1

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

typedef struct{
   GPIO_TypeDef *port;
   int pin;
   TIM_TypeDef *timer;
   int ch;
} PWM_t;


/* PWM Configuration */
void PWM_init(PWM_t *pwm, GPIO_TypeDef *port, int pin);
void PWM_period_ms(PWM_t *pwm,  uint32_t msec);		
void PWM_period_us(PWM_t *pwm, uint32_t usec);  


void PWM_pulsewidth_ms(PWM_t *pwm, float pulse_width_ms);
void PWM_pulsewidth_us(PWM_t *pwm, float pulse_width_us);
void PWM_duty(PWM_t *pwm, float duty);
void PWM_pinmap(PWM_t *PWM_pin);

int update_dir(int dir,uint8_t idx);
float update_duty(uint8_t idx);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
