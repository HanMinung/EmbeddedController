/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : HanMinung
Created          : 05-03-2021
Modified         : 10-29-2022
Language/ver     : C++ in Keil uVision
/----------------------------------------------------------------*/

#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecEXTI.h"
#include "ecTIM.h"
#include "ecPWM.h"
#include "ecSysTick.h"


void EXTI15_10_IRQHandler(void);
void setup(void);

PWM_t pwm;
uint8_t idx = 0;
int dir = 1;
float duty = 0.5/20.0;

int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){ }

} 


// Initialiization 
void setup(void)
{
	
	TIM_INT_init(TIM3,500);
	RCC_PLL_init();
	SysTick_init();
	
	EXTI_button(FALL,EC_PU,0);						 // EXTI button configuration
	
	PWM_init(&pwm,GPIOA,PA1,HIGH_SPEED,PUSH_PULL,PULL_UP);
	PWM_period_ms(&pwm,20);
	PWM_duty(&pwm,duty);
	
}


void EXTI15_10_IRQHandler(void) { 
	
	if (is_pending_EXTI(BUTTON_PIN)) {
		
		PWM_duty(&pwm,0.5/20.0);
		idx = 0; dir = 1;
		
		clear_pending_EXTI(BUTTON_PIN); 			// cleared by writing '1'
	}
	
	
		
}	


void TIM3_IRQHandler(void){
	if((TIM3->SR & TIM_SR_UIF) == TIM_SR_UIF){    // update interrupt flag

		if(dir == 1) idx ++;
		else if(dir == -1) idx --;
		
		dir = update_dir(dir,idx);
	  duty = update_duty(idx);
		PWM_duty(&pwm,duty);
		
	}
	// Check the flag in status register
	TIM3->SR &= ~TIM_SR_UIF;                  		// clear by writing 0
	
}
