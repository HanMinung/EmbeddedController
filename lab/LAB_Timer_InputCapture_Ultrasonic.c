/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : HanMinung
Created          : 05-03-2021
Modified         : 10-29-2022
Language/ver     : C++ in Keil uVision
/----------------------------------------------------------------*/

#include "ecUART.h"
#include "ecTIM.h"
#include "ecGPIO.h"
#include "ecSysTick.h"
#include "ecRCC.h"
#include "ecPWM.h"

uint32_t ovf_cnt = 0;
float distance = 0;
float timeInterval = 0;
uint32_t time1 = 0;
uint32_t time2 = 0;
float PulseWidth = 10;

void setup(void);
void TIM2_IRQHandler(void);

int main(void){

	setup();
	
	while(1){
		
		if(distance > 0.2 && distance < 40){
			
			printf("%f [cm]	\r\n", distance);
		
			delay_ms(500);
		}
		
		else{
		
			printf("garbage value : no recognition \r\n");

			delay_ms(1000);
		}
	}

}


void TIM2_IRQHandler(){

	if(is_UIF(TIM2)){
	
		ovf_cnt ++;
		clear_UIF(TIM2);
	}
	
	if(is_CCIF(TIM2, 3)){											//	TIM2_CH3 (IC3) Capture flag. Rising edge detect
	
		time1 = TIM2->CCR3;											// Capture time start
		clear_CCIF(TIM2,3);											// Clear capture/compare interrupt flag
		
	}
	
	else if(is_CCIF(TIM2,4)){
	
		time2 = TIM2->CCR4;											// Capture time end
		
		timeInterval = PulseWidth*((ovf_cnt*(TIM2->ARR+1)) + (time2 - time1));		// Total time of echo pulse 
		distance = (float) timeInterval/58.0;		// distance --> cm
		
		ovf_cnt = 0;														// overflow reset
		clear_CCIF(TIM2,4);
		
	}


}

void setup(){

	RCC_PLL_init();
	SysTick_init(1);
	UART2_init();
	
	// PWM configuration -----------------------------------------------------------------
	PWM_t trig;
	PWM_init(&trig, GPIOA, LED_PA6,FAST_SPEED, PUSH_PULL,NO_PUPD);
	PWM_period_us(&trig, 50000);
	PWM_pulsewidth_us(&trig, 10);
	
	// Input capture configuration ---------	----------------------------------------------
	IC_t echo;
	ICAP_init(&echo, GPIOB, LED_PB10);
	ICAP_counter_us(&echo,10);
	ICAP_setup(&echo, 3, IC_RISE);
	ICAP_setup(&echo, 4, IC_FALL);
	
	
	TIM_INT_enable(TIM2);
}
