#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecEXTI.h"
#include "ecSysTick.h"
#include "ecStepper.h"

uint32_t flag = 0;
void setup(void);
void EXTI15_10_IRQHandler(void);

int main(void){
	//----------------------  setup  ----------------------------
	setup();
	
	//------------------- infinite loop -------------------------
	while(1){

		if(flag == HIGH)		Stepper_stop();
		else 		 						Stepper_step(GEAR_RATIO*FULL_ANGLE, CCW, HALF, 2);		
	
	}
}

void setup(void){

	RCC_PLL_init();
	SysTick_init(1);
	
	GPIO_init(GPIOC,BUTTON_PIN,INPUT);
	EXTI_init(GPIOC,BUTTON_PIN,FALL,0);
	
	Stepper_init(GPIOB,LED_PB10,GPIOB,LED_PB4,GPIOB,LED_PB5,GPIOB,LED_PB3);
	
}


void EXTI15_10_IRQHandler(void){

	if(is_pending_EXTI(BUTTON_PIN)){
		
		flag ^= 1;
		Stepper_stop();
		// clear by writing '1'
		clear_pending_EXTI(BUTTON_PIN);			
	}
	
}