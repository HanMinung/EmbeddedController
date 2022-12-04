/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : HanMinung
Created          : 2022.11.27
Modified         : 2022.11.28
Language/ver     : C++ in Keil uVision
Description 		 : Final project in Embedded Controller
/----------------------------------------------------------------*/

#include "ecHAL.h"
#include "FINAL_variable.h"

// VARIABLES : RC SERVO MOTOR --------------
PWM_t pwm;

// FUNCTION DECLARATION --------------------
void SERVO_setup();
void ULTRASONIC_setup();

// MAIN STATEMENT --------------------------
int main(void){
	
	// setup ---------------------------------
	FINAL_setup();
	SERVO_setup();
	ULTRASONIC_setup();


	// infinite loop -------------------------
	while(1){
		
		
		// LED state update in stirrer
		Stirrer_LED_state(LED_stir_state);
		
		// Algorithm to complement error values 
		Stir_dist_flag = Pre_process(distance);
		
		// FOR DEBUGGING -------------------------------------
//		printf("distance flag : %d\r\n\n\n",Stir_dist_flag);
//		printf("distance : %f\r\n",distance);		
		
		//LED state determination
		LED_stir_state = Stirrer_state(Stir_dist_flag, Stir_pre_flag);
		
		sevensegment_decoder(LED_stir_state);
		// FOR preventing error : Stir_pre_flag ( 0 OR 1 )
		Stir_pre_flag = LED_stir_state;

		
		// TIMER interrupt for servo motor : 0.5 sec
		
		delay_ms(500);
		
	}
	
}







//--------------------------------------------------------------------------------



/*  -------------------------------------------------
|		- SERVO MOTOR TIMER INTERRUPT : TIM5						|	
|		- SERVO MOTOR PWM : TIMER 4 CHANNEL 1						|
|		- ULTRASONIC PWM : TIM3 CHANNEL 1					 			|
|		- ULTRASONIC TIMER INTERRUPT  : TIM2 CHANNEL 3	|
|---------------------------------------------------*/


void TIM5_IRQHandler(void){
	
	if((TIM5->SR & TIM_SR_UIF) == TIM_SR_UIF){    

		if(servo_dir == 1) servo_idx ++;
		else if(servo_dir == -1) servo_idx --;
		
		servo_dir = update_dir(servo_dir,servo_idx);
	  servo_duty = update_duty(servo_idx);
		PWM_duty(&pwm,servo_duty);
	}

	TIM5->SR &= ~TIM_SR_UIF;                  		
}


void SERVO_setup(){

	TIM_INT_init(TIM5,SERVO_tim_period);
	
	PWM_init(&pwm,GPIOB,6);
	PWM_period_ms(&pwm,20);
	PWM_duty(&pwm,servo_duty);
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
		clear_CCIF(TIM2,4);
		
		timeInterval = PulseWidth*((ovf_cnt*(TIM2->ARR+1)) + (time2 - time1));		// Total time of echo pulse 
		distance = (float) timeInterval/58.0;		// distance --> cm
		
		ovf_cnt = 0;														// overflow reset
		
	}
}


void ULTRASONIC_setup(){

	UART2_init();
	PWM_t trig;
	PWM_init(&trig, GPIOA, 6);
	PWM_period_ms(&trig, 50);
	PWM_pulsewidth_us(&trig, 10);
	
	// Input capture configuration
	IC_t echo;
	ICAP_init(&echo, GPIOB, 10);
	ICAP_counter_us(&echo,10);
	ICAP_setup(&echo, 3, IC_RISE);
	ICAP_setup(&echo, 4, IC_FALL);
	
}


// BLUETOOTH COMMUNICATION
void USART1_IRQHandler(){  
	
	if(is_USART_RXNE(USART1)){
		
		mcuData = USART_getc(USART1);
		
//		USART_write(USART1,(uint8_t*) "BT sent : ", 10);
//		USART_write(USART1, &mcuData, 1);
//		USART_write(USART1, "\r\n", 2);
//		printf("NUCLEO got : %c (from BT)\r\n",mcuData);
		
		switch(mcuData){
			
			// STOP FLAG
			case 's' :  ALLSTOP_flag = 1;		break;
			// ACTIVATION FLAG
			case 'r' :  ALLSTOP_flag = 0;		break;
			
		}
		
	}
}


// PC <---> MCU 통신
void USART2_IRQHandler(){     
	
	if(is_USART_RXNE(USART2)){
		
		pcData = USART_getc(USART2);
		USART_write(USART1, &pcData, 1);
//		printf("Nucleo got : %c \r\rn", pcData);
		
		printf("%c", pcData);
		
		if (pcData == END_CHAR)
			printf("\r\n");
		
	}
}