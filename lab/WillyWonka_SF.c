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
PWM_t opc;

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
		
		if(ESTOP_flag == 0){
			
			delay_ms(500);
			
			// LED state update in stirrer
			Stirrer_LED_state(LED_stir_state);
			
			// Algorithm to complement error values 
			Stir_dist_flag = Pre_process(distance);		
			
			//LED state determination
			LED_stir_state = Stirrer_state(Stir_dist_flag, Stir_pre_flag);	
	
  		//printf("STIR_DIST_FLAG : %d\r\n",Stir_dist_flag);
			
			Stir_pre_flag = LED_stir_state;
			
			// FOR OPENING AND CLOSING : debouncing 문제가 심해서 넣은 알고리즘
			OPC_flag = BUTTON_stir_det(BUTTON_cnt,BUTTON_prev, OPC_flag);
			
			BUTTON_prev = BUTTON_cnt;
			
			servo_opc_int = STIR_OPC_control(OPC_flag);
			
			PWM_duty(&opc,servo_opc_int);
		  
			// TEMPERATURE SENSOR			
			TEMP_det(TEMP_out);
			
			
			
		}
		
		else if(ESTOP_flag){
			
			TIM_INT_disable(TIM5);
			
			LED_estop();

			delay_ms(500);
			
		}

//    FOR DEBUGGING --------------------------------------------------------
//		printf("Distance : %f \r\n",distance);
//		printf("OPC flag : %d \r\n",OPC_flag);
//		printf("STIR_PRE_FLAG : %d \r\n\n\n",Stir_pre_flag);
//		printf("EMERGENCY_flag FLAG: %d",EMERGENCY_flag);
		


		
		
		
	}
	
}







//--------------------------------------------------------------------------------
/*  --------------------------------------------------
|		- SERVO MOTOR_1 TIMER INTERRUPT : TIM5						|	
|		- SERVO MOTOR_1 PWM : TIMER 4 CHANNEL 1						|
|		- SERVO MOTOR_2 PWM : TIMER 1 CHANNEL 1						|
|		- ULTRASONIC PWM : TIM3 CHANNEL 1					 		  	|
|		- ULTRASONIC TIMER INTERRUPT  : TIM2 CHANNEL 3  	|
|----------------------------------------------------*/


void TIM5_IRQHandler(void){
	
	if((TIM5->SR & TIM_SR_UIF) == TIM_SR_UIF){    

		if(servo_dir == 1) servo_idx ++;
		else if(servo_dir == -1) servo_idx --;
		
		servo_dir = update_dir(servo_dir,servo_idx);
	  servo_stir_duty = update_duty(servo_idx);
		PWM_duty(&pwm,servo_stir_duty);
	}

	TIM5->SR &= ~TIM_SR_UIF;                  		
}


void SERVO_setup(){

	TIM_INT_init(TIM5,SERVO_tim_period);
	
	PWM_init(&pwm,GPIOB,6);
	PWM_period_ms(&pwm,20);
	PWM_duty(&pwm,servo_stir_duty);
	
	PWM_init(&opc,GPIOA,8);
	PWM_period_ms(&opc,20);
	PWM_duty(&opc,servo_opc_duty);
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


void EXTI15_10_IRQHandler(void) { 
	
	if (is_pending_EXTI(BUTTON_PIN)) {
		
		BUTTON_cnt ++;
		
		clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
	}
	
}

// TEMPERRATUR SENSOR
void ADC_IRQHandler(void){
   if((is_ADC_OVR())){
      clear_ADC_OVR();
   }
   
   if( is_ADC_EOC() ){       	
     TEMP_out = ADC_read();
      
      
	}
}


// PC <---> MCU communication
void USART2_IRQHandler(){     
	
	if(is_USART_RXNE(USART2)){
		
		pcData = USART_getc(USART2);
		USART_write(USART1, &pcData, 1);
		printf("Nucleo got : %c \r\rn", pcData);
		
		printf("%c", pcData);
		
		if (pcData == END_CHAR)
			printf("\r\n");
		
	}
}


// bluetooth 통신
void USART6_IRQHandler(){         //USART6 INT 
	
	if(is_USART_RXNE(USART6)){
		
		mcuData = USART_getc(USART6);
		
//		USART_write(USART1,(uint8_t*) "BT sent : ", 10);
//		USART_write(USART1, &mcuData, 1);
//		USART_write(USART1, "\r\n", 2);
//		printf("NUCLEO got : %c (from BT)\r\n",mcuData);
		
		switch(mcuData){
			
			case 's' : ESTOP_flag = 1;													break;
			
			case 'r' : ESTOP_flag = 0;	TIM_INT_enable(TIM5);		break;
			
			default : break;
 
		}
		
	}
}

