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

// VARIABLE DECLARATION --------------------------------------
PWM_t pwm;

// FUNCTION DECLARATION --------------------------------------
void FINAL_setup(void);
void MULTLED_init(void);
void PIEZO_init(void);
void SECTION_det(void);
void PIEZO_det(uint8_t _flag);

int main(void){
	
	FINAL_setup();	
	
	while(1){
		
		if(ESTOP_flag == 0){
			
			SECTION_det();
			
			PIEZO_det(SEC_flag);
			
			PWM_duty(&pwm,BLIND_duty);
			
		}
		
		else if(ESTOP_flag){
		
			GPIO_write(GPIOC,8,HIGH);
			GPIO_write(GPIOC,6,HIGH);
			GPIO_write(GPIOC,5,HIGH);
		}
		
	}
}


// ------------------- SETUP --------------------------------

void FINAL_setup(void){

	RCC_PLL_init();
	USART_init(USART2,9600);
         
	USART_begin(USART1, GPIOB, 6, GPIOB, 3, 9600);    // WIRED COMMUNICATION
  USART_begin(USART6, GPIOA, 11, GPIOA, 12, 9600);	// ZIGBEE communication
	
	MULTLED_init();
	PIEZO_init();
		
}


void MULTLED_init(void){

	// FIRST LED
	indiv_init(GPIOC,8,OUTPUT,PUSH_PULL,NO_PUPD,MEDIUM_SPEED);
	// SECOND LED
	indiv_init(GPIOC,6,OUTPUT,PUSH_PULL,NO_PUPD,MEDIUM_SPEED);
	// THIRD LED
	indiv_init(GPIOC,5,OUTPUT,PUSH_PULL,NO_PUPD,MEDIUM_SPEED);
	indiv_init(GPIOA,0,AF,PUSH_PULL,NO_PUPD,MEDIUM_SPEED);
	
	// PWM INITIATION
	PWM_init(&pwm,GPIOA,0);
	PWM_period_ms(&pwm,200); 
	
	// INITIALIZATION
	GPIO_write(GPIOC,8,LOW);
	GPIO_write(GPIOC,6,LOW);
	GPIO_write(GPIOC,5,LOW);
	
}


void PIEZO_init(void){
	
	// FIRST PIEZO
	GPIO_init(GPIOC,9,INPUT);	GPIO_pupd(GPIOC, 9,PULL_DOWN);
	// SECOND PIEZO
	GPIO_init(GPIOB,8,INPUT);	GPIO_pupd(GPIOB, 8,PULL_DOWN);
	// THIRD PIEZO
	GPIO_init(GPIOB,9,INPUT);	GPIO_pupd(GPIOB, 9,PULL_DOWN);
}

void SECTION_det(void){

	uint8_t PIEZO_1 = GPIO_read(GPIOC,9);
	uint8_t PIEZO_2 = GPIO_read(GPIOB,8);
	uint8_t PIEZO_3 = GPIO_read(GPIOB,9);
	
	if(PIEZO_1 == HIGH)					SEC_flag = flag_sec1;
	else if(PIEZO_2 == HIGH)		SEC_flag = flag_sec2;
	else if(PIEZO_3 == HIGH)		SEC_flag = flag_sec3;
	
}

void PIEZO_det(uint8_t _flag){

	switch(_flag){
	
		case 0 : {
			GPIO_write(GPIOC,8,LOW);
			GPIO_write(GPIOC,6,LOW);
			GPIO_write(GPIOC,5,LOW);
			BLIND_duty = 0.0;
			
		} break;
			
		case 1 : {
			GPIO_write(GPIOC,8,HIGH);
			GPIO_write(GPIOC,6,LOW);
			GPIO_write(GPIOC,5,LOW);	
			BLIND_duty = 0.10;
						
		} break;
		
		case 2 :{
			GPIO_write(GPIOC,8,LOW);
			GPIO_write(GPIOC,6,HIGH);
			GPIO_write(GPIOC,5,LOW);			
			BLIND_duty = 0.50;
			
		} break;
			
		case 3 :{
			GPIO_write(GPIOC,8,LOW);
			GPIO_write(GPIOC,6,LOW);
			GPIO_write(GPIOC,5,HIGH);	
			BLIND_duty = 0.80;
			
		} break;
		
		defualt : break;
	
	}
	
}

// WIRED COMMUNICATION
void USART1_IRQHandler(){         //USART1 INT 
	
   if(is_USART_RXNE(USART1)){
      mcu2Data = USART_getc(USART1);
      USART_write(USART6, &mcu2Data, 1);
      
		  if(mcu2Data == 's') 			ESTOP_flag = 1;
		 	else if(mcu2Data == 'r') 	ESTOP_flag = 0;
      
      printf("received: %c\r\n", mcu2Data);
   }
}


