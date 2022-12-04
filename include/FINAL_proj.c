#include "ecHAL.h"

// -------------------- Variable definition ----------------------


// -------------------- Structure definition ----------------------

unsigned int LED_stir_struct[3][3] = {
	
	// color : green yellow red
	// PA8 , PB10 , PB4
	{1,0,0},		//	stable state flag
	{0,1,0},		//	motion detected flag
	{0,0,1}			//	all stop flag
	
};

unsigned int LED_dist_struct[3][3] = {
	
	// color : green yellow red
	// PB5 , PB3 , PA10
	{1,0,0},
	{0,1,0},
	{0,0,1}
	
};

// ---------------------		SETUP PROCESS			--------------------------


void FINAL_setup(void){
	
	// Servo motor setup -------------------------------------------
	RCC_PLL_init();
	SysTick_init(1);
	
	// LED setup ---------------------------------------------------
	LED_stir_setup();
	LED_dist_setup();
	
	// SEVENSEGMENT INIT -------------------------------------------
	sevensegment_init();
	
	// Temperature sensor setup ------------------------------------
	
	// BLUETOOTH communication setup : PA9 (TX) , PA10 (RX)
	USART_begin(USART1, GPIOA, 9, GPIOA, 10, 9600); 

	
}



void LED_stir_setup(){
	// STIRRER - STATE LED
	// PA8 , PB10 , PB4
	indiv_init(GPIOB,0,OUTPUT,PUSH_PULL,PULL_UP,MEDIUM_SPEED);
	indiv_init(GPIOC,1,OUTPUT,PUSH_PULL,PULL_UP,MEDIUM_SPEED);
	indiv_init(GPIOC,0,OUTPUT,PUSH_PULL,PULL_UP,MEDIUM_SPEED);

}


void LED_dist_setup(){
	// DISTANCE - STATE LED
	// PB5 , PB3 , PA10
	indiv_init(GPIOB,5,OUTPUT,PUSH_PULL,NO_PUPD,MEDIUM_SPEED);
	indiv_init(GPIOB,3,OUTPUT,PUSH_PULL,NO_PUPD,MEDIUM_SPEED);
	indiv_init(GPIOA,10,OUTPUT,PUSH_PULL,NO_PUPD,MEDIUM_SPEED);
	
}


// --------------------		FUCTION DEFINITION	 --------------------------
void Stirrer_LED_state(uint16_t LED_stir_state){

	GPIO_write(GPIOB, 0, LED_stir_struct[LED_stir_state][0]);				// LED : GREEN
	GPIO_write(GPIOC, 1, LED_stir_struct[LED_stir_state][1]);				// LED : WHITE
	GPIO_write(GPIOC, 0, LED_stir_struct[LED_stir_state][2]);				// LED : RED
	
}


uint8_t Pre_process(float dist){

	uint8_t _flag = 0;
	
	if(dist > 12.0 && dist < 17.0)		_flag = 0;		// PROPER value : closed
	else if (dist > 20 && dist < 40) 	_flag = 1;		// OPEN state
	else if(dist < 2 || dist > 400)		_flag = 2;		// ERROR value : exception
	
	return _flag;

}


uint16_t Stirrer_state(uint8_t _flag, uint8_t _preflag){

	// ALGORITHM
	
	switch(_flag){
	
		case 0 : TIM_INT_enable(TIM5);			break;

		case 1 : TIM_INT_disable(TIM5);			break;
			
		case 2 : delay_ms(100);							break;
			
		default : break;
	
	}
	
	if(_flag != 2)	return _flag;
	else 						return _preflag;
	
}




