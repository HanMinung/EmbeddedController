/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : HanMinung
Created          : 05-03-2021
Modified         : 11-02-2022
Language/ver     : C++ in Keil uVision
/----------------------------------------------------------------*/

#include "stm32f4xx.h"
#include "stm32f411xe.h"
#include "ecGPIO.h"

void GPIO_init(GPIO_TypeDef *Port, int pin, int mode){     
	// mode  : Input(0), Output(1), AlterFunc(2), Analog(3)   
	if (Port == GPIOA)
		RCC_GPIOA_enable();
	
	if (Port == GPIOB)
		RCC_GPIOB_enable();
	
	if (Port == GPIOC)
		RCC_GPIOC_enable();
	
	if (Port == GPIOD)
		RCC_GPIOD_enable();
	
	if (Port == GPIOE)
		RCC_GPIOE_enable();
	
	if (Port == GPIOH)
		RCC_GPIOH_enable();

	// You can also make a more general function of
	// void RCC_GPIO_enable(GPIO_TypeDef *Port); 

	GPIO_mode(Port, pin, mode);
	
}

// port, pin, type(input or output), output type(push pull or open drain) , pupd , output speed
void indiv_init(GPIO_TypeDef *Port,int pin,int TYPE,int OTYPE,int PUPD,int OSPEED){

	GPIO_init(Port, pin, TYPE);    // calls RCC_GPIOA_enable()	
	GPIO_otype(Port, pin, OTYPE);
	GPIO_pupd(Port, pin,PUPD);
	GPIO_ospeed(Port, pin, OSPEED);
	
}



// PB4 , PB5 , PB3 , PA10 sequencially
void LED_init(GPIO_TypeDef *Port1, int pin1, GPIO_TypeDef *Port2, int pin2, GPIO_TypeDef *Port3, int pin3, GPIO_TypeDef *Port4, int pin4){

	GPIO_init(Port1, pin1, OUTPUT);    // calls RCC_GPIOA_enable()	
	GPIO_otype(Port1, pin1, PUSH_PULL);
	GPIO_pupd(Port1, pin1,NO_PUPD);
	GPIO_ospeed(Port1, pin1, MEDIUM_SPEED);
	
	GPIO_init(Port2, pin2, OUTPUT);    // calls RCC_GPIOA_enable()	
	GPIO_otype(Port2, pin2, PUSH_PULL);
	GPIO_pupd(Port2, pin2,NO_PUPD);
	GPIO_ospeed(Port2, pin2, MEDIUM_SPEED);

	GPIO_init(Port3, pin3, OUTPUT);    // calls RCC_GPIOA_enable()	
	GPIO_otype(Port3, pin3, PUSH_PULL);
	GPIO_pupd(Port3, pin3,NO_PUPD);
	GPIO_ospeed(Port3, pin3, MEDIUM_SPEED);
	
	GPIO_init(Port4, pin4, OUTPUT);    // calls RCC_GPIOA_enable()	
	GPIO_otype(Port4, pin4, PUSH_PULL);
	GPIO_pupd(Port4, pin4,NO_PUPD);
	GPIO_ospeed(Port4, pin4, MEDIUM_SPEED);

}

// GPOI WRITE					: Ouput(LOW), Output(HIGH)
void GPIO_write(GPIO_TypeDef *Port, int pin, int Output){
	Port->ODR &= ~(1UL<<pin);
	Port->ODR |= Output<<pin;

}


// GPIO Mode          : Input(00), Output(01), AlterFunc(10), Analog(11)
void GPIO_mode(GPIO_TypeDef *Port, int pin, int mode){
  Port->MODER &= ~(3UL<<(2*pin));    		// Initialization
  Port->MODER |= mode<<(2*pin);    			
}

   
// GPIO Speed       : Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
void GPIO_ospeed(GPIO_TypeDef *Port, int pin, int speed){
	Port->OSPEEDR &= ~(3UL<<(2*pin));				// Initialization
	Port->OSPEEDR |= speed<<(2*pin);

}

// GPIO Output Type: Output push-pull (0, reset), Output open drain (1)
void GPIO_otype(GPIO_TypeDef *Port, int pin, int type){
  Port->OTYPER |= (type<< pin);

}

// GPIO Push-Pull    : No pull-up (00), Pull-up (01), Pull-down (10), Reserved (11)
void GPIO_pupd(GPIO_TypeDef *Port, int pin, int pupd){
	Port->PUPDR &= ~(3UL<<(2*pin));
	Port->PUPDR |= pupd<<(2*pin);
	
}


int GPIO_read(GPIO_TypeDef *Port, int pin){
	// return the value of present state
	int bitVal = ((Port->IDR)>>pin)&1; 
	
	return bitVal;
	
}


void bittoggle(GPIO_TypeDef *Port, int pin){
	
	Port->ODR ^= (1<<pin);
		
}

unsigned int LED_state[8][3] = {
	
	{0,0,0},
	{0,0,1},
	{0,1,0},
	{0,1,1},
	{1,0,0},
	{1,0,1},
	{1,1,0},
	{1,1,1},
	
};


void LED_toggle(uint8_t cnt){

		GPIO_write(GPIOA, LED_PA0, LED_state[cnt][0]);		// led a
		GPIO_write(GPIOA, LED_PA1, LED_state[cnt][1]);		// led b
		GPIO_write(GPIOB, LED_PB0, LED_state[cnt][2]);		// led c
		
}

//------------------------------ 7 SEGMENT -----------------------------------------

void sevensegment_init(void){
		
	GPIO_init(GPIOB, 7, OUTPUT);    	// calls RCC_GPIOA_enable()	
	GPIO_otype(GPIOB, 7, PUSH_PULL);
	GPIO_pupd(GPIOB, 7,NO_PUPD);
	GPIO_ospeed(GPIOB, 7, MEDIUM_SPEED);
	
	GPIO_init(GPIOC, 13, OUTPUT);    // calls RCC_GPIOA_enable()	
	GPIO_otype(GPIOC, 13, PUSH_PULL);
	GPIO_pupd(GPIOC, 13,NO_PUPD);
	GPIO_ospeed(GPIOC, 13, MEDIUM_SPEED);

	GPIO_init(GPIOC, 14, OUTPUT);    // calls RCC_GPIOA_enable()	
	GPIO_otype(GPIOC, 14, PUSH_PULL);
	GPIO_pupd(GPIOC, 14,NO_PUPD);
	GPIO_ospeed(GPIOC, 14, MEDIUM_SPEED);
	
	
	GPIO_init(GPIOC, 15, OUTPUT);    // calls RCC_GPIOA_enable()	
	GPIO_otype(GPIOC, 15, PUSH_PULL);
	GPIO_pupd(GPIOC, 15,NO_PUPD);
	GPIO_ospeed(GPIOC, 15, MEDIUM_SPEED);

	GPIO_init(GPIOH, 0, OUTPUT);    // calls RCC_GPIOA_enable()	
	GPIO_otype(GPIOH, 0, PUSH_PULL);
	GPIO_pupd(GPIOH, 0,NO_PUPD);
	GPIO_ospeed(GPIOH, 0, MEDIUM_SPEED);

	GPIO_init(GPIOH, 1, OUTPUT);    // calls RCC_GPIOA_enable()	
	GPIO_otype(GPIOH, 1, PUSH_PULL);
	GPIO_pupd(GPIOH, 1,NO_PUPD);
	GPIO_ospeed(GPIOH, 1, MEDIUM_SPEED);
	
	GPIO_init(GPIOC, 2, OUTPUT);    // calls RCC_GPIOA_enable()	
	GPIO_otype(GPIOC, 2, PUSH_PULL);
	GPIO_pupd(GPIOC, 2,NO_PUPD);
	GPIO_ospeed(GPIOC, 2, MEDIUM_SPEED);
	
	GPIO_init(GPIOC, 3, OUTPUT);    // calls RCC_GPIOA_enable()	
	GPIO_otype(GPIOC, 3, PUSH_PULL);
	GPIO_pupd(GPIOC, 3,NO_PUPD);
	GPIO_ospeed(GPIOC, 3, MEDIUM_SPEED);

	GPIO_write(GPIOB, 7, LOW);		// led a
	GPIO_write(GPIOC, 13, LOW);		// led b
	GPIO_write(GPIOC, 14, LOW);		// led c
	GPIO_write(GPIOC, 15, HIGH);	// led d		
	GPIO_write(GPIOH, 0, LOW);		// led e
	GPIO_write(GPIOH, 1, LOW);		// led f
	GPIO_write(GPIOC, 2, HIGH);		// led g
	GPIO_write(GPIOC, 3, LOW);		// led dp
	
}


// FOR FINAL PROJECT
unsigned int sevensegment_state[3][8]={
		// order : PB7 |PC13|PC14 |PC15 |PH0 |PH1 |PC2	|PC3
		//					a  | b  | c   | d   | e  | f  | g	| dp
    {0,0,0,1,0,0,1,0},          // A
    {0,1,0,0,1,0,0,0},          // S
    {0,1,1,0,0,0,0,0},          // E
										
};

//void sevensegment_decoder(uint8_t num){
//	GPIO_write(GPIOB, 7, sevensegment_state[num][0]);			// led a
//	GPIO_write(GPIOC, 13, sevensegment_state[num][1]);		// led b
//	GPIO_write(GPIOC, 14, sevensegment_state[num][2]);		// led c
//	GPIO_write(GPIOC, 15, sevensegment_state[num][3]);		// led d		
//	GPIO_write(GPIOH, 0, sevensegment_state[num][4]);			// led e
//	GPIO_write(GPIOH, 1, sevensegment_state[num][5]);			// led f
//	GPIO_write(GPIOC, 2, sevensegment_state[num][6]);			// led g
//	GPIO_write(GPIOC, 3,sevensegment_state[num][7]);			// led dp
//}

void sevensegment_decoder(uint8_t num){
	GPIO_write(GPIOB, 7, LOW);			// led a
	GPIO_write(GPIOC, 13,LOW);		// led b
	GPIO_write(GPIOC, 14,LOW);		// led c
	GPIO_write(GPIOC, 15,LOW);		// led d		
	GPIO_write(GPIOH, 0, LOW);			// led e
	GPIO_write(GPIOH, 1, LOW);			// led f
	GPIO_write(GPIOC, 2, LOW);			// led g
	GPIO_write(GPIOC, 3, LOW);			// led dp
}


//-------------------------------------------------------------------------------

//unsigned int sevensegment_state[11][8]={
//		// order : PB5 |PA6 |PA7 |PB6 |PC7 |PA9 |PA8	|PB10
//		//					a  | b  | c  | d  | e  | f  | g		| dp
//    {0,0,0,0,0,0,1,0},          //zero
//    {1,0,0,1,1,1,1,0},          //one
//    {0,0,1,0,0,1,0,0},          //two
//    {0,0,0,0,1,1,0,0},          //three
//    {1,0,0,1,1,0,0,0},          //four
//    {0,1,0,0,1,0,0,0},          //five
//    {0,1,0,0,0,0,0,0},          //six
//    {0,0,0,1,1,1,1,0},          //seven
//    {0,0,0,0,0,0,0,0},          //eight
//    {0,0,0,1,1,0,0,0},          //nine
//		{0,0,1,1,0,0,0,0},					//pause mode
//										
//};




//void sevensegment_decoder(uint8_t num){
//	GPIO_write(GPIOB, LED_PB9, sevensegment_state[num][0]);		// led a
//	GPIO_write(GPIOA, LED_PA6, sevensegment_state[num][1]);		// led b
//	GPIO_write(GPIOA, LED_PA7, sevensegment_state[num][2]);		// led c
//	GPIO_write(GPIOB, LED_PB6, sevensegment_state[num][3]);		// led d		
//	GPIO_write(GPIOC, LED_PC7, sevensegment_state[num][4]);		// led e
//	GPIO_write(GPIOA, LED_PA9, sevensegment_state[num][5]);		// led f
//	GPIO_write(GPIOA, LED_PA8, sevensegment_state[num][6]);		// led g
//	GPIO_write(GPIOB, LED_PB10,sevensegment_state[num][7]);		// led dp
//}