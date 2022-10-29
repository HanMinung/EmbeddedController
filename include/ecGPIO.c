/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : HanMinung
Created          : 05-03-2021
Modified         : 10-29-2022
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

void indiv_init(GPIO_TypeDef *Port,int pin,int TYPE,int OTYPE,int PUPD,int OSPEED){

	GPIO_init(Port, pin, OUTPUT);    // calls RCC_GPIOA_enable()	
	GPIO_otype(Port, pin, PUSH_PULL);
	GPIO_pupd(Port, pin,NO_PULL_UP);
	GPIO_ospeed(Port, pin, MEDIUM_SPEED);
	
}

// PB4 , PB5 , PB3 , PA10 sequencially
void LED_init(GPIO_TypeDef *Port1, int pin1, GPIO_TypeDef *Port2, int pin2, GPIO_TypeDef *Port3, int pin3, GPIO_TypeDef *Port4, int pin4){

	GPIO_init(Port1, pin1, OUTPUT);    // calls RCC_GPIOA_enable()	
	GPIO_otype(Port1, pin1, PUSH_PULL);
	GPIO_pupd(Port1, pin1,NO_PULL_UP);
	GPIO_ospeed(Port1, pin1, MEDIUM_SPEED);
	
	GPIO_init(Port2, pin2, OUTPUT);    // calls RCC_GPIOA_enable()	
	GPIO_otype(Port2, pin2, PUSH_PULL);
	GPIO_pupd(Port2, pin2,NO_PULL_UP);
	GPIO_ospeed(Port2, pin2, MEDIUM_SPEED);

	GPIO_init(Port3, pin3, OUTPUT);    // calls RCC_GPIOA_enable()	
	GPIO_otype(Port3, pin3, PUSH_PULL);
	GPIO_pupd(Port3, pin3,NO_PULL_UP);
	GPIO_ospeed(Port3, pin3, MEDIUM_SPEED);
	
	GPIO_init(Port4, pin4, OUTPUT);    // calls RCC_GPIOA_enable()	
	GPIO_otype(Port4, pin4, PUSH_PULL);
	GPIO_pupd(Port4, pin4,NO_PULL_UP);
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

unsigned int LED_state[16][4] = {
	
	{0,0,0,0},
	{0,0,0,1},
	{0,0,1,0},
	{0,0,1,1},
	{0,1,0,0},
	{0,1,0,1},
	{0,1,1,0},
	{0,1,1,1},
	{1,0,0,0},
	{1,0,0,1},
	{1,0,1,0},
	{1,0,1,1},
	{1,1,0,0},
	{1,1,0,1},
	{1,1,1,0},
	{1,1,1,1},
	
};


void LED_toggle(uint8_t cnt){

		GPIO_write(GPIOA, LED_PA0, LED_state[cnt][0]);		// led a
		GPIO_write(GPIOA, LED_PA1, LED_state[cnt][1]);		// led b
		GPIO_write(GPIOB, LED_PB0, LED_state[cnt][2]);		// led c
		GPIO_write(GPIOC, LED_PC1, LED_state[cnt][3]);		// led d
		
}

//------------------------------ 7 SEGMENT -----------------------------------------

void sevensegment_init(void){
		
	// 시험때 LED_PB9으로 바뀜
	GPIO_init(GPIOB, LED_PB9, OUTPUT);    // calls RCC_GPIOA_enable()	
	GPIO_otype(GPIOB, LED_PB9, PUSH_PULL);
	GPIO_pupd(GPIOB, LED_PB9,NO_PULL_UP);
	GPIO_ospeed(GPIOB, LED_PB9, MEDIUM_SPEED);
	
	GPIO_init(GPIOA, LED_PA6, OUTPUT);    // calls RCC_GPIOA_enable()	
	GPIO_otype(GPIOA, LED_PA6, PUSH_PULL);
	GPIO_pupd(GPIOA, LED_PA6,NO_PULL_UP);
	GPIO_ospeed(GPIOA, LED_PA6, MEDIUM_SPEED);

	GPIO_init(GPIOA, LED_PA7, OUTPUT);    // calls RCC_GPIOA_enable()	
	GPIO_otype(GPIOA, LED_PA7, PUSH_PULL);
	GPIO_pupd(GPIOA, LED_PA7,NO_PULL_UP);
	GPIO_ospeed(GPIOA, LED_PA7, MEDIUM_SPEED);
	
	
	GPIO_init(GPIOB, LED_PB6, OUTPUT);    // calls RCC_GPIOA_enable()	
	GPIO_otype(GPIOB, LED_PB6, PUSH_PULL);
	GPIO_pupd(GPIOB, LED_PB6,NO_PULL_UP);
	GPIO_ospeed(GPIOB, LED_PB6, MEDIUM_SPEED);

	GPIO_init(GPIOC, LED_PC7, OUTPUT);    // calls RCC_GPIOA_enable()	
	GPIO_otype(GPIOC, LED_PC7, PUSH_PULL);
	GPIO_pupd(GPIOC, LED_PC7,NO_PULL_UP);
	GPIO_ospeed(GPIOC, LED_PC7, MEDIUM_SPEED);

	GPIO_init(GPIOA, LED_PA9, OUTPUT);    // calls RCC_GPIOA_enable()	
	GPIO_otype(GPIOA, LED_PA9, PUSH_PULL);
	GPIO_pupd(GPIOA, LED_PA9,NO_PULL_UP);
	GPIO_ospeed(GPIOA, LED_PA9, MEDIUM_SPEED);
	
	GPIO_init(GPIOA, LED_PA8, OUTPUT);    // calls RCC_GPIOA_enable()	
	GPIO_otype(GPIOA, LED_PA8, PUSH_PULL);
	GPIO_pupd(GPIOA, LED_PA8,NO_PULL_UP);
	GPIO_ospeed(GPIOA, LED_PA8, MEDIUM_SPEED);
	
	GPIO_init(GPIOB, LED_PB10, OUTPUT);    // calls RCC_GPIOA_enable()	
	GPIO_otype(GPIOB, LED_PB10, PUSH_PULL);
	GPIO_pupd(GPIOB, LED_PB10,NO_PULL_UP);
	GPIO_ospeed(GPIOB, LED_PB10, MEDIUM_SPEED);

	GPIO_write(GPIOA, LED_PA5, LOW);		// led a
	GPIO_write(GPIOA, LED_PA6, LOW);		// led b
	GPIO_write(GPIOA, LED_PA7, LOW);		// led c
	GPIO_write(GPIOA, LED_PB6, LOW);		// led d		
	GPIO_write(GPIOC, LED_PC7, LOW);		// led e
	GPIO_write(GPIOA, LED_PA9, LOW);		// led f
	GPIO_write(GPIOA, LED_PA8, HIGH);		// led g
	GPIO_write(GPIOB, LED_PB10, LOW);		// led dp
}


unsigned int sevensegment_state[11][8]={
		// order : PB5 |PA6 |PA7 |PB6 |PC7 |PA9 |PA8	|PB10
		//					a  | b  | c  | d  | e  | f  | g		| dp
    {0,0,0,0,0,0,1,0},          //zero
    {1,0,0,1,1,1,1,0},          //one
    {0,0,1,0,0,1,0,0},          //two
    {0,0,0,0,1,1,0,0},          //three
    {1,0,0,1,1,0,0,0},          //four
    {0,1,0,0,1,0,0,0},          //five
    {0,1,0,0,0,0,0,0},          //six
    {0,0,0,1,1,1,1,0},          //seven
    {0,0,0,0,0,0,0,0},          //eight
    {0,0,0,1,1,0,0,0},          //nine
		{0,0,1,1,0,0,0,0},					//pause mode
										
};


void sevensegment_decoder(uint8_t num){
	GPIO_write(GPIOB, LED_PB9, sevensegment_state[num][0]);		// led a
	GPIO_write(GPIOA, LED_PA6, sevensegment_state[num][1]);		// led b
	GPIO_write(GPIOA, LED_PA7, sevensegment_state[num][2]);		// led c
	GPIO_write(GPIOB, LED_PB6, sevensegment_state[num][3]);		// led d		
	GPIO_write(GPIOC, LED_PC7, sevensegment_state[num][4]);		// led e
	GPIO_write(GPIOA, LED_PA9, sevensegment_state[num][5]);		// led f
	GPIO_write(GPIOA, LED_PA8, sevensegment_state[num][6]);		// led g
	GPIO_write(GPIOB, LED_PB10,sevensegment_state[num][7]);		// led dp
}