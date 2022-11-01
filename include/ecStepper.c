#include "stm32f4xx.h"
#include "ecStepper.h"
#include "ecGPIO.h"

//State number : Full stepping 4개 사용, Half stepping 8개 사용 
#define S0 0
#define S1 1
#define S2 2
#define S3 3
#define S4 4
#define S5 5
#define S6 6
#define S7 7

// Stepper Motor function
uint32_t dir = 1; 
uint32_t step_delay = 100; 
uint32_t step_per_rev = 64*32;

// Stepper Motor variable
volatile Stepper_t myStepper; 

//FULL stepping sequence  - FSM
typedef struct {
	
	uint8_t out;
  uint32_t next[2];
} State_full_t;


// FSM table : full mode
State_full_t FSM_full[4] = {  
	
 {0b1010,{S1,S3}},						// S0	
 {0b0110,{S2,S0}},						// S1
 {0b0101,{S3,S1}},						// S2
 {0b1001,{S0,S2}}							// S3
};

// HALF stepping sequence
typedef struct {
	
	uint8_t out;
  uint32_t next[2];
} State_half_t;


// FSM table : half mode
State_half_t FSM_half[8] = { 
 {0b1000,{S1,S7}},
 {0b1010,{S2,S0}},
 {0b0010,{S3,S1}},
 {0b0100,{S4,S2}},
 {0b0110,{S5,S3}},
 {0b0100,{S6,S4}},
 {0b0101,{S7,S5}},
 {0b0001,{S0,S6}}
};



void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4){
	 
//  GPIO Digital Out Initiation
	myStepper.port1 = port1;		myStepper.pin1  = pin1;
	myStepper.port2 = port2;		myStepper.pin2  = pin2;
	myStepper.port3 = port3;		myStepper.pin3  = pin3;
	myStepper.port4 = port4;		myStepper.pin4  = pin4;	
	
// (GPIPO_TypeDef *Port1,int pin,int TYPE,int OTYPE,int PUPD,int OSPEED)
	indiv_init(port1,pin1,OUTPUT,PUSH_PULL,NO_PUPD,FAST_SPEED);
	indiv_init(port2,pin2,OUTPUT,PUSH_PULL,NO_PUPD,FAST_SPEED);
	indiv_init(port3,pin3,OUTPUT,PUSH_PULL,NO_PUPD,FAST_SPEED);
	indiv_init(port4,pin4,OUTPUT,PUSH_PULL,NO_PUPD,FAST_SPEED);
	
}

//  n rpm 기준 : 한 스텝을 회전하는데 몇 ms로 할지 결정하는 함수
void Stepper_setSpeed (long whatSpeed,int mode){  // what speed : rpm 기준
	
	if(mode == FULL)	step_delay = 60000 / (FULL_ANGLE * GEAR_RATIO * whatSpeed);
	else if(mode == HALF)		step_delay = 60000 / (HALF_ANGLE * GEAR_RATIO * whatSpeed);
	
	delay_ms(step_delay);
}


void Stepper_step(int steps, int dir, int mode, long rpm){
	 
	 uint32_t state = 0;
	 myStepper._step_num = steps;

	 for(; myStepper._step_num > 0; myStepper._step_num--){ 			// run for step size
				Stepper_setSpeed(rpm,mode);                             // delay (step_delay); 
					 
		    if (mode == FULL) 		 												
						state = FSM_full[state].next[dir];      					  // state = next state
				
				else if (mode == HALF) 	
						state = FSM_half[state].next[dir];       						// state = next state
				
				Stepper_pinOut(state, mode);
				
   }
}


void Stepper_pinOut (uint32_t state, int mode){
	
	   if (mode == FULL){         // FULL mode
			
			GPIO_write(myStepper.port1, myStepper.pin1, FSM_full[state].out >> 3 & 1);
			GPIO_write(myStepper.port2, myStepper.pin2, FSM_full[state].out >> 2 & 1);
			GPIO_write(myStepper.port3, myStepper.pin3, FSM_full[state].out >> 1 & 1);
			GPIO_write(myStepper.port4, myStepper.pin4, FSM_full[state].out >> 0 & 1);
		 }
		 
		 else if (mode == HALF){    // HALF mode
			
			GPIO_write(myStepper.port1, myStepper.pin1, FSM_half[state].out >> 3 & 1);
			GPIO_write(myStepper.port2, myStepper.pin2, FSM_half[state].out >> 2 & 1);
			GPIO_write(myStepper.port3, myStepper.pin3, FSM_half[state].out >> 1 & 1);
			GPIO_write(myStepper.port4, myStepper.pin4, FSM_half[state].out >> 0 & 1);
			}
}


void Stepper_stop (void){ 
     
    	myStepper._step_num = 0;   
	
			GPIO_write(myStepper.port1, myStepper.pin1, myStepper._step_num);
			GPIO_write(myStepper.port2, myStepper.pin2, myStepper._step_num);
			GPIO_write(myStepper.port3, myStepper.pin3, myStepper._step_num);
			GPIO_write(myStepper.port4, myStepper.pin4, myStepper._step_num);
	
}

