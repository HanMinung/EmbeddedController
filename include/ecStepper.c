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
uint32_t direction = 1; 
uint32_t step_delay = 100; 
uint32_t step_per_rev = 64*32;

// Stepper Motor variable
volatile Stepper_t myStepper; 

//FULL stepping sequence  - FSM
typedef struct {
	uint8_t out;
  uint32_t next[2];
} State_full_t;

// FSM 테이블 작성한거를 그대로 넣으면 된다 : 왼쪽은 CW or CCW
// sequence와 하드웨어 pin이 잘 매칭되는지 꼭 확인해야함
State_full_t FSM_full[4] = {  
	
 {0b1010,{S1,S3}},						// S0	
 {0b0110,{S2,S0}},						// S1
 {0b0101,{S3,S1}},						// S2
 {0b1001,{S0,S2}}							// S3
};

// e.g : output = FSM[S0].out
// e.g : state = FSM[S0].next[0]  인자에 들어갈 거는 현재 state

// HALF stepping sequence
typedef struct {
	uint8_t out;
  uint32_t next[2];
} State_half_t;

// Half state에 대한 FSM 테이블을 그대로 넣으면 된다
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
	myStepper.port1 = port1;
  myStepper.pin1  = pin1;
	myStepper.port2 = port2;
  myStepper.pin2  = pin2;
	myStepper.port3 = port3;
  myStepper.pin3  = pin3;
	myStepper.port4 = port4;
  myStepper.pin4  = pin4;	
	
// (GPIPO_TypeDef *Port1,int pin,int TYPE,int OTYPE,int PUPD,int OSPEED)
	indiv_init(port1,pin1,OUTPUT,PUSH_PULL,NO_PULL_UP,FAST_SPEED);
	indiv_init(port2,pin2,OUTPUT,PUSH_PULL,NO_PULL_UP,FAST_SPEED);
	indiv_init(port3,pin3,OUTPUT,PUSH_PULL,NO_PULL_UP,FAST_SPEED);
	indiv_init(port4,pin4,OUTPUT,PUSH_PULL,NO_PULL_UP,FAST_SPEED);
	
}

// 실제로 stepper에게 각 핀마다 어떤 아웃풋을 줄건지 전달해주는 함수
// 어떻게 하면 쉽게 전달가능할지 고민해볼것
// 비트연산으로 넣을 수 있다.
// 위에서 정의한 state를 이용해서 bit연산으로 출력할 수 있다.
void Stepper_pinOut (uint32_t state, int mode){
	
	   if (mode == FULL){         // FULL mode
			GPIO_write(myStepper.port1, myStepper.pin1, 		// YOUR CODE_____);
  		 // Repeat for port2,pin2~port4,pin4 


			}	 
		 else if (mode == HALF){    // HALF mode
			 	// YOUR CODE 
				// YOUR CODE 
				// YOUR CODE 
				// YOUR CODE 
			}
}

// 한 phase에 90도 인데, 90도를 64개로
// 한 스텝에 5.62 deg만큼 움직인다.
// 기어비 때문데 32:1 
// 출력되는 바퀴수는 계산해봗야 한다.
// 한 스텝 이동하고, 또 한스텝 이동하는 데에 얼마만큼의 딜레이를 줄건지를 결정하는 함수 : 어느 정도일지 결정해줘야함
void Stepper_setSpeed (long whatSpeed){      // rppm
		step_delay = 	_________//YOUR CODE   // Convert rpm to milli sec
}


void Stepper_step(int steps, int direction, int mode){
	 uint32_t state = 0;
	 myStepper._step_num = steps;

	 for(; myStepper._step_num > 0; myStepper._step_num--){ // run for step size
				// YOUR CODE                                  // delay (step_delay); 
					 
		    if (mode == FULL) 		 												
						state = ___________// YOUR CODE       // state = next state
				else if (mode == HALF) 
						state = ___________// YOUR CODE       // state = next state
				
				Stepper_pinOut(state, mode);
   }
}


void Stepper_stop (void){ 
     
    	myStepper._step_num = 0;    
			// All pins(Port1~4, Pin1~4) set as DigitalOut '0'
			 	// YOUR CODE 
				// YOUR CODE 
				// YOUR CODE 
				// YOUR CODE 
}

