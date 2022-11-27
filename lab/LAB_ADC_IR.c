/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2021-8-12 by YKKIM  	
* @brief   Embedded Controller:  LAB 
*					 - Line tracing using IR sensors
* 
******************************************************************************
*/
#include "ecHAL.h"
//IR parameter//
#define END_CHAR 13
#define A 	0
#define B 	1

uint32_t IR1, IR2;
int flag = 0;
int seqCHn[16] = {8,9,};				// 8번 채널 9번 채널 순서

//--------------------  Variable declaration --------------------
uint8_t pcData = 0;
uint8_t mcuData = 0;

uint8_t DIR = LOW;
int	endChar = 13;

float duty_A = 0.60;
float duty_B = 0.60;

uint32_t ovf_cnt = 0;
float distance = 0;
float timeInterval = 0;
float time1 = 0;
float time2 = 0;
float PulseWidth = 10;

_Pin dcPwmPin[2] = {
	// PC9
	// PC8
	{GPIOC, 9}, // TIM3 Ch3
	{GPIOC, 8}	// TIM3 Ch4
};

PWM_t dcPwm[2];

_Pin dcDirPin[2] = {
	// PB8
	// PC6
	{GPIOB, 8}, 
	{GPIOC, 6}	
};

//-------------------------------------------------------------------

void setup(void);
void TIM2_IRQHandler(void);


int main(void) { 

	// Initialiization --------------------------------------------------------
	setup();
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		
//		printf("IR1 = %d \r\n",IR1);
//		printf("IR2 = %d \r\n",IR2);
//		printf("\r\n");
		
		if (IR1 > 1000){
		
			//printf("GO RIGHT\r\n");
			
			PWM_duty(&dcPwm[A], 1.0);
			PWM_duty(&dcPwm[B], 0.60);

			GPIO_write(dcDirPin[A].port, dcDirPin[A].pin, DIR);
			GPIO_write(dcDirPin[B].port, dcDirPin[B].pin, DIR);
			
			if(distance > 2 && distance < 25){
			
				PWM_duty(&dcPwm[A], 0.0);
				PWM_duty(&dcPwm[B], 0.0);
	
				GPIO_write(dcDirPin[A].port, dcDirPin[A].pin, DIR);
				GPIO_write(dcDirPin[B].port, dcDirPin[B].pin, DIR);
				
			}
			
		}
		
		else if (IR2 > 1000) {
			
			//printf("GO LEFT\r\n");
		
			PWM_duty(&dcPwm[A], 0.60);
			PWM_duty(&dcPwm[B], 1.0);

			GPIO_write(dcDirPin[A].port, dcDirPin[A].pin, DIR);
			GPIO_write(dcDirPin[B].port, dcDirPin[B].pin, DIR);
			
			if(distance > 2 && distance < 25){
			
				PWM_duty(&dcPwm[A], 0.0);
				PWM_duty(&dcPwm[B], 0.0);
	
				GPIO_write(dcDirPin[A].port, dcDirPin[A].pin, DIR);
				GPIO_write(dcDirPin[B].port, dcDirPin[B].pin, DIR);
				
			}
			
		}
		
		else{
			
			PWM_duty(&dcPwm[A], duty_A);
			PWM_duty(&dcPwm[B], duty_B);

			GPIO_write(dcDirPin[A].port, dcDirPin[A].pin, DIR);
			GPIO_write(dcDirPin[B].port, dcDirPin[B].pin, DIR);
			
			if(distance > 2 && distance < 25){
			
				PWM_duty(&dcPwm[A], 0.0);
				PWM_duty(&dcPwm[B], 0.0);
	
				GPIO_write(dcDirPin[A].port, dcDirPin[A].pin, DIR);
				GPIO_write(dcDirPin[B].port, dcDirPin[B].pin, DIR);
				
			}
		
		}
		
//		printf("\r\n");
//		printf("distance : %lf",distance);
		
		delay_ms(200);
	
	}
}

// Initialiization 
void setup(void){	
	
	RCC_PLL_init();                         					 // System Clock = 84MHz
	SysTick_init(1);
	
	USART_init(USART2, 9600);                          // PC <--> mcu.
	printf("1\r\n");
	USART_begin(USART1, GPIOA, 9, GPIOA, 10, 9600);    // Bluetooth
	
	for (int i = 0; i < 2; i++){

		GPIO_init(dcDirPin[i].port, dcDirPin[i].pin, OUTPUT);
		GPIO_pupd(dcDirPin[i].port, dcDirPin[i].pin, EC_PD);
		GPIO_otype(dcDirPin[i].port, dcDirPin[i].pin, PUSH_PULL);
		GPIO_ospeed(dcDirPin[i].port, dcDirPin[i].pin, HIGH_SPEED);
		
	}
	
	// ADC setting
	// 채널 8,9에 대한 TRGO 설정
  ADC_init(GPIOB, 0, TRGO);
	ADC_init(GPIOB, 1, TRGO);

	// ADC channel sequence setting
	// 멀티 채널이기 때문에 sequence를 써야한다.
	// sequence에 맞게 세팅을 해달라는 의미
	ADC_sequence(2, seqCHn);
	
	// ADON, SW Trigger enable
	ADC_start();

	
		// PWM configuration
	PWM_init(&dcPwm[A], dcPwmPin[A].port, dcPwmPin[A].pin);
	PWM_init(&dcPwm[B], dcPwmPin[B].port, dcPwmPin[B].pin);
	
	PWM_period_us(&dcPwm[A], 100);
	PWM_period_us(&dcPwm[B], 100);
	
	PWM_duty(&dcPwm[A], duty_A);
	PWM_duty(&dcPwm[B], duty_B);

	// A : 왼쪽 바퀴 , B : 오른쪽 바퀴
	// HIGH : Ultrasonic sensor opposite direction (Duty ratio : low set)
	// LOW : Ultrasonic sensor direction (Duty ratio : high set)
	GPIO_write(dcDirPin[A].port, dcDirPin[A].pin, LOW);
	GPIO_write(dcDirPin[B].port, dcDirPin[B].pin, LOW);

	
//	// PWM configuration -----------------------------------------------------------------
	PWM_t trig;
	PWM_init(&trig, GPIOA, LED_PA6);
	PWM_period_ms(&trig, 50);
	PWM_pulsewidth_us(&trig, 10);

	// Input capture configuration ---------------------------------------------------------
	IC_t echo;
	ICAP_init(&echo, GPIOB, LED_PB10);
	ICAP_counter_us(&echo,10);
	ICAP_setup(&echo, 3, IC_RISE);
	ICAP_setup(&echo, 4, IC_FALL);
	
	//printf("SETUP complete\n");
}


void ADC_IRQHandler(void){
	
	if((is_ADC_OVR())){
		
		clear_ADC_OVR();
	}
	
	if(is_ADC_EOC()){       //after finishing sequence
			
			// REGISTER가 하나라서 두개를 동시에 읽을 수 없다
			// flag가 0일때는 IR1에 있는값을 읽는다
			if (flag == 0){
				IR1 = ADC_read();
			}  
			
			// flag가 1일때는 IR2에 있는값을 읽는다
			else if (flag == 1){
				IR2 = ADC_read();
			}
			
		flag =! flag;
			
	}
}


void TIM2_IRQHandler(){

	if(is_UIF(TIM2)){
		
		// TIMx_SR : UIF (bit 0) 1 : Update interrupt pending
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

