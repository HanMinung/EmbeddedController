/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : HanMinung
Created          : 05-03-2021
Modified         : 10-29-2022
Language/ver     : C++ in Keil uVision
/----------------------------------------------------------------*/

/*
TIMERx에 따라 resolution이 달라지고, 그에 따라 ARRval을 uint8_t OR uint16_t OR uint32_t 뭘로 할지 설정
advanced timer & general timer 자료 보고 조건문으로 설정하기
TIMER2,5에 대해서는 16bit로 arrval을 설정해야 한다.
1~16 bit : PSC는 모두 동일하다
counter resolution을 보고 
*/

#include "ecPWM.h"
#include "ecGPIO.h"
#include "ecTIM.h"

/* PWM Configuration */

void PWM_init(PWM_t *pwm, GPIO_TypeDef *port, int pin,int ospeed, int otype, int pupd){
// 0. Match Output Port and Pin for TIMx 	
		pwm->port = port;
		pwm->pin  = pin;
		PWM_pinmap(pwm);
		// Port, pin에 따라 Timer가 자동 할당된다.
		TIM_TypeDef *TIMx = pwm->timer;
		int CHn = pwm->ch;	

// 1. Initialize GPIO port and pin as AF
		GPIO_init(port, pin, AF);   		 // AF=2
		GPIO_ospeed(port, pin, ospeed);  // speed VHIGH=3
		GPIO_otype(port,pin,otype);
		GPIO_pupd(port,pin,pupd);
	
	
// 2. Configure GPIO AFR by Pin num.				
	  //  AFR[0] for pin: 0~7  &   AFR[1] for pin 8~15
	  // AF1: TIM1,2      AF2: TIM3~5         AF3: TIM9~11
	  //  TIM1&TIM2 : AFR=1  | TIM3 : AFR=2  |  TIM3 : etc
		uint16_t AFx = 0;
	
		if ((TIMx == TIM1) || (TIMx == TIM2)) { AFx = 1UL;}
    else if ((TIMx == TIM3) || (TIMx == TIM4) || (TIMx == TIM5)) { AFx = 2UL; }
    else if ((TIMx == TIM9) || (TIMx == TIM10) || (TIMx == TIM11)) { AFx = 3UL; }

    // 각 핀별로 AFR 배열로 들어가는 값의 일반화
    port->AFR[pin/8] &= ~(0xFUL << (4*(pin%8)));        // 4 bit clear AFRx
    port->AFR[pin/8] |= AFx << (4*(pin%8)); 
			
	
// 3. Initialize Timer 
		TIM_init(TIMx, 1);						// with default msec=1 value.		
		TIMx->CR1 &= ~TIM_CR1_CEN;		// disable counter
// 3-2. Direction of Counter
		TIMx->CR1 |= 1<<4UL;   				// Counting direction: 0 = up-counting, 1 = down-counting
	
			
// 4. Configure Timer Output mode as PWM
// CHANNEL 3,4 --> CCMR2
	uint32_t ccVal=TIMx->ARR/2;  // default value : CC=ARR/2
	
	if(CHn == 1){
		TIMx->CCMR1 &= ~TIM_CCMR1_OC1M;                     // Clear ouput compare mode bits for channel 1
		TIMx->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // OC1M = 110 for PWM Mode 1 output on ch1.
		TIMx->CCMR1	|= TIM_CCMR1_OC1PE;                     // Output 1 preload enable (make CCR1 value changable)
		TIMx->CCR1  = ccVal; 																// Output Compare Register for channel 1 (default duty ratio = 50%)		
		TIMx->CCER  &= ~TIM_CCER_CC1P;                      // select output polarity: active high	
		TIMx->CCER  |= TIM_CCER_CC1E;												// Enable output for ch1
	}
	
	else if(CHn == 2){
		TIMx->CCMR1 &= ~TIM_CCMR1_OC2M;                     
		TIMx->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2; 
		TIMx->CCMR1	|= TIM_CCMR1_OC2PE;                			
		TIMx->CCR2  = ccVal; 																
		TIMx->CCER  &= ~TIM_CCER_CC1P;                      	
		TIMx->CCER  |= TIM_CCER_CC2E;																
	}
	
	else if(CHn == 3){
		TIMx->CCMR2 &= ~TIM_CCMR2_OC3M;                     
		TIMx->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; 
		TIMx->CCMR2	|= TIM_CCMR2_OC3PE;                			
		TIMx->CCR3  = ccVal; 																
		TIMx->CCER  &= ~TIM_CCER_CC1P;                      	
		TIMx->CCER  |= TIM_CCER_CC3E;															
	}
	
	else if(CHn == 4){
		TIMx->CCMR2 &= ~TIM_CCMR2_OC4M;                     
		TIMx->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2; 
		TIMx->CCMR2	|= TIM_CCMR2_OC4PE;                			
		TIMx->CCR3  = ccVal; 																
		TIMx->CCER  &= ~TIM_CCER_CC1P;                      	
		TIMx->CCER  |= TIM_CCER_CC4E;			
	}	
	
	
	
// 5. Enable Timer Counter
	if(TIMx == TIM1) TIMx->BDTR |= TIM_BDTR_MOE;					// Main output enable (MOE): 0 = Disable, 1 = Enable	
	TIMx->CR1  |= TIM_CR1_CEN;  													// Enable counter
	
}


void PWM_period_ms(PWM_t *pwm, uint32_t msec){
	TIM_TypeDef *TIMx = pwm->timer;
	TIM_period_ms(TIMx, msec); 
	
}

void PWM_period_us(PWM_t *pwm, uint32_t usec){
	TIM_TypeDef *TIMx = pwm->timer;
	TIM_period_us(TIMx, usec); 	
	
}


void PWM_pulsewidth_ms(PWM_t *pwm, float pulse_width_ms){ 
	int CHn = pwm->ch;
	uint32_t fsys = 0;
	uint32_t psc = pwm->timer->PSC;
	
	// Check System CLK: PLL or HSI
	if((RCC->CFGR & (3<<0)) == 2)      { fsys = 84000; }  // for msec 84MHz/1000
	else if((RCC->CFGR & (3<<0)) == 0) { fsys = 16000; }
	
	float fclk = fsys/(psc+1);														// fclk=fsys/(psc+1);
	uint32_t ccval = (pulse_width_ms * fclk) - 1;					// pulse_width_ms *fclk - 1;
	
	//YOUR CODE GOES HERE
	switch(CHn){
		case 1: pwm->timer->CCR1 = ccval; break;
		case 2: pwm->timer->CCR2 = ccval; break;
		case 3: pwm->timer->CCR3 = ccval; break;
		case 4: pwm->timer->CCR4 = ccval; break;
		
		default: break;
	}
}


void PWM_duty(PWM_t *pwm, float duty) {                 //  duty=0 to 1	
	float ccval = (pwm->timer->ARR+1) * duty - 1;    			// (ARR+1)*dutyRatio - 1          
	int CHn = pwm->ch;
  	
	switch(CHn){
		case 1: pwm->timer->CCR1 = ccval;	  break;
		case 2: pwm->timer->CCR2 = ccval; 	break;
		case 3: pwm->timer->CCR3 = ccval; 	break;
		case 4: pwm->timer->CCR4 = ccval; 	break;
		
		default: break;
	}
}


// DO NOT MODIFY HERE
void PWM_pinmap(PWM_t *pwm){
   GPIO_TypeDef *port = pwm->port;
   int pin = pwm->pin;
   
   if(port == GPIOA) {
      switch(pin){
         case 0 : pwm->timer = TIM2; pwm->ch = 1; break;
         case 1 : pwm->timer = TIM2; pwm->ch = 2; break;
         case 5 : pwm->timer = TIM2; pwm->ch = 1; break;
         case 6 : pwm->timer = TIM3; pwm->ch = 1; break;
         //case 7: PWM_pin->timer = TIM1; PWM_pin->ch = 1N; break;
         case 8 : pwm->timer = TIM1; pwm->ch = 1; break;
         case 9 : pwm->timer = TIM1; pwm->ch = 2; break;
         case 10: pwm->timer = TIM1; pwm->ch = 3; break;
         case 15: pwm->timer = TIM2; pwm->ch = 1; break;
         default: break;
      }         
   }
   else if(port == GPIOB) {
      switch(pin){
         //case 0: PWM_pin->timer = TIM1; PWM_pin->ch = 2N; break;
         //case 1: PWM_pin->timer = TIM1; PWM_pin->ch = 3N; break;
         case 3 : pwm->timer = TIM2; pwm->ch = 2; break;
         case 4 : pwm->timer = TIM3; pwm->ch = 1; break;
         case 5 : pwm->timer = TIM3; pwm->ch = 2; break;
         case 6 : pwm->timer = TIM4; pwm->ch = 1; break;
         case 7 : pwm->timer = TIM4; pwm->ch = 2; break;
         case 8 : pwm->timer = TIM4; pwm->ch = 3; break;
         case 9 : pwm->timer = TIM4; pwm->ch = 4; break;
         case 10: pwm->timer = TIM2; pwm->ch = 3; break;
         
         default: break;
      }
   }
   else if(port == GPIOC) {
      switch(pin){
         case 6 : pwm->timer = TIM3; pwm->ch = 1; break;
         case 7 : pwm->timer = TIM3; pwm->ch = 2; break;
         case 8 : pwm->timer = TIM3; pwm->ch = 3; break;
         case 9 : pwm->timer = TIM3; pwm->ch = 4; break;
         
         default: break;
      }
   }
	 // TIM5 needs to be added, if used.
}

int update_dir(int dir,uint8_t idx){

		if(idx%19 == 0) dir *= -1;
	
		return dir;
}

float update_duty(uint8_t idx){
	
	return (0.5 + (idx*0.11))/20.0;
}


/* ------------------------------  TIMER , PIN number에 따른 AFR setting  ------------------------------*/
   /*   
            TIM   Ch    Por   pin
            1      1      A     8
            1      1n  	  A     7
            1      1n     B     13
                  
            1      2      A     9
            1      2n     B     0
            1      2n     B     14
                  
            1      3      A     10
            1      3n     B     11
            1      3n     B     15

            2      1      A      0
            2      1      A      5
            2      1      A      15

            2      2      A      1
            2      2      B      3

            2      3      B      10

            3      1      A      6
            3      1      B      4
            3      1      C      6

            3      2      B      5
            3      2      C      7
            
            3      4      C      9

            4      1      B      6
            4      2      B      7
            4      3      B      8
            4      4      B      9
         .... 
         이렇게 이미 다 지정이 되어 있다. 근데 그건 
         PWM_pinmap(pwm);여기서 해준다
   */
