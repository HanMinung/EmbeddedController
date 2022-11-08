/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2022-8-12 by YKKIM  	
* @brief   Embedded Controller:  Quis1
*					 - HanMinung , 21800773
* 
******************************************************************************
*/
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecEXTI.h"
#include "ecTIM.h"
#include "ecSysTick.h"


uint32_t cnt = 0;
uint32_t cnt2 = 0;
uint32_t idx = 0;
uint32_t cnt_seg = 0;
uint32_t cnt_but = 0;
uint32_t button_flag = 0;

void setup(void);
void EXTI15_10_IRQHandler(void);	
void TIM2_IRQHandler(void);
	
	
int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();

//	// Inifinite Loop ----------------------------------------------------------
	while(1){ 	
		
		if(button_flag == 0){
			
			GPIO_write(GPIOA,LED_PA5,HIGH);
		}
		
		else{
		
			bittoggle(GPIOA,LED_PA5);
			delay_ms(500);
		}	
		
	}
}

// Initialiization 
void setup(void)
{	
	
	RCC_PLL_init();               					  // System Clock = 84MHz
	// 먼저, PA0, PA1, PB0, PC1에 대해 설정
	
	SysTick_init(1);
	GPIO_init(GPIOA, LED_PA5, OUTPUT);   		 // calls RCC_GPIOA_enable()	
	GPIO_otype(GPIOA, LED_PA5, PUSH_PULL);
	GPIO_pupd(GPIOA, LED_PA5,NO_PUPD);
	GPIO_ospeed(GPIOA, LED_PA5, MEDIUM_SPEED);
	LED_init(GPIOA,LED_PA0,GPIOA,LED_PA1,GPIOB,LED_PB0,GPIOC,LED_PC1);
	sevensegment_init();
	
	EXTI_init(GPIOC,BUTTON_PIN,RISE,0);				// EXTI initialization , button pin,rising edge , priority : 0
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);			// BUTTON_PIN : INPUT
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PD);			// BUTTON_PIN : PULL_DOWN
	TIM_period_us(TIM2,10);
	TIM_INT_init(TIM2,1);
	
}


void EXTI15_10_IRQHandler(void) { 
	
	if (is_pending_EXTI(BUTTON_PIN)) {
		
		// 한 번 버튼이 눌릴때마다 button cnt를 증가
		cnt_but ++;

		
		// 버튼을 누른 횟수 : 홀수 번 ( P를 display에 출력 )
		if(cnt_but%2 == 1)	{
			sevensegment_decoder(10);
			TIM2->CR1 &= ~1 <<0;
			
		}
		// 버튼을 누른 횟수 : 짝수 번 ( 기존의 숫자를 display에 출력 )
		else if(cnt_but%2 == 0)		{
			sevensegment_decoder(cnt_seg);
			TIM2->CR1 |= 1 <<0;
			
		}
		
		button_flag ^= 1;
		clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
		
	}
	
}



void TIM2_IRQHandler(void){
	if((TIM2->SR & TIM_SR_UIF) ==	 1){ // update interrupt flag
		
		// 0.5마다 count가 500씩 증가하게 된다.
	  cnt ++;
		
		if(cnt >= 1000){
		
			idx ++;
			LED_toggle(idx%8);
			
			// initialization
			cnt = 0;
			
			// 한 바퀴 돌면 segment cnt를 증가
			if(idx % 8 == 0){
				// 한 바퀴는 돌아서 증가를 해야하니 조건문이 필요
				if(idx != 0){
					cnt_seg ++;
					sevensegment_decoder(cnt_seg);
				}
			}
		}
		
		
		// Check the flag in status register
		TIM2->SR &= ~TIM_SR_UIF;                  // clear by writing 0
	}
	
}