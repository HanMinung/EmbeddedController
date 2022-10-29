/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : HanMinung
Created          : 05-03-2021
Modified         : 10-29-2022
Language/ver     : C++ in Keil uVision
/----------------------------------------------------------------*/

#include "ecTIM.h"
#include "ecGPIO.h"

/* Timer Configuration */

void TIM_init(TIM_TypeDef* timerx, uint32_t msec){ 
   
// 1. Enable Timer CLOCK
   if(timerx ==TIM1) RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
   else if(timerx == TIM2)  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
   else if(timerx == TIM3)  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
   else if(timerx == TIM4)  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
   else if(timerx == TIM5)  RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
   else if(timerx == TIM9)  RCC->APB2ENR |= RCC_APB2ENR_TIM9EN ;
   else if(timerx == TIM10) RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
   else if(timerx == TIM11) RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
   
// 2. Set CNT period
   TIM_period_ms(timerx,msec); 

   TIM_INT_enable(timerx);
// 3. CNT Direction
   timerx->CR1 &= ~(1<<4);         // UP-COUNT   
   
// 4. Enable Timer Counter
   timerx->CR1 |= TIM_CR1_CEN;   
}


void TIM_period_us(TIM_TypeDef *TIMx, uint32_t usec){   
   // Period usec = 1 to 1000

   //16bit: 0~65000 
   //32bit: 0~4,294,967,295
   //1us(1MHz, ARR=1) to 65msec (ARR=0xFFFF)
   uint32_t prescaler = 84;  //1[MHz]=1[us]
   uint16_t ARRval= (84/(prescaler)*usec);  // 84MHz/1000000 us
   
   TIMx->PSC = prescaler-1;               
   TIMx->ARR = ARRval-1;        
	
}

// PLL : 84Mhz 

void TIM_period_ms(TIM_TypeDef* TIMx, uint32_t msec){ 
   // Period msec = 1 to 6000
   
   //0.1ms(10kHz, ARR=1) to 6.5sec (ARR=0xFFFF) PSC는 0~65000 범위
   uint32_t prescaler = 8400; //10[kHz]=0.1[ms]      // 
   uint16_t ARRval=(84000/(prescaler)*msec);           // 84MHz/1000ms

   TIMx->PSC = prescaler-1;               
   TIMx->ARR = ARRval-1;                     
}


// Update Event Interrupt 
void TIM_INT_init(TIM_TypeDef* timerx, uint32_t msec){
// 1. Initialize Timer   
   TIM_init(timerx,msec);
   
// 2. Enable Update Interrupt
   uint32_t IRQn_reg =0;
	
   if(timerx ==TIM1)       IRQn_reg = TIM1_UP_TIM10_IRQn;
   else if(timerx ==TIM2)  IRQn_reg = TIM2_IRQn;
   else if(timerx ==TIM3)  IRQn_reg = TIM3_IRQn;
   else if(timerx ==TIM4)  IRQn_reg = TIM4_IRQn;
   else if(timerx ==TIM5)  IRQn_reg = TIM5_IRQn;
   else if(timerx ==TIM9)  IRQn_reg = TIM1_BRK_TIM9_IRQn;
   else if(timerx ==TIM10) IRQn_reg = TIM1_UP_TIM10_IRQn;
   else if(timerx ==TIM11) IRQn_reg = TIM1_TRG_COM_TIM11_IRQn;
   
	 // NVIC setting : Enbale & Priority
   NVIC_EnableIRQ(IRQn_reg);            
   NVIC_SetPriority(IRQn_reg,2);
	
}


void TIM_INT_enable(TIM_TypeDef* timerx){
   timerx->DIER |=1<<0;         			 // Enable Timer Update Interrupt      
}

void TIM_INT_disable(TIM_TypeDef* timerx){
   timerx->DIER &= ~(1<<0);            // Disable Timer Update Interrupt      
}

uint32_t is_UIF(TIM_TypeDef *TIMx){
   return ((TIMx->SR &TIM_SR_UIF)==TIM_SR_UIF);
}

void clear_UIF(TIM_TypeDef *TIMx){
   TIMx->SR &= ~TIM_SR_UIF;
}