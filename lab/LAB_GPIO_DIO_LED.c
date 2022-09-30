/**
******************************************************************************
* @author	Han,Minung
* @Mod		2022.09.29
* @brief	Embedded Controller:  LAB Digital In/Out
*					 - Toggle LED LD2 by Button B1 pressing
* 
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"
#include "ecSysTick.h"

#define LED_PIN 	5
#define BUTTON_PIN 13

void setup(void);
	
int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		
		if(GPIO_read(GPIOC,BUTTON_PIN) == 0){
			
			delay_ms(30);
			bittoggle(GPIOA,LED_PIN);

		}
	
	}
	
}
	

// Initialiization 
void setup(void){
	
	RCC_HSI_init();
	SysTick_init();
	
	// led
	GPIO_init(GPIOA, LED_PIN, OUTPUT);    // calls RCC_GPIOA_enable()
	GPIO_pupd(GPIOA, LED_PIN, EC_PU);			// pull up
	GPIO_otype(GPIOA, LED_PIN, OPEN_DRAIN);				// open drain
	GPIO_write(GPIOA, LED_PIN, LOW);			
	GPIO_ospeed(GPIOA, LED_PIN, MEDIUM_SPEED);				// Medium speed
	
	// button
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // Calls RCC_GPIOC_enable()
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);	// Pull up
	GPIO_otype(GPIOC, BUTTON_PIN, 1);

}