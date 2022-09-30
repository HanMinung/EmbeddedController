/**
******************************************************************************
* @author	Han,Minung
* @Mod		09.28.2022
* @brief	Embedded Controller:  LAB Digital In/Out
*					 - Toggle LED LD2 by Button B1 pressing
* 
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"
#include "ecSysTick.h"

#define LED_PA5 	5			// PA5
#define LED_PA6 	6			// PA6
#define LED_PA7 	7			// PA7
#define LED_PB6 	6			// PB6
#define BUTTON_PIN 13

void setup(void);
	
int main(void) { 
	
	
	// Initialiization --------------------------------------------------------
	setup();
	int flag = 0;
	int cnt = 0;
	
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		
		if(GPIO_read(GPIOC,BUTTON_PIN) == 0)	{
			cnt ++;
			flag = cnt%4;
		
			switch(flag){
				
				case 1 :
					
					bittoggle(GPIOA,LED_PA5);
					if(cnt > 4){
						bittoggle(GPIOB,LED_PB6);
					}
					break;
				
				case 2 :
					bittoggle(GPIOA,LED_PA5);
					bittoggle(GPIOA,LED_PA6);
					break;
				
				case 3 :
					bittoggle(GPIOA,LED_PA6);
					bittoggle(GPIOA,LED_PA7);
					break;
				
				default : 
					bittoggle(GPIOA,LED_PA7);	
					bittoggle(GPIOB,LED_PB6);
				}

		} 
			delay_ms(60);
		
	}
	
}


// Initialiization 
void setup(void)
{
	
	RCC_HSI_init();	
	SysTick_init();
	
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);
	
	GPIO_init(GPIOA, LED_PA5, OUTPUT);    // calls RCC_GPIOA_enable()
	GPIO_pupd(GPIOA, LED_PA5, EC_PU);
	GPIO_otype(GPIOA, LED_PA5, 0);
	GPIO_write(GPIOA, LED_PA5, LOW);
	GPIO_ospeed(GPIOA , LED_PA5, MEDIUM_SPEED);
	
	GPIO_init(GPIOA, LED_PA6, OUTPUT);    // calls RCC_GPIOA_enable()
	GPIO_pupd(GPIOA, LED_PA6, EC_PU);
	GPIO_otype(GPIOA, LED_PA6, 0);
	GPIO_write(GPIOA, LED_PA6, LOW);
	GPIO_ospeed(GPIOA , LED_PA6, MEDIUM_SPEED);

	GPIO_init(GPIOA, LED_PA7, OUTPUT);    // calls RCC_GPIOA_enable()
	GPIO_pupd(GPIOA, LED_PA7, EC_PU);
	GPIO_otype(GPIOA, LED_PA7, 0);
	GPIO_write(GPIOA, LED_PA7, LOW);
	GPIO_ospeed(GPIOA , LED_PA7, MEDIUM_SPEED);

	GPIO_init(GPIOB, LED_PB6, OUTPUT);    // calls RCC_GPIOA_enable()
	GPIO_pupd(GPIOB, LED_PB6, EC_PU);
	GPIO_otype(GPIOB, LED_PB6, 0);
	GPIO_write(GPIOB, LED_PB6, LOW);
	GPIO_ospeed(GPIOA , LED_PB6, MEDIUM_SPEED);
					  
}