/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2021-8-12 by YKKIM  	
* @brief   Embedded Controller:  LAB Digital In/Out
*					 - Toggle LED LD2 by Button B1  pressing
* 
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecSysTick.h"

void setup(void);
	
int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	uint8_t cnt = 0;
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		
		if(GPIO_read(GPIOC,BUTTON_PIN) == 0) {
			
			cnt ++;
			
			sevensegment_decoder(cnt%10);
			
			}
		
		delay_ms(50);
		
	}
	
}


// Initialiization 
void setup(void)
{
	
	RCC_HSI_init();	
	SysTick_init();
	
	segment_init();
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);

}
