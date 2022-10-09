# GPIO Digital InOut 7-segment

**Date:** 2022-10-01

**Author/Partner:** HanMinung/NamHyunwoo

**Github:** https://github.com/HanMinung/EC-muhan-773

**Demo Video:** https://youtu.be/ARiJ3gvf2Yw



# Introduction

​	In this lab, you are required to create a simple program to control a 7-segment display to show a decimal number (0~9) that increases by pressing a push-button.



## Requirement

### Hardware

- MCU
  - NUCLEO-F401RE
- Actuator/Sensor/Others:
  - 7-segment display (5101ASR)
  - Array resistor (330 ohm)
  - breadboard

### Software

- Keil uVision, CMSIS, EC_HAL library



# Problem 1: Connecting 7-Segment

## Procedure

​	We need to consider the connection structure of the 7semgent device and the pin connection of the stm32 board. The detailed connection is specified below.

![img](https://user-images.githubusercontent.com/38373000/192133325-a4844100-ab1c-445b-8832-837c8f988f35.png)



## Connection Diagram

Circuit diagram

![7seg](https://user-images.githubusercontent.com/99113269/193389671-2be58b00-f65b-4175-843c-2120c2fd19aa.png)

image

## Discussion

1. Draw the truth table for the BCD 7-segment decoder with the 4-bit input.

![truth table](https://user-images.githubusercontent.com/99113269/193385924-fb18a04a-e9d4-4598-bd5f-cde11e45937b.png)

2. What are the common cathode and common anode of 7-segment display?

   There are two types of seven segmentsm,  Common-anode type and Common-cathode type. The Common-Anode Type is a seven-segment  in which the Anode of the internal LED is connected to the Common Pin and each of the eight pins of the Cathode is connected. Unlike it, the Common-Cathode Type is a seven-segment in which the cathode of the internal LED is connected to the Common Pin and each of the eight pins of the Anode. In the Common-Anode type, the LED turns on when the VCC is connected to the Common Pin and the GND is connected to each pin. The Common-Cathode type lights up when the GND is connected to the Common Pin and the VCC is connected to each pin.

3. Does the LED of a 7-segment display (common anode) pin turn ON when 'HIGH' is given to the LED pin from the MCU?

   Since 7 segment that we used is common-anode type, 5V voltage was applied to the element. In order to make a voltage difference and flow a current to each LED device, each state must be in a LOW state. In other words, if a specific pin is to be turned on, a LOW value must be given to the pin to cause a voltage difference and become a HIGH value state.



# Problem 2: Display 0~9 with button press

## Configuration

![table](https://user-images.githubusercontent.com/99113269/193394697-d2a015c2-852a-4334-8ed6-7e50a17758f2.png)



## Code

Code Github : https://github.com/HanMinung/EC-muhan-773/blob/main/lab/LAB_GPIO_7segment.c

* I defined a function called 'segment_init' to reduce redundant code while setting the above configurations.

* In addition, a function called 'sevensegment_decoder' that determines the next number by taking the remaining number divided by the number of times the button is pressed by 10 as a factor.

* Those functions are defined and used in the code like below : 

  ```c
  /* LAB_GPIO_7segment.c : main statement */
  
  void setup(void);
  	
  int main(void) { 
  	// Initialiization --------------------------------------------------------
  	setup();
  	uint8_t cnt = 0;
  	// Inifinite Loop ----------------------------------------------------------
  	while(1){
  		
  		if(GPIO_read(GPIOC,BUTTON_PIN) == 0) {	
  			cnt ++;
  			sevensegment_decoder(cnt%10);			// user-defined function
  			}
  		delay_ms(50);								// software debouncing
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
  
  
  -------------------------------------------------------------------------------------------------------------------
  /* ecGPIO.c */
  // user defined function used in main statement
      
  unsigned int state[10][8]={
  	// order : PA5 |PA6 |PA7 |PB6 |PC7 |PA9 |PA8 |PB10
  	// order :  a  | b  | c  | d  | e  | f  | g	 | dp
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
  										
  };
  
  void segment_init(void){
  		
  	GPIO_init(GPIOA, LED_PA5, OUTPUT);    // calls RCC_GPIOA_enable()	
  	GPIO_otype(GPIOA, LED_PA5, PUSH_PULL);
  	GPIO_pupd(GPIOA, LED_PA5,NO_PULL_UP);
  	GPIO_ospeed(GPIOA, LED_PA5, MEDIUM_SPEED);
  	
  	GPIO_init(GPIOA, LED_PA6, OUTPUT);    // calls RCC_GPIOA_enable()	
  	GPIO_otype(GPIOA, LED_PA6, PUSH_PULL);
  	GPIO_pupd(GPIOA, LED_PA6,NO_PULL_UP);
  	GPIO_ospeed(GPIOA, LED_PA6, MEDIUM_SPEED);
  
  	GPIO_init(GPIOA, LED_PA7, OUTPUT);    // calls RCC_GPIOA_enable()	
  	GPIO_otype(GPIOA, LED_PA7, PUSH_PULL);
  	GPIO_pupd(GPIOA, LED_PA7,NO_PULL_UP);
  	GPIO_ospeed(GPIOA, LED_PA7, MEDIUM_SPEED);
  	
  	GPIO_init(GPIOB, LED_PB6, OUTPUT);    // calls RCC_GPIOB_enable()	
  	GPIO_otype(GPIOB, LED_PB6, PUSH_PULL);
  	GPIO_pupd(GPIOB, LED_PB6,NO_PULL_UP);
  	GPIO_ospeed(GPIOB, LED_PB6, MEDIUM_SPEED);
  
  	GPIO_init(GPIOC, LED_PC7, OUTPUT);    // calls RCC_GPIOC_enable()	
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
  	
  	GPIO_init(GPIOB, LED_PB10, OUTPUT);    // calls RCC_GPIOB_enable()	
  	GPIO_otype(GPIOB, LED_PB10, PUSH_PULL);
  	GPIO_pupd(GPIOB, LED_PB10,NO_PULL_UP);
  	GPIO_ospeed(GPIOB, LED_PB10, MEDIUM_SPEED);
  	// Define initial state : show 0
  	GPIO_write(GPIOA, LED_PA5, LOW);		// led a
  	GPIO_write(GPIOA, LED_PA6, LOW);		// led b
  	GPIO_write(GPIOA, LED_PA7, LOW);		// led c
  	GPIO_write(GPIOA, LED_PB6, LOW);		// led d		
  	GPIO_write(GPIOC, LED_PC7, LOW);		// led e
  	GPIO_write(GPIOA, LED_PA9, LOW);		// led f
  	GPIO_write(GPIOA, LED_PA8, HIGH);		// led g
  	GPIO_write(GPIOB, LED_PB10, LOW);		// led dp
  
  }
  
  
  void sevensegment_decoder(uint8_t num){
  		GPIO_write(GPIOA, LED_PA5, state[num][0]);		// led a
  		GPIO_write(GPIOA, LED_PA6, state[num][1]);		// led b
  		GPIO_write(GPIOA, LED_PA7, state[num][2]);		// led c
  		GPIO_write(GPIOB, LED_PB6, state[num][3]);		// led d		
  		GPIO_write(GPIOC, LED_PC7, state[num][4]);		// led e
  		GPIO_write(GPIOA, LED_PA9, state[num][5]);		// led f
  		GPIO_write(GPIOA, LED_PA8, state[num][6]);		// led g
  		GPIO_write(GPIOB, LED_PB10,state[num][7]);		// led dp
  }
  ```

​	

## Results

Demo video :

 ![IU(아이유) _ Into the I-LAND](http://img.youtube.com/vi/ARiJ3gvf2Yw/0.jpg)](https://www.youtube.com/watch?v=ARiJ3gvf2Yw) 

Demo video link : https://www.youtube.com/watch?v=ARiJ3gvf2Yw

# Reference

* https://ykkim.gitbook.io/ec/
* https://kocoafab.cc/tutorial/view/351