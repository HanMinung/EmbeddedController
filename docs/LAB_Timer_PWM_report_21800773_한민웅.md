# LAB: Timer & PWM

**Date:** 2022-10-27

**Author/Partner:** Han,Minung / Nam,Hyunwoo

**Github:** https://github.com/HanMinung/EC-muhan-773

**Demo Video:** https://youtu.be/Yls7fqT0Mn8

# Introduction

In this lab, you are required to create a simple program that control a sevo motor with PWM output.



## Requirement

### Hardware

- MCU
  - NUCLEO-F411RE
- Actuator/Others:
  - Actuator/Display
    * RC Servo Motor (SG90)
  - Others
    * breadboard
    * Oscilloscope

### Software

- Keil uVision, CMSIS, EC_HAL library

  

# Problem 1: Create HAL library

## Create HAL library

* **ecTIM.h**

```c
// Timer Period setup
void TIM_init(TIM_TypeDef *TIMx, uint32_t msec);
void TIM_period_us(TIM_TypeDef* TIMx, uint32_t usec);
void TIM_period_ms(TIM_TypeDef* TIMx, uint32_t msec);

// Timer Interrupt setup
void TIM_INT_init(TIM_TypeDef* TIMx, uint32_t msec);
void TIM_INT_enable(TIM_TypeDef* TIMx);
void TIM_INT_disable(TIM_TypeDef* TIMx);

// Timer Interrupt Flag 
uint32_t is_UIF(TIM_TypeDef *TIMx);
void clear_UIF(TIM_TypeDef *TIMx);
```

* **ecPWM.h**

```c
/* PWM STRUCTURE */

typedef struct {
	GPIO_TypeDef *port;
	int pin;
	TIM_TypeDef *timer;
	int ch;
} PWM_t;

/* PWM initialization */
// Default: 84MHz PLL, 1MHz CK_CNT, 50% duty ratio, 1msec period
void PWM_init(PWM_t *pwm, GPIO_TypeDef *port, int pin);

/* PWM PERIOD SETUP */
// allowable range for msec:  1~2,000
void PWM_period_ms(PWM_t *pwm,  uint32_t msec);	

// allowable range for usec:  1~1,000
void PWM_period_us(PWM_t *pwm, uint32_t usec);

/* DUTY RATIO SETUP */
// High Pulse width in msec
void PWM_pulsewidth_ms(PWM_t *pwm, float pulse_width_ms);

// Duty ratio 0~1.0
void PWM_duty(PWM_t *pwm, float duty);

// User-defined functions to implement requirements in this lab
int update_dir(int dir, uint8_t idx);
float update_duty(uint8_t idx);
```



# Problem 2: RC Servo motor

* An RC servo motor is a tiny and light weight motor with high output power. It is used to control rotation angles, approximately 180 degrees (90 degrees in each direction) and commonly applied in RC car, and Small-scaled robots.

* The angle of the motor can be controlled by the pulse width (duty ratio) of PWM signal. The PWM period should be set at **20ms or 50Hz**. Refer to the data sheet of the RC servo motor for detailed specifications.

<img src="https://user-images.githubusercontent.com/38373000/195773601-f0f19e35-0a6f-49af-aa87-574c86bfec62.png" alt="img" style="zoom: 33%;" />



- The button input has to be External Interrupt
- Use Port A Pin 1 as PWM output pin, for TIM2_Ch2.
- Use Timer interrupt of period 500msec.
- The angle of RC servo motor should rotate from 0° to 180° and back 0° at a step of 10° at the rate of 500msec.



## Procedure

1. Connect the RC servo motor to MCU pin (PA1) , VCC and GND
2. Increase the angle of RC servo motor from 0° to 180° with a step of 10° every 500msec. After reaching 180°, decrease the angle back to 0°. Use timer interrupt IRQ.
3. When the button is pressed, it should reset to the angle 0° and start over. (Use EXTI interrupt.)



## Configuration

<img src="https://user-images.githubusercontent.com/99113269/198284477-db1c4032-39b5-41df-9b90-3b464530f18d.png" alt="image" style="zoom: 50%;" />



## Circuit Diagram

<img src="https://user-images.githubusercontent.com/99113269/198309658-97030ef2-e7b7-4b15-a465-8d9201c85ce0.png" alt="image" style="zoom:50%;" />



## Discussion

1. Derive a simple logic to calculate for CRR and ARR values to generate xHz and y% duty ratio of PWM. How can you read the values of input clock frequency and PSC?

​		Duty ratio is defined as follows :

<img src="https://user-images.githubusercontent.com/99113269/198628317-3d778996-3b2f-4893-8287-65476965ad41.png" alt="image" style="zoom: 67%;" />

​		The relationship between system clock and frequency of PWM is as follows :

<img src="https://user-images.githubusercontent.com/99113269/198629862-10c35454-4d12-4b3d-a1e9-97d7f8549c9e.png" alt="image" style="zoom: 50%;" />

​		For high resolution of PWM duty control, suficient ARR number needs to be satisfied.

2. What is the smallest and highest PWM frequency that can be generated for Q1?

​		Duty ratio has a value between 0 and 1. Considering that ARR value is 16 bits, we can determine the range of PWM frequency as follows :

​		<img src="https://user-images.githubusercontent.com/99113269/198825895-a0ee23d8-2683-47b0-8f62-83fab85f1333.png" alt="image" style="zoom:50%;" />

## Code

Code link : https://github.com/HanMinung/EC-muhan-773/blob/main/lab/LAB_PWM_RCmotor.c

* **PWM_RCmotor.c ( main statement )**

```c
// Used variable
PWM_t pwm;
uint8_t idx = 0;
int dir = 1;
float duty = 0.5/20.0;
```

```c
// Setup elements
void setup(void)
{
	TIM_INT_init(TIM3,500);
	RCC_PLL_init();
	SysTick_init();
    
	// EXTI button configuration : priority 0
	EXTI_button(FALL,EC_PU,0);						 
	
    // PWM configuration
	PWM_init(&pwm,GPIOA,PA1,HIGH_SPEED,PUSH_PULL,PULL_UP);
	PWM_period_ms(&pwm,20);
	PWM_duty(&pwm,duty);
}
```

```c
// EXTI button interrupt handler function
void EXTI15_10_IRQHandler(void) { 
    
	if (is_pending_EXTI(BUTTON_PIN)) {
		// If button is pressed, set to initial state
        PWM_duty(&pwm,0.5/20.0);
		idx = 0; dir = 1;
        // cleared by writing '1'
		clear_pending_EXTI(BUTTON_PIN); 			
	}	
}	

// Timer interrupt handler function
void TIM3_IRQHandler(void){
	if((TIM3->SR & TIM_SR_UIF) == TIM_SR_UIF){    
		// update interrupt flag
		if(dir == 1) idx ++;
		else if(dir == -1) idx --;
		
		dir = update_dir(dir,idx);
	  	duty = update_duty(idx);
		PWM_duty(&pwm,duty);
	}

	TIM3->SR &= ~TIM_SR_UIF;                  		
}
```

* user defiend function in **ecPWM.c**

```c
int update_dir(int dir,uint8_t idx){

		if(idx%19 == 0) dir *= -1;
		return dir;
}

float update_duty(uint8_t idx){
	
	return (0.5 + (idx*0.11))/20.0;
}
```



## Results

* Demo video link : https://youtube.com/shorts/Yls7fqT0Mn8?feature=share

[<img src="http://img.youtube.com/vi/Yls7fqT0Mn8/0.jpg" alt="Video Label" style="zoom: 80%;" />](https://youtu.be/Yls7fqT0Mn8)

* Oscilloscope captured image ( used program : openchoice desktop )
  * 20*ms* PWM period 
  * measured from pin PA1 with oscilloscope

<img src="https://user-images.githubusercontent.com/99113269/198303417-c8045640-1cb6-4fa4-8b1c-a8910cd3dd4e.png" alt="image" style="zoom: 50%;" />



# Reference

* https://ykkim.gitbook.io/ec/