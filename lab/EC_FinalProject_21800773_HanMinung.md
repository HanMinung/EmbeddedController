# Embedded Controller Final project : Smart factory

**Date:** 2022.12.14

**Author/Partner:** HanMinung / GaSeungho 

**Github:** https://github.com/HanMinung/EC-muhan-773

**Demo Video:** https://www.youtube.com/watch?v=pOZleyquSps



## Introduction

​		Recently, many people were satisfied due to unexpected accidents at labor sites such as factories. Therefore, topic about 'smart factory' was selected feeling the necessity of an automated process system related to safety and to implement it. In particular, this project aims to establish a system to prevent safety accidents related to stirrers mainly used in industrial sites. We also aim to consider and implement how safety accidents can be prevented in automated transportation systems using RC car module. The ultimate goal is to establish an industrial accident prevention system through communication with several modules, including Bluetooth and zigbee.



## Requirement

### Hardware

- MCU

  - NUCLEO-F401RE x3

- Actuator/Sensor/Others:

  - ​	Actuator list :

    <img src="https://user-images.githubusercontent.com/99113269/207205321-c4cfc061-2b7f-4127-aa75-98b3ac495f20.png" alt="image" style="zoom:50%;" />
  
  - ​    Sensor list :

    <img src="https://user-images.githubusercontent.com/99113269/207205144-9190d788-ac9d-4bf9-95ff-2dcb1e09ef06.png" alt="image" style="zoom:50%;" />
    
  - ​     Others
  
    <img src="https://user-images.githubusercontent.com/99113269/207205827-ef64ed02-f34a-4c73-a1ff-cb0293df5cc4.png" alt="image" style="zoom:56%;" />
  
    

### Software

- Keil uVision, CMSIS, EC_HAL library



## Design requirements

* Stirrer ( MCU1 )

  * Essentially, in preparation for dangerous situations, when a user presses key 's', an 'ALLSTOP' commang is given and the plant is designed to stop operation.

  * If user press button, the cover of stirrer is opened. And designed to be closed when pressed again.

  * In the state with the cover opened, motor driving in the stirrer is stopped, and a white LED is turned on simultaneously.

  * In addition, the existing LED automatically turns on, and the motor operates properly when the cover is closed.

  * If the temperature is above the proper temperature, LED on the stirrer is turned on. ( critical temperature : 80 )

  * Flow chart with regard to operation of stirrer is as follow :

    <img src="https://user-images.githubusercontent.com/99113269/205848766-5ab09b07-5ebd-4d34-92a6-f433cc494320.png" alt="image" style="zoom:45%;" />

    ​                                      

* Blind spot warning system ( MCU2 )

  * When a car passes through a designated area, the LED on the spot turns on and informs the people around it the location of the car.

  * Accurate information about the above is found using a piezoelectric sensor.

  * When the car get close to the blind spot, buzzer sensor should inform workers through sound.

    <img src="https://user-images.githubusercontent.com/99113269/206866692-c2777835-a10f-4d33-89d2-d6c76861233d.png" alt="image" style="zoom:45%;" />


* Autonomous transportation system ( MCU3 )

  * When supplies are loaded on the car, the car recognizes and starts automatically with LED turning on simultaneously.

  * After that, the car performs line tracing and drives a predetermined path.

  * In preparation of emergency circumstance, car has to stop when ESTOP flag comes from PC1, and run again when the resume flag comes from it.

  * If there exists an obstacle ahead, the car has to be stopped.

  * Flow char with regard to operation of transportation vehicle is as follow : 

    <img src="https://user-images.githubusercontent.com/99113269/207206177-cd69cc29-38ed-4bfa-8a06-434a43ffdf47.png" alt="image" style="zoom:45%;" />

  

* Factory module

  * For testing system we constructed, we made factory module with some materials.

  * The module is like below :

    <img src="https://user-images.githubusercontent.com/99113269/206871575-340b9f56-2fd0-4718-9ac4-1b422d046652.png" alt="image" style="zoom: 40%;" />



## Communication diagram between MCUs

<img src="https://user-images.githubusercontent.com/99113269/206871012-d1a4875e-a90e-4fb0-afc7-fb12643eae09.png" alt="image" style="zoom: 33%;" />





## Configuration lists

### MCU1

* MCU1 is used to control the overall operation of the industrial stirrer.
* The configuration of various sensors and communication connected to the MCU1 is as follows :

------------

* Servo motor & LED for stirrer , temperature display

<img src="https://user-images.githubusercontent.com/99113269/206865461-68f5af1c-f45b-4339-a52d-608964f648ff.png" alt="image" style="zoom:45%;" />

* Ultrasonic sensor (HC-SR04) , Used timer list

  <img src="https://user-images.githubusercontent.com/99113269/206269922-88055239-549c-4eee-8a08-9bf74bf96c76.png" alt="image" style="zoom: 59%;" />

* Bluetooth communication ( PC & MCU1 ), Wired communication ( MCU1 & MCU2 )

  <img src="https://user-images.githubusercontent.com/99113269/208109138-e81003d2-8542-439f-b07e-5a065f14acad.png" alt="image" style="zoom: 78%;" />



### MCU2

* MCU2 serves to provide warnings for preventing dangerous situations in factory of blind spots and information about autonomous vehicles.

* Related sensors and configuration lists are as follows : 

--------------

* LEDs, Buzzer sensor

  ![image](https://user-images.githubusercontent.com/99113269/206830448-b853da63-3fce-4ad0-be01-aaefe8bc60f6.png)

* Piezoelectrical sensor : read with digital value

  <img src="https://user-images.githubusercontent.com/99113269/206830501-d9412e86-193a-4c01-968f-afdb513a8e62.png" alt="image" style="zoom:50%;" />
  
* Zigbee wireless communication with MCU2

  <img src="https://user-images.githubusercontent.com/99113269/208108282-2933d7e9-8213-4061-bc57-ba36879f5f5a.png" alt="image" style="zoom:70%;" />



### MCU3

* The MCU3 is used to control a I/O of sensor values and a motor of a vehicle performing an autonomous transportation system.
* Related sensors and configuration lists are as follows :

-------

* Ultrasonic sensor , IR sensors

<img src="https://user-images.githubusercontent.com/99113269/205881612-7ebbd9c4-adc5-451b-99fc-708e8f3aee44.png" alt="image" style="zoom: 67%;" />

* DC motor x2

  <img src="https://user-images.githubusercontent.com/99113269/205885534-00d0fe0a-e0e4-441c-9262-b9bcb437cd67.png" alt="image" style="zoom: 67%;" />

* Zigbee communication

  <img src="https://user-images.githubusercontent.com/99113269/208110890-d5f9e1d0-8789-4435-8739-98b2927c0e2c.png" alt="image" style="zoom: 50%;" />

  



## Circuit connection Diagram

* Circuit diagram : MCU1

<img src="https://user-images.githubusercontent.com/99113269/206830862-01865fda-8820-4fe8-b113-1d6266b15cf8.png" alt="image" style="zoom:40%;" />

* Circuit diagram : MCU2

<img src="https://user-images.githubusercontent.com/99113269/206895167-e032f41b-bb5e-4396-8309-9ffd748038bb.png" alt="image" style="zoom:40%;" />

* Wired communication between MCU 1 and MCU2

  <img src="https://user-images.githubusercontent.com/99113269/206866400-898cd4e7-f123-4a0f-9f85-5daf21415b8f.png" alt="image" style="zoom: 33%;" />

* Circuit diagram : MCU3

<img src="https://user-images.githubusercontent.com/99113269/208245065-d5ebd4a7-ff39-4bf8-ad12-5cd93d5625da.png" alt="image" style="zoom:60%;" />



## Firmware programming

### MCU1

* We added one more to the existing header for efficient code construction with regard to MCU1 related processes.
  * FINAL_proj.c	&	FINAL_proj.h 
  * FINAL_variable.h
* We used existing HAL libraries for rest of the header files.

* **FINAL_variable.h**

  ```c
  // BLUETOOTH COMMUNICATION ------------------
  #define END_CHAR 13
  #define A 0
  #define B 1
  
  uint8_t pcData = 0;
  uint8_t mcuData = 0;
  
  // STIRRER ----------------------------------
  uint8_t LED_stir_state = 0;
  uint8_t Stir_pre_flag = 0;
  uint8_t Stir_dist_flag = 0;
  
  // SERVO MOTOR : INTERRUPT PERIOD -----------
  uint32_t SERVO_tim_period = 500;
  uint8_t servo_idx = 0;
  int servo_dir = 1;
  float servo_stir_duty = 0.5/20.0;
  float servo_opc_duty = 0.5/20.0;
  float servo_opc_int = 0.0;
  
  // Stirrer open & close flag ---------------
  uint8_t OPC_flag = 0;
  uint32_t BUTTON_cnt = 0;
  uint32_t BUTTON_prev = 0;
  uint8_t BUTTON_flag = 0;
  
  //TEMPERATURE SENSOR -----------------------
  float TEMP_out = 0.0;
  
  // VARIABLES : ULTRASONIC SENSOR -----------
  uint32_t ovf_cnt = 0;
  float distance = 0;
  float timeInterval = 0;
  float time1 = 0;
  float time2 = 0;
  float PulseWidth = 10;
  
  // VARIABLE : ALL STOP FALG
  uint8_t btData = 0;
  uint8_t ESTOP_flag = 0;
  
  #endif
  ```
  
* **FINAL_proj.h**

  ```c
  //----------------------			VARIABLES			 ----------------------
  extern unsigned int LED_stir_struct[3][3];
  extern unsigned int LED_dist_struct[3][3];
  //----------------------		SETUP FACTORS		 ----------------------
  void FINAL_setup(void);
  void RCmotor_setup(void);
  void LED_stir_setup(void);
  void LED_dist_setup(void);
  
  // ---------------------		FUNCTION DEF		-----------------------
  void Stirrer_LED_state(uint16_t LED_stir_state);
  uint8_t Pre_process(float dist);
  uint16_t Stirrer_state(uint8_t _flag, uint8_t _preflag);
  float STIR_OPC_control(uint8_t _flag);
  void TEMP_det(float temp_out);
  uint8_t BUTTON_stir_det(uint32_t _currCNT,uint32_t _prevCNT, uint8_t _flag);
  void LED_estop(void);
  ```

- **FINAL_proj.c**

  * structure definition
  
  ```c
  unsigned int LED_stir_struct[3][3] = {
  	
  	{1,0,0},		//	stable state flag 
  	{0,1,0},		//	motion detected flag 
  	{0,0,1}			//	all stop flag
  };
  
  unsigned int LED_dist_struct[3][3] = {
  	
  	{1,0,0},
  	{0,1,0},
  	{0,0,1}
  };
  
  ```
  
  * Setup process
  
  ```c
  void FINAL_setup(void){
  	
  	// Servo motor setup -------------------------------------------
  	RCC_PLL_init();
  	SysTick_init(1);
  	
  	// LED setup ---------------------------------------------------
  	LED_stir_setup();
  	LED_dist_setup();
  	
  	// BLUETOOTH communication setup : PA11 (TX) , PA12 (RX)
  	UART2_init();
  	USART_init(USART2, 9600);                          		// PC <--> mcu.
  	USART_begin(USART6, GPIOA, 11, GPIOA, 12, 9600);    	// Bluetooth : PA11 (TX) PA12 (RX)
  	USART_begin(USART1, GPIOB, 6, GPIOB, 3, 9600);			// TX RX wired communicaion
  
  	// EXTI init
  	EXTI_init(GPIOC,BUTTON_PIN,FALL,1);		
  	GPIO_init(GPIOC, BUTTON_PIN, INPUT);			
  	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);			
  	
  	// ADC init : FOR TEMPERATURE SENSOR
  	ADC_init(GPIOB, 1, SW);
    	ADC_continue(CONT);
  	ADC_start();
  }
  
  void LED_stir_setup(){
  	// STIRRER - STATE LED
  	// PA8 , PB10 , PB4
  	indiv_init(GPIOB,0,OUTPUT,PUSH_PULL,NO_PUPD,MEDIUM_SPEED);
  	indiv_init(GPIOC,1,OUTPUT,PUSH_PULL,NO_PUPD,MEDIUM_SPEED);
  	indiv_init(GPIOC,0,OUTPUT,PUSH_PULL,NO_PUPD,MEDIUM_SPEED);
  
  	// TEMPERATURE
  	indiv_init(GPIOC,10,OUTPUT,PUSH_PULL,NO_PUPD,MEDIUM_SPEED);
  }
  
  
  void LED_dist_setup(){
  	// DISTANCE - STATE LED
  	// PB5 , PB3 , PA10
  	indiv_init(GPIOB,5,OUTPUT,PUSH_PULL,NO_PUPD,MEDIUM_SPEED);
  	indiv_init(GPIOB,3,OUTPUT,PUSH_PULL,NO_PUPD,MEDIUM_SPEED);
  	indiv_init(GPIOA,10,OUTPUT,PUSH_PULL,NO_PUPD,MEDIUM_SPEED);
  }
  ```
  
  * Additional function deifnition
  
  ```c
  void Stirrer_LED_state(uint16_t LED_stir_state){
  
  	GPIO_write(GPIOB, 0, LED_stir_struct[LED_stir_state][0]);		// LED : GREEN
  	GPIO_write(GPIOC, 1, LED_stir_struct[LED_stir_state][1]);		// LED : WHITE
  	GPIO_write(GPIOC, 0, LED_stir_struct[LED_stir_state][2]);		// LED : RED
  }
  
  uint8_t Pre_process(float dist){
  
  	uint8_t _flag = 0;
  	
  	if(dist > 12.0 && dist < 17.0)		_flag = 0;		// PROPER value : closed
  	else if (dist >= 17 && dist < 60) 	_flag = 1;		// OPEN state
  	else if(dist < 2 || dist > 400)		_flag = 2;		// ERROR value : exception
  	
  	return _flag;
  }
  
  
  uint16_t Stirrer_state(uint8_t _flag, uint8_t _preflag){
  
  	// ALGORITHM
  	
  	switch(_flag){
  	
  		case 0 : TIM_INT_enable(TIM5);			break;
  		case 1 : TIM_INT_disable(TIM5);			break;
  		case 2 : delay_ms(100);					break;
  		default : break;
  	}
  	
  	if(_flag != 2)	return _flag;
  	else 			return _preflag;
  }
  
  
  float STIR_OPC_control(uint8_t _flag){
  
  	if(_flag == 1)			return 0.088;
  	else if(_flag == 0) 	return 0.00;
  }
  
  
  void TEMP_det(float temp_out){
  	
  	float TEMP_act = 0.0;
  	
  	TEMP_act = (float)temp_out/10 + 50;
  	
  	if(TEMP_act > 80)	GPIO_write(GPIOC,10,HIGH);
  	else GPIO_write(GPIOC,10,LOW);
  	
  	printf("Actual temp : %f\r\n",TEMP_act);
  }
  
  uint8_t	BUTTON_stir_det(uint32_t _currCNT, uint32_t _prevCNT, uint8_t _flag){
  	
  	if(_currCNT - _prevCNT >= 1 ) _flag ^= 1;
  	
  	return _flag;
  }
  
  void LED_estop(void){
  
  	GPIO_write(GPIOB, 0, LOW);
  	GPIO_write(GPIOC, 1, LOW);
  	bittoggle(GPIOC,0);
  }
  ```
  
- Main statement : while loop

  ```c
  if(ESTOP_flag == 0){
  			
  			delay_ms(500);
  			// LED state update in stirrer
  			Stirrer_LED_state(LED_stir_state);
  			// Algorithm to complement error values 
  			Stir_dist_flag = Pre_process(distance);		
  			//LED state determination
  			LED_stir_state = Stirrer_state(Stir_dist_flag, Stir_pre_flag);	
  
  			Stir_pre_flag = LED_stir_state;
  			// FOR OPENING AND CLOSING : debouncing problem
  			OPC_flag = BUTTON_stir_det(BUTTON_cnt,BUTTON_prev, OPC_flag);
  			
  			BUTTON_prev = BUTTON_cnt;
  			
  			servo_opc_int = STIR_OPC_control(OPC_flag);
  			
  			PWM_duty(&opc,servo_opc_int);
  		  
  			// TEMPERATURE SENSOR			
  			TEMP_det(TEMP_out);
  		}
  		
  		else if(ESTOP_flag){
  			
  			TIM_INT_disable(TIM5);
  			LED_estop();
  			delay_ms(500);
  		}
  ```

  



------

### mcu2

* main code statement : 

  * main code

  ```c
  int main(void){
  	
  	FINAL_setup();	
  	
  	while(1){
  		
  		if(ESTOP_flag == 0){
  			
  			SECTION_det();
  			PIEZO_det(SEC_flag);
  			PWM_duty(&pwm,BLIND_duty);
  		}
  		
  		else if(ESTOP_flag){
  		
  			GPIO_write(GPIOC,8,HIGH);
  			GPIO_write(GPIOC,6,HIGH);
  			GPIO_write(GPIOC,5,HIGH);
  		}
  	}
  }
  ```
  
  * setup elements
  
  ```c
  // FUNCTION DECLARATION --------------------------------------
  void FINAL_setup(void);
  void MULTLED_init(void);
  void PIEZO_init(void);
  void SECTION_det(void);
  void PIEZO_det(uint8_t _flag);
  
   
  // FUNCTION DEFINITION --------------------------------------
  void FINAL_setup(void){
  
  	RCC_PLL_init();
  	USART_init(USART2,9600);
           
  	USART_begin(USART1, GPIOB, 6, GPIOB, 3, 9600);    	// WIRED COMMUNICATION
   	USART_begin(USART6, GPIOA, 11, GPIOA, 12, 9600);	// ZIGBEE communication
  	
  	MULTLED_init();
  	PIEZO_init();	
  }
  
  
  void MULTLED_init(void){
  
  	indiv_init(GPIOC,8,OUTPUT,PUSH_PULL,NO_PUPD,MEDIUM_SPEED);
  	indiv_init(GPIOC,6,OUTPUT,PUSH_PULL,NO_PUPD,MEDIUM_SPEED);		
  	indiv_init(GPIOC,5,OUTPUT,PUSH_PULL,NO_PUPD,MEDIUM_SPEED);
  	indiv_init(GPIOA,0,AF,PUSH_PULL,NO_PUPD,MEDIUM_SPEED);
  
  	// PWM INITIATION
  	PWM_init(&pwm,GPIOA,0);
  	PWM_period_ms(&pwm,200); 
  	
  	// INITIALIZATION
  	GPIO_write(GPIOC,8,LOW);
  	GPIO_write(GPIOC,6,LOW);
  	GPIO_write(GPIOC,5,LOW);
  }
  
  
  void PIEZO_init(void){
  	
  	GPIO_init(GPIOC,9,INPUT);	GPIO_pupd(GPIOC, 9,PULL_DOWN);
  	GPIO_init(GPIOB,8,INPUT);	GPIO_pupd(GPIOB, 8,PULL_DOWN);
  	GPIO_init(GPIOB,9,INPUT);	GPIO_pupd(GPIOB, 9,PULL_DOWN);
  }
  ```
  
  * User-defined function
  
  ```c
  void SECTION_det(void){
  
  	uint8_t PIEZO_1 = GPIO_read(GPIOC,9);
  	uint8_t PIEZO_2 = GPIO_read(GPIOB,8);
  	uint8_t PIEZO_3 = GPIO_read(GPIOB,9);
  	
  	if(PIEZO_1 == HIGH)				SEC_flag = flag_sec1;
  	else if(PIEZO_2 == HIGH)		SEC_flag = flag_sec2;
  	else if(PIEZO_3 == HIGH)		SEC_flag = flag_sec3;
  }
  
  void PIEZO_det(uint8_t _flag){
  
  	switch(_flag){
  	
  		case 0 : {
  			GPIO_write(GPIOC,8,LOW);
  			GPIO_write(GPIOC,6,LOW);
  			GPIO_write(GPIOC,5,LOW);
  			BLIND_duty = 0.0;
  		} break;
  			
  		case 1 : {
  			GPIO_write(GPIOC,8,HIGH);
  			GPIO_write(GPIOC,6,LOW);
  			GPIO_write(GPIOC,5,LOW);	
  			BLIND_duty = 0.10;			
  		} break;
  		
  		case 2 :{
  			GPIO_write(GPIOC,8,LOW);
  			GPIO_write(GPIOC,6,HIGH);
  			GPIO_write(GPIOC,5,LOW);			
  			BLIND_duty = 0.50;
  		} break;
  			
  		case 3 :{
  			GPIO_write(GPIOC,8,LOW);
  			GPIO_write(GPIOC,6,LOW);
  			GPIO_write(GPIOC,5,HIGH);	
  			BLIND_duty = 0.80;
  		} break;
  		
  		defualt : break;
  	}
  }
  
  
  void USART1_IRQHandler(){  
      
     if(is_USART_RXNE(USART1)){
        mcu2Data = USART_getc(USART1);
        // Transfer flag that if from Wired communication to MCU3 with zigbee 
        USART_write(USART6, &mcu2Data, 1);				
        
  	  if(mcu2Data == 's') 		ESTOP_flag = 1;
  	  else if(mcu2Data == 'r') 	ESTOP_flag = 0;
   
     }
  }
  ```
  
  

-----------

### MCU3

* main code statement : 

  * User defined function

  ```c
  // Function declaration
  void setup(void);
  void motor_stop(void);
  void motor_right(void);
  void motor_left(void);
  void motor_straight(void);
  ```

  ```c
  // Initialiization 
  void setup(void){   
     RCC_PLL_init();                 
     UART2_init();
     SysTick_init();
     choco_init();
     
     GPIO_init(GPIOA, PA5, OUTPUT);
     GPIO_otype(GPIOA,PA5, 0UL);       
     GPIO_pupd(GPIOA,PA5, 0UL);       
     GPIO_ospeed(GPIOA,PA5, 1UL);      
      
     GPIO_init(GPIOA, 2, OUTPUT);
     GPIO_otype(GPIOA,2, 0UL);       // 0:Push-Pull
     GPIO_pupd(GPIOA,2, 0UL);        // 00: no Pull-up/ pull-down 
     GPIO_ospeed(GPIOA,2, 1UL);      // 01:Medium Speed
     GPIO_write(GPIOA,2,HIGH);
     
     USART_init(USART2, 9600);
     USART_begin(USART6, GPIOA, 11, GPIOA, 12, 9600);    // PA9: TXD, PA10: RXD
     PWM_init(&dcPwm[A], dcPwmPin[A].port, dcPwmPin[A].pin);
     PWM_init(&dcPwm[B], dcPwmPin[B].port, dcPwmPin[B].pin);
     
     PWM_period_ms(&dcPwm[A], 1); //TIM2 1ms
     PWM_period_ms(&dcPwm[B], 1); //TIM2 1ms
  
     for (int i = 0; i < 2; i++){
       GPIO_init(dcDirPin[i].port, dcDirPin[i].pin, OUTPUT);
       GPIO_pupd(dcDirPin[i].port, dcDirPin[i].pin, PD);
       GPIO_otype(dcDirPin[i].port, dcDirPin[i].pin, OUTPP);
       GPIO_ospeed(dcDirPin[i].port, dcDirPin[i].pin, HIGHSPEED);
     }
     
     GPIO_write(dcDirPin[A].port, dcDirPin[A].pin, LOW);
     GPIO_write(dcDirPin[B].port, dcDirPin[B].pin, LOW);
     // PWM configuration ---------------------------------------------------------------------------------   
     PWM_t trig;                                
     PWM_init(&trig,GPIOA,6);               
     PWM_period_us(&trig, 50000);       
     PWM_pulsewidth_us(&trig, 10);      
     
     // Input Capture configuration -----------------------------------------------------------------------   
     IC_t echo;                    
     ICAP_init(&echo,GPIOB,6);   
     ICAP_counter_us(&echo, 10);     
     ICAP_setup(&echo, 1, IC_RISE);     
     ICAP_setup(&echo, 2, IC_FALL);     
     
     TIM_INT_enable(TIM4);        
     ADC_init(GPIOB, 0, TRGO);
     ADC_init(GPIOB, 1, TRGO);
     ADC_sequence(2, seqCHn);
     ADC_start();
  }
  
  // Motor stop function when stop flag is applied
  void motor_stop(void){
     PWM_duty(&dcPwm[A], 0.0);
     PWM_duty(&dcPwm[B], 0.0);
     GPIO_write(dcDirPin[A].port, dcDirPin[A].pin, LOW);
     GPIO_write(dcDirPin[B].port, dcDirPin[B].pin, LOW);
  }
  
  void motor_right(void){
     PWM_duty(&dcPwm[A], 0.1 );
     PWM_duty(&dcPwm[B], 0.9);
     GPIO_write(dcDirPin[A].port, dcDirPin[A].pin, LOW);
     GPIO_write(dcDirPin[B].port, dcDirPin[B].pin, LOW);
  }
  
  void motor_left(void){
     PWM_duty(&dcPwm[A], 0.9);
     PWM_duty(&dcPwm[B], 0.1 );
     GPIO_write(dcDirPin[A].port, dcDirPin[A].pin, LOW);
     GPIO_write(dcDirPin[B].port, dcDirPin[B].pin, LOW);
  }
  void motor_straight(void){
     PWM_duty(&dcPwm[A], 0.7);
     PWM_duty(&dcPwm[B], 0.7);
     GPIO_write(dcDirPin[A].port, dcDirPin[A].pin, LOW);
     GPIO_write(dcDirPin[B].port, dcDirPin[B].pin, LOW);
  }
  ```

  ```c
  int main(void) { 
  
     // Initialiization --------------------------------------------------------
     setup();
     
     // Inifinite Loop ----------------------------------------------------------
     while(1){
        
        distance = (float) timeInterval / 58.0;
     
        if(GPIO_read(GPIOC,10) != LOW){
              PIEZO_flag = 1;
         }
         
      switch(allstopflag){
              
           case 0 :
          	if(PIEZO_flag == 1){
                   GPIO_write(GPIOA,5,HIGH);
                  
              if(distance < 10.0) motor_stop();
              else{
              
                 if (IR1 > 1000){
                    if(IR2 >1000) printf("Error\r\n");
                    else motor_right();    
                 }
                 
                    else if (IR2 > 1000){
                    if(IR1 >1000) printf("Error\r\n");
                    else motor_left();    
                 }
                 
                    else{
                    if(IR2 >1000 && IR1 >1000) printf("error");
                    else motor_straight();
                 }
              }
           }
           else motor_stop(); 
               break;
                     
          case 1:
               motor_stop();
               break;
           default:
               motor_straight();
        }
        delay_ms(15);
    }
  }
  ```

  * Handler list

  ```c
  void USART6_IRQHandler(){      
      
     if(is_USART_RXNE(USART6)){
        recvChar = USART_getc(USART6);
        printf("%c", recvChar);                      
        if(recvChar == 's') {
           GPIO_write(GPIOA,5,HIGH);
           allstopflag = 1;
        }
         
        if(recvChar == 'r') {
           GPIO_write(GPIOA,5,LOW);
           allstopflag = 0;
        }
        
        if(recvChar == END_CHAR) {
           bReceive = 1;
           idx = 0;
        }
         
        else{
           if(idx > MAX_BUF){
              idx = 0;
              memset(buffer, 0, sizeof(char) * MAX_BUF);
              printf("ERROR : Too long string\r\n");
           }
           buffer[idx] = recvChar;
           idx++;
        }
     }
  }
  
  // ULTRASONIC SENSOR
  void TIM4_IRQHandler(void){
     if(is_UIF(TIM4)){                       	// Update interrupt
        ovf_cnt++;                            // overflow count
        clear_UIF(TIM4);                      // clear update interrupt flag
     }
     if(is_CCIF(TIM4, 1)){                    // TIM4_Ch1 (IC1) Capture Flag. Rising Edge Detect
        timestart = TIM4->CCR1;               // Capture TimeStart
        clear_CCIF(TIM4, 1);                  // clear capture/compare interrupt flag 
     }                                              
     else if(is_CCIF(TIM4,2)){                // TIM4_Ch1 (IC2) Capture Flag. Falling Edge Detect
        timeend = TIM4->CCR2;      			// Capture TimeEnd
        clear_CCIF(TIM4, 2);
        
        timeInterval = 10*(-(timeend-timestart)+(0XFFFF*ovf_cnt));                
  
        ovf_cnt = 0;                                                     
     }
  }
  
  void ADC_IRQHandler(void){
      
     if(is_ADC_OVR()){
        clear_ADC_OVR();
     }
      
     if(is_ADC_EOC()){       
           if (flag==0){
              IR1 = ADC_read();
           }  
           else if (flag==1){
              IR2 = ADC_read();
           }
        flag =! flag;
     }
  }
  
  ```




## Results

Demo video : [![Video Label](http://img.youtube.com/vi/pOZleyquSps/0.jpg)](https://www.youtube.com/watch?v=pOZleyquSps)

Demo video link : https://www.youtube.com/watch?v=pOZleyquSps



## Troubleshooting

​		There were no major problems with individual utilization of sensors and communication. However, it took a lot of time to integrate the system. It took quite a long time to build an entire system that reads the values of multiple sensors in real time, performing the actions of multiple actuators, and communications between different systems under individual MCUs. Also, it was also difficult to debug things do not work properly that used to work well. By solving such things one by one, we were able to develop various perspectives in terms of system construction.



### Reference

----

* https://ykkim.gitbook.io/ec/