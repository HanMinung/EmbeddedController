/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : HanMinung
Created          : 05-03-2021
Modified         : 10-29-2022
Language/ver     : C++ in Keil uVision
/----------------------------------------------------------------*/

#ifndef __FINAL_PROJ_H
#define __FINAL_PROJ_H

#include "stm32f4xx.h"

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

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






#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif