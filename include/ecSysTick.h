/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : HanMinung
Created          : 05-03-2021
Modified         : 10-29-2022
Language/ver     : C++ in Keil uVision
/----------------------------------------------------------------*/

#ifndef __EC_SYSTICK_H
#define __EC_SYSTICK_H

#include "stm32f4xx.h"
#include "ecRCC.h"
#include <stdint.h>

#define MCU_CLK_PLL 84000000            // 84 MHz
#define MCU_CLK_HSI 16000000            // 16 MHz

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

void SysTick_init(uint32_t msec);
void SysTick_Handler(void);
void SysTick_counter();
void delay_ms(uint32_t msec);
void SysTick_reset(void);
uint32_t SysTick_val(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif