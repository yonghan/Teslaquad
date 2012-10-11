/*
 * timers.c - configurations and functions for misc timers
 *
 *  Created on: Sep 22, 2012
 *      Author: Yonghan Ching
 *
 */

 #include "timers.h"

// Set up a timer that overflows and interrupts at 1ms interval
// to be used by CPAL instead of Systick
 void CPAL_TimeOutManagerInit(void)
 {
    TIM_TimeBaseInitTypeDef TIM9_InitStructure;

    // Turn on TIM9 Clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);

    TIM9_InitStructure.TIM_Prescaler = 16800; /* SystemCoreClock / 10000 */
    TIM9_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM9_InitStructure.TIM_Period = 65535;
    TIM9_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM9_InitStructure.TIM_RepetitionCounter = 0;

    // Initialize TIM9 Registers
    TIM_TimeBaseInit(TIM9, &TIM9_InitStructure);
    // Enable interrupt on timer update
    TIM_ITConfig(TIM9, TIM_IT_Update, ENABLE);
    // Set autoreload at 1ms (10 tick)
    TIM_SetAutoreload(TIM9, 10);
    // Enable preloading autoreload register
    TIM_ARRPreloadConfig(TIM9, ENABLE);
    // Enable Timer
    TIM_Cmd(TIM9, ENABLE);
 }