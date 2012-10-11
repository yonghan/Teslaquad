/*
===============================================================================================
 Name        : motor_control.c
 Author      : Yonghan Ching
 Version     : 0.1
 Date        : Oct 11, 2011
 Description : Definition for controlling brushless DC motor via a electronic speed controller
===============================================================================================
*/

#include "motor_control.h"


void motor_setPulseWidth(MotorCmdTypeDef *motor)
{
    // constrain Pulse Width
    if (motor->Motor1Pulse > 2.0f)
        motor->Motor1Pulse = 2.0f;
    else if (motor->Motor1Pulse < 1.0f)
        motor->Motor1Pulse = 1.0f;

    if (motor->Motor2Pulse > 2.0f)
        motor->Motor2Pulse = 2.0f;
    else if (motor->Motor2Pulse < 1.0f)
        motor->Motor2Pulse = 1.0f;

    if (motor->Motor3Pulse > 2.0f)
        motor->Motor3Pulse = 2.0f;
    else if (motor->Motor3Pulse < 1.0f)
        motor->Motor3Pulse = 1.0f;

    if (motor->Motor4Pulse > 2.0f)
        motor->Motor4Pulse = 2.0f;
    else if (motor->Motor4Pulse < 1.0f)
        motor->Motor4Pulse = 1.0f;

    // Update PulseWidth
    TIM_SetCompare1(TIM3, (uint32_t)(100.0f * motor->Motor1Pulse));
    TIM_SetCompare2(TIM3, (uint32_t)(100.0f * motor->Motor2Pulse));
    TIM_SetCompare3(TIM3, (uint32_t)(100.0f * motor->Motor3Pulse));
    TIM_SetCompare4(TIM3, (uint32_t)(100.0f * motor->Motor4Pulse));
}

void motor_init()
{
           GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM3_InitStructure;
          TIM_OCInitTypeDef TIM_OCInitStructure;



    /* GPIOC Periph clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    // Turn on TIM3 Clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* Configure PG13 in output pushpull mode */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Connect TIM3 pins to AF2 */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);

    /* Time base configuration */
    TIM3_InitStructure.TIM_Prescaler = 840 - 1; // TIM3CLK = 84MHz
    TIM3_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM3_InitStructure.TIM_Period = 2000;
    TIM3_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM3_InitStructure.TIM_RepetitionCounter = 0; // Not applicable
    TIM_TimeBaseInit(TIM3, &TIM3_InitStructure);

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 9970;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel2 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1500;
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel3 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1750;
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel4 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 2000;
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

    // Set Auto reload on
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

void motor_start()
{
    /* Reset and Start counter */
    TIM_SetCounter(TIM3, 0);

    /* Set Pulse Width to 1.0ms to arm Motors */
    MotorCmdTypeDef motorCmdInit = { 1.0f, 1.0f, 1.0f, 1.0f };
    motor_setPulseWidth(&motorCmdInit);

    /* Start PWM now */
    TIM_Cmd(TIM3, ENABLE);
}

void motor_stop()
{
    /* Stop PWM output */
    TIM_Cmd(TIM3, DISABLE);
}
