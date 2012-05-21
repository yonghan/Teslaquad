#include "stm32f4xx_conf.h"
#include "motor_control.h"
#include "stickIMU.h"

MotorCmdTypeDef MotorCmd;

//void TIM3_IRQHandler(void) 
//{
//
//    // flash on update event
//    if (TIM3->SR & TIM_FLAG_Update) 
//        GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
//
//    // Clear interrupt flag
//    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
//}

void PeriphInit(void)
{
    StickIMU_Init(100000); // 100kHz or 400kHz
    motor_init();
}

void main(void)
{
    PeriphInit();
    motor_start();

    while (1);
}