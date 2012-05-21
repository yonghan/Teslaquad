/*
 * motor_control.h
 *
 *  Created on: Oct 11, 2011
 *      Author: Yonghan Ching
 */

#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include "stm32f4xx_conf.h"

typedef struct
{
    float Motor1Pulse;
    float Motor2Pulse;
    float Motor3Pulse;
    float Motor4Pulse;
} MotorCmdTypeDef;

void motor_init();
void motor_setPulseWidth(MotorCmdTypeDef *motor);
void motor_start();
void motor_stop();


#endif /* MOTOR_CONTROL_H_ */
