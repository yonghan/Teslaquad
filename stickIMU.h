/*
 * stickIMU.h
 *
 *  Created on: Mar 5, 2012
 *      Author: yonghan
 */

#ifndef STICKIMU_H_
#define STICKIMU_H_

#include "stm32f4xx_conf.h"
#include "structs.h"
#include "cpal_i2c.h"

typedef enum
{
    ACCELEROMETER = 0x53 << 1,
    GYROSCOPE     = 0x68 << 1,
    MAGNETOMETER  = 0x1e << 1
} StickIMU_Device;

void StickIMU_Init(uint32_t clockrate);
ErrorStatus StickIMU_WriteRegister(StickIMU_Device device, uint8_t reg, uint8_t val);
ErrorStatus StickIMU_ReadRegister(StickIMU_Device device, uint8_t reg);
ErrorStatus StickIMU_ReadAccelerometer(IMUData_t *imu_Data);
ErrorStatus StickIMU_ReadGyroscope(IMUData_t *imu_Data);
ErrorStatus StickIMU_ReadMagnetometer(IMUData_t *imu_Data);

#endif /* STICKIMU_H_ */
