/*
 * stickIMU.c
 *
 *  Created on: Mar 5, 2012
 *      Author: Yonghan Ching
 */

#include "stickIMU.h"

CPAL_TransferTypeDef stickIMUTx, stickIMURx;
uint8_t stickIMUTxBuff, stickIMURxBuff[6];



void StickIMU_Init(uint32_t clockrate)
{
    stickIMUTx.pbBuffer = &stickIMUTxBuff;
    stickIMURx.pbBuffer = stickIMURxBuff;
    CPAL_I2C_StructInit(&I2C1_DevStructure);
    I2C1_DevStructure.CPAL_ProgModel = CPAL_PROGMODEL_INTERRUPT;
    I2C1_DevStructure.pCPAL_TransferTx = &stickIMUTx;
    I2C1_DevStructure.pCPAL_TransferRx = &stickIMURx;
    I2C1_DevStructure.pCPAL_I2C_Struct->I2C_ClockSpeed = clockrate;
    CPAL_I2C_Init(&I2C1_DevStructure);

    // Initialize Accelerometer
    // Set POWER_CTL register to use measurement mode
    StickIMU_WriteRegister(ACCELEROMETER, 0x2d, 0x08);
    // Set DATA_FORMAT register to use full resolution
    StickIMU_WriteRegister(ACCELEROMETER, 0x31, 0x08);
    // Set BW_RATE register to 50Hz (25Hz bandwidth)
    StickIMU_WriteRegister(ACCELEROMETER, 0x2c, 0x09);

    // Initialize Gyroscope
    // Power on defaults
    StickIMU_WriteRegister(GYROSCOPE, 0x3e, 0x80);
    // Set Gyroscope to use Full Scale (±2000°/s),
    // internal LPF bandwith to 42Hz
    StickIMU_WriteRegister(GYROSCOPE, 0x16, 0x1b);
    // Set Internal Sampling Rate to 50Hz
    StickIMU_WriteRegister(GYROSCOPE, 0x15, 0x0a);
    // Set clock to PLL with z gyro reference
    StickIMU_WriteRegister(GYROSCOPE, 0x3e, 0x00);
    
    // Initialize Magnetometer
    // Write to mode register to set at continuous mode
    StickIMU_WriteRegister(MAGNETOMETER, 0x02, 0x00);
    // Write to config register A to set output rate to 50Hz
    StickIMU_WriteRegister(MAGNETOMETER, 0x00, 0x18);
}

ErrorStatus StickIMU_WriteRegister(StickIMU_Device device, uint8_t reg, uint8_t val)
{
    stickIMUTx.wNumData = 1;
    stickIMUTx.wAddr1 = device << 1;
    stickIMUTx.wAddr2 = reg;
    stickIMUTxBuff = val;
    I2C1_DevStructure.CPAL_Direction = CPAL_DIRECTION_TX;

    if (CPAL_I2C_Write(&I2C1_DevStructure) != CPAL_PASS)
    {
        // not able to start communication: Error Management
        return ERROR;
    }

    // Wait for the end of transfer
    while(I2C1_DevStructure.CPAL_State != CPAL_STATE_READY);
    return SUCCESS;
}

ErrorStatus StickIMU_ReadRegister(StickIMU_Device device, uint8_t reg)
{
    stickIMURx.wNumData = 6;
    stickIMURx.wAddr1 = device << 1;
    stickIMURx.wAddr2 = reg;
    I2C1_DevStructure.CPAL_Direction = CPAL_DIRECTION_RX;

    if (CPAL_I2C_Read(&I2C1_DevStructure) != CPAL_PASS)
    {
        // not able to start communication: Error Management
        return ERROR;
    }

    // Wait for the end of transfer
    while(I2C1_DevStructure.CPAL_State != CPAL_STATE_READY);
    return SUCCESS;
}

ErrorStatus StickIMU_ReadAccelerometer(IMUData_t *imu_Data)
{
    if(!StickIMU_ReadRegister(ACCELEROMETER, 0x32))
        return ERROR;

    imu_Data->AccelX = (int16_t)((stickIMURxBuff[3] << 8) | stickIMURxBuff[2]); // X axis (internal sensor Y axis)
    imu_Data->AccelY = (int16_t)((stickIMURxBuff[1] << 8) | stickIMURxBuff[0]); // Y axis (internal sensor X axis)
    imu_Data->AccelZ = (int16_t)((stickIMURxBuff[5] << 8) | stickIMURxBuff[4]); // Z axis (internal sensor Z axis)

    return SUCCESS;
}

ErrorStatus StickIMU_ReadGyroscope(IMUData_t *imu_Data)
{
    if(!StickIMU_ReadRegister(GYROSCOPE, 0x1d))
        return ERROR;

    imu_Data->GyroX = -1.0 * (int16_t)((stickIMURxBuff[2] << 8) | stickIMURxBuff[3]); // X (Roll) axis (internal sensor Y axis)
    imu_Data->GyroY = -1.0 * (int16_t)((stickIMURxBuff[0] << 8) | stickIMURxBuff[1]); // Y (Pitch) axis (internal sensor X axis)
    imu_Data->GyroZ = -1.0 * (int16_t)((stickIMURxBuff[4] << 8) | stickIMURxBuff[5]); // Z (Yaw) axis (internal sensor Z axis)

    return SUCCESS;
}

ErrorStatus StickIMU_ReadMagnetometer(IMUData_t *imu_Data)
{
    if(!StickIMU_ReadRegister(MAGNETOMETER, 0x03))
        return ERROR;

    imu_Data->MagnetX =        (int16_t)((stickIMURxBuff[0] << 8) | stickIMURxBuff[1]);
    imu_Data->MagnetY = -1.0 * (int16_t)((stickIMURxBuff[2] << 8) | stickIMURxBuff[3]);
    imu_Data->MagnetZ = -1.0 * (int16_t)((stickIMURxBuff[4] << 8) | stickIMURxBuff[5]);

    return SUCCESS;
}
