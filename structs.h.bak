/*
 * structs.h
 *
 *  Created on: Mar 9, 2012
 *      Author: yonghan
 */

#ifndef STRUCTS_H_
#define STRUCTS_H_
#include "stm32f4xx_conf.h"

typedef struct
{
	double Roll, Pitch, Yaw;
} eulerAngle_t;

typedef struct
{
	// Accelerometer Data
	double AccelX, AccelY, AccelZ;

	// Gyroscope Data
	double GyroX, GyroY, GyroZ;

	// Magnetometer Data
	double MagnetX, MagnetY, MagnetZ;
} IMUData_t;

typedef struct
{
	double e0, e1, e2;
} vector_t;

typedef struct
{
	vector_t m0, m1, m2;
} matrix33_t;

#endif /* STRUCTS_H_ */
