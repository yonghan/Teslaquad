/*
 * DCM.h
 *
 *  Created on: Mar 8, 2012
 *      Author: yonghan
 */

#ifndef DCM_H_
#define DCM_H_

#include "structs.h"
#include <math.h>
#include "AHRS_math.h"

// SENSOR CALIBRATION
/*****************************************************************/
// How to calibrate? Read the tutorial at http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
// Put MIN/MAX and OFFSET readings for your board here!
// Accelerometer
// "accel x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define ACCEL_X_MIN ((double) -287)
#define ACCEL_X_MAX ((double) 245)
#define ACCEL_Y_MIN ((double) -251)
#define ACCEL_Y_MAX ((double) 280)
#define ACCEL_Z_MIN ((double) -298)
#define ACCEL_Z_MAX ((double) 220)

// Magnetometer
// "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define MAGN_X_MIN ((double) -652)
#define MAGN_X_MAX ((double) 679)
#define MAGN_Y_MIN ((double) -575)
#define MAGN_Y_MAX ((double) 756)
#define MAGN_Z_MIN ((double) -454)
#define MAGN_Z_MAX ((double) 747)

// Gyroscope
// "gyro x,y,z (current/average) = .../OFFSET_X  .../OFFSET_Y  .../OFFSET_Z
#define GYRO_AVERAGE_OFFSET_X ((double) 0.86)
#define GYRO_AVERAGE_OFFSET_Y ((double) -56.50)
#define GYRO_AVERAGE_OFFSET_Z ((double) -6.38)

/*
// Calibration example:
// "accel x,y,z (min/max) = -278.00/270.00  -254.00/284.00  -294.00/235.00"
#define ACCEL_X_MIN ((double) -278)
#define ACCEL_X_MAX ((double) 270)
#define ACCEL_Y_MIN ((double) -254)
#define ACCEL_Y_MAX ((double) 284)
#define ACCEL_Z_MIN ((double) -294)
#define ACCEL_Z_MAX ((double) 235)

// "magn x,y,z (min/max) = -511.00/581.00  -516.00/568.00  -489.00/486.00"
#define MAGN_X_MIN ((double) -511)
#define MAGN_X_MAX ((double) 581)
#define MAGN_Y_MIN ((double) -516)
#define MAGN_Y_MAX ((double) 568)
#define MAGN_Z_MIN ((double) -489)
#define MAGN_Z_MAX ((double) 486)

//"gyro x,y,z (current/average) = -32.00/-34.82  102.00/100.41  -16.00/-16.38"
#define GYRO_AVERAGE_OFFSET_X ((double) -34.82)
#define GYRO_AVERAGE_OFFSET_Y ((double) 100.41)
#define GYRO_AVERAGE_OFFSET_Z ((double) -16.38)
*/

// Sensor calibration scale and offset values
#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))

// Gain for gyroscope (ITG-3200)
#define GYRO_GAIN 0.06957 // Same gain on all axes
#define GYRO_SCALED_RAD(x) (x * TO_RAD(GYRO_GAIN)) // Calculate the scaled gyro readings in radians per second

// DCM parameters
#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

#define GRAVITY 256.0 // "1G reference" used for DCM filter and accelerometer calibration

#define DEBUG__NO_DRIFT_CORRECTION 0


void DCM_normalize(void);
void DCM_driftCorrection(void);
void DCM_updateMatrix(double G_Dt, IMUData_t *imu_data);
eulerAngle_t DCM_calculateEulerAngles(void);
void DCM_compassHeading(eulerAngle_t *e, IMUData_t *imu_data);
void DCM_compensateSensorErrors(IMUData_t *imu_rawData);
void DCM_resetSensorFusion(eulerAngle_t *e, IMUData_t *imu_raw);

#endif /* DCM_H_ */
