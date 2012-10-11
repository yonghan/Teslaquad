/*
 * debug_output.h
 *
 *  Created on: Mar 19, 2012
 *      Author: yonghan
 */

#ifndef DEBUG_OUTPUT_H_
#define DEBUG_OUTPUT_H_

#include "structs.h"
#include "stickIMU.h"

#define DEBUG_ENABLE

// OUTPUT OPTIONS
/*****************************************************************/
// Output mode
#define OUTPUT__MODE_CALIBRATE_SENSORS  0// Outputs sensor min/max values as text for manual calibration
#define OUTPUT__MODE_ANGLES_TEXT 1 // Outputs yaw/pitch/roll in degrees as text
#define OUTPUT__MODE_ANGLES_BINARY 2 // Outputs yaw/pitch/roll in degrees as binary float
#define OUTPUT__MODE_SENSORS_TEXT 3 // Outputs (calibrated) sensor values for all 9 axes as text


// Select if serial continuous streaming output is enabled per default on startup.
#define OUTPUT__STARTUP_STREAM_ON TRUE  // true or false

uint8_t output_mode;
uint8_t curr_calibrationSensor;

uint8_t num_accelErrors;
uint8_t num_magnetErrors;
uint8_t num_gyroErrors;

Bool output_singleOn;
Bool output_streamOn;
Bool output_errors;
Bool reset_calibrationSessionFlag;

void _checkResetCalibrationSession(IMUData_t *imu_Raw);
void _outputAngles(eulerAngle_t *e);
void _outputCalibration(uint8_t s, IMUData_t *imu_Raw);
void _outputSensors(IMUData_t *imu_data);
void _outputErrorCounts(void);
void _outputIteration(uint32_t iterations);
#endif /* DEBUG_OUTPUT_H_ */
