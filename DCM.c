/*
 * DCM.c
 *
 *  Created on: Mar 8, 2012
 *      Author: yonghan
 */

/* This file is part of the Razor AHRS Firmware */

#include "DCM.h"

// DCM variables
double MAG_Heading;
vector_t accel_v; // Store the acceleration in a vector
vector_t gyro_v;  // Store the gyros turn rate in a vector
vector_t omega_v; // Corrected gyro_v data
vector_t omega_P; // Omega Proportional correction
vector_t omega_I; // Omega Integrator
vector_t omega;
vector_t errorRollPitch;
vector_t errorYaw;
matrix33_t DCM_Matrix = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
matrix33_t update_Matrix = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
matrix33_t temporary_Matrix = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// DCM algorithm

void DCM_normalize(void)
{
	double error = 0;
	double renorm = 0;
	matrix33_t temporary;

	error = -vector_DotProduct(&DCM_Matrix.m0, &DCM_Matrix.m1) * 0.5; //eq.19
	temporary.m0 = vector_Scale(&DCM_Matrix.m1, error);              //eq.19
	temporary.m1 = vector_Scale(&DCM_Matrix.m0, error);              //eq.19

	temporary.m0 = vector_Add(&temporary.m0, &DCM_Matrix.m0);         //eq.19
	temporary.m1 = vector_Add(&temporary.m1, &DCM_Matrix.m1);         //eq.19

	temporary.m2 = vector_CrossProduct(&temporary.m0, &temporary.m1); // c = a x b, eq.20

	renorm = 0.5 * (3.0 - vector_DotProduct(&temporary.m0, &temporary.m0)); // eq.21
	DCM_Matrix.m0 = vector_Scale(&temporary.m0, renorm);

	renorm = 0.5 * (3.0 - vector_DotProduct(&temporary.m1, &temporary.m1)); // eq.21
	DCM_Matrix.m1 = vector_Scale(&temporary.m1, renorm);

	renorm = 0.5 * (3.0 - vector_DotProduct(&temporary.m2, &temporary.m2)); // eq.21
	DCM_Matrix.m2 = vector_Scale(&temporary.m2, renorm);
}

void DCM_driftCorrection(void)
{
	double mag_heading_x;
	double mag_heading_y;
	double errorCourse;

	//Compensation the Roll, Pitch and Yaw drift.
	static vector_t scaled_Omega_P;
	static vector_t scaled_Omega_I;
	double accel_Magnitude;
	double accel_Weight;


	//*****Roll and Pitch***************

	// Calculate the magnitude of the accelerometer vector
	accel_Magnitude = sqrt(accel_v.e0 * accel_v.e0 + accel_v.e1 * accel_v.e1 + accel_v.e2 * accel_v.e2);
	accel_Magnitude = accel_Magnitude / GRAVITY; // Scale to gravity.

	// Dynamic weighting of accelerometer info (reliability filter)
	// Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
	accel_Weight = constrain(1.0 - 2.0 * absd(1.0 - accel_Magnitude), 0.0, 1.0);

	errorRollPitch = vector_CrossProduct(&accel_v, &DCM_Matrix.m2); //adjust the ground of reference
	omega_P = vector_Scale(&errorRollPitch, Kp_ROLLPITCH * accel_Weight);
	scaled_Omega_I = vector_Scale(&errorRollPitch, Ki_ROLLPITCH * accel_Weight);
	omega_I = vector_Add(&omega_I, &scaled_Omega_I);

	//*****YAW***************
	// We make the gyro YAW drift correction based on compass magnetic heading

	mag_heading_x = cos(MAG_Heading);
	mag_heading_y = sin(MAG_Heading);
	errorCourse = (DCM_Matrix.m0.e0 * mag_heading_y) - (DCM_Matrix.m1.e0 * mag_heading_x);  //Calculating YAW error
	errorYaw = vector_Scale(&DCM_Matrix.m2, errorCourse); //Apply the yaw correction to the XYZ rotation of the aircraft, depeding the position.

	scaled_Omega_P = vector_Scale(&errorYaw, Kp_YAW); //.01proportional of YAW.
	omega_P = vector_Add(&omega_P, &scaled_Omega_P);   //Adding  Proportional.

	scaled_Omega_I = vector_Scale(&errorYaw, Ki_YAW);  //.00001Integrator
	omega_I = vector_Add(&omega_I, &scaled_Omega_I);   //adding integrator to the Omega_I
}

// BEWARE OF POINTERS!!!!!
void DCM_updateMatrix(double G_Dt, IMUData_t *imu_data)
{
	gyro_v.e0 = GYRO_SCALED_RAD(imu_data->GyroX); //gyro x roll
	gyro_v.e1 = GYRO_SCALED_RAD(imu_data->GyroY); //gyro y pitch
	gyro_v.e2 = GYRO_SCALED_RAD(imu_data->GyroZ); //gyro z yaw

	accel_v.e0 = imu_data->AccelX;
	accel_v.e1 = imu_data->AccelY;
	accel_v.e2 = imu_data->AccelZ;

	omega = vector_Add(&gyro_v, &omega_I);  //adding proportional term
	omega_v = vector_Add(&omega, &omega_P); //adding Integrator term

#if DEBUG__NO_DRIFT_CORRECTION == 1 // Do not use drift correction
	update_Matrix.m0.e0 = 0;
	update_Matrix.m0.e1 = -G_Dt * gyro_v.e2; //-z
	update_Matrix.m0.e2 = G_Dt * gyro_v.e1;  // y
	update_Matrix.m1.e0 = G_Dt * gyro_v.e2;  // z
	update_Matrix.m1.e1 = 0;
	update_Matrix.m1.e2 = -G_Dt * gyro_v.e0;
	update_Matrix.m2.e0 = -G_Dt * gyro_v.e1;
	update_Matrix.m2.e1 = G_Dt * gyro_v.e0;
	update_Matrix.m2.e2 = 0;
#else // Use drift correction
	update_Matrix.m0.e0 = 0;
	update_Matrix.m0.e1 = -G_Dt * omega_v.e2; //-z
	update_Matrix.m0.e2 = G_Dt * omega_v.e1;  // y
	update_Matrix.m1.e0 = G_Dt * omega_v.e2;  // z
	update_Matrix.m1.e1 = 0;
	update_Matrix.m1.e2 = -G_Dt * omega_v.e0;   //-x
	update_Matrix.m2.e0 = -G_Dt * omega_v.e1;   //-y
	update_Matrix.m2.e1 = G_Dt * omega_v.e0;    // x
	update_Matrix.m2.e2 = 0;
#endif

	temporary_Matrix = matrix_Multiply(&DCM_Matrix, &update_Matrix);

	DCM_Matrix.m0.e0 += temporary_Matrix.m0.e0;
	DCM_Matrix.m0.e1 += temporary_Matrix.m0.e1;
	DCM_Matrix.m0.e2 += temporary_Matrix.m0.e2;
	DCM_Matrix.m1.e0 += temporary_Matrix.m1.e0;
	DCM_Matrix.m1.e1 += temporary_Matrix.m1.e1;
	DCM_Matrix.m1.e2 += temporary_Matrix.m1.e2;
	DCM_Matrix.m2.e0 += temporary_Matrix.m2.e0;
	DCM_Matrix.m2.e1 += temporary_Matrix.m2.e1;
	DCM_Matrix.m2.e2 += temporary_Matrix.m2.e2;
}

eulerAngle_t DCM_calculateEulerAngles(void)
{
	eulerAngle_t a = {atan2(DCM_Matrix.m2.e1, DCM_Matrix.m2.e2),
			         -asin(DCM_Matrix.m2.e0),
			          atan2(DCM_Matrix.m1.e0, DCM_Matrix.m0.e0)};
	return a;
}

void DCM_compassHeading(eulerAngle_t *e, IMUData_t *imu_data)
{
	double mag_x;
	double mag_y;
	double cos_roll;
	double sin_roll;
	double cos_pitch;
	double sin_pitch;

	cos_roll = cos(e->Roll);
	sin_roll = sin(e->Roll);
	cos_pitch = cos(e->Pitch);
	sin_pitch = sin(e->Pitch);

	// Tilt compensated magnetic field X
	mag_x = imu_data->MagnetX * cos_pitch + imu_data->MagnetY * sin_roll * sin_pitch + imu_data->MagnetZ * cos_roll * sin_pitch;
	// Tilt compensated magnetic field Y
	mag_y = imu_data->MagnetY * cos_roll - imu_data->MagnetZ * sin_roll;
	// Magnetic Heading
	MAG_Heading = atan2(-mag_y, mag_x);
}

// Apply calibration to raw sensor readings
void DCM_compensateSensorErrors(IMUData_t *imu_raw)
{
    // Compensate accelerometer error
	imu_raw->AccelX = (imu_raw->AccelX - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
    imu_raw->AccelY = (imu_raw->AccelY - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
    imu_raw->AccelZ = (imu_raw->AccelZ - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;

    // Compensate magnetometer error
    imu_raw->MagnetX = (imu_raw->MagnetX - MAGN_X_OFFSET) * MAGN_X_SCALE;
    imu_raw->MagnetY = (imu_raw->MagnetY - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
    imu_raw->MagnetZ = (imu_raw->MagnetZ - MAGN_Z_OFFSET) * MAGN_Z_SCALE;

    // Compensate gyroscope error
    imu_raw->GyroX = imu_raw->GyroX - GYRO_AVERAGE_OFFSET_X;
    imu_raw->GyroY = imu_raw->GyroY - GYRO_AVERAGE_OFFSET_Y;
    imu_raw->GyroZ = imu_raw->GyroZ - GYRO_AVERAGE_OFFSET_Z;
}

// Read every sensor and record a time stamp
// Init DCM with unfiltered orientation
// TODO re-init global vars?
void DCM_resetSensorFusion(eulerAngle_t *e, IMUData_t *imu_raw)
{
	vector_t temp1;
	vector_t temp2;
	vector_t accelv = {imu_raw->AccelX, imu_raw->AccelY, imu_raw->AccelZ};
	vector_t xAxis = {1.0, 0.0, 0.0};


	double mag_x;
	double mag_y;
	double cos_roll;
	double sin_roll;
	double cos_pitch;
	double sin_pitch;

	// GET PITCH
	// Using y-z-plane-component/x-component of gravity vector
	e->Pitch = -atan2(imu_raw->AccelX, sqrt(imu_raw->AccelY * imu_raw->AccelY + imu_raw->AccelZ * imu_raw->AccelZ));

	// GET ROLL
	// Compensate pitch of gravity vector
	temp1 = vector_CrossProduct(&accelv, &xAxis);
	temp2 = vector_CrossProduct(&xAxis, &temp1);
	// Normally using x-z-plane-component/y-component of compensated gravity vector
	// roll = atan2f(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
	// Since we compensated for pitch, x-z-plane-component equals z-component:
	e->Roll = atan2(temp2.e1, temp2.e2);

	// GET YAW
	// DCM_compassHeading for raw data
	cos_roll = cos(e->Roll);
	sin_roll = sin(e->Roll);
	cos_pitch = cos(e->Pitch);
	sin_pitch = sin(e->Pitch);

	// Tilt compensated magnetic field X
	mag_x = imu_raw->MagnetX * cos_pitch + imu_raw->MagnetY * sin_roll * sin_pitch + imu_raw->MagnetZ * cos_roll * sin_pitch;
	// Tilt compensated magnetic field Y
	mag_y = imu_raw->MagnetY * cos_roll - imu_raw->MagnetZ * sin_roll;
	// Magnetic Heading
	MAG_Heading = atan2(-mag_y, mag_x);
	e->Yaw = MAG_Heading;

	// Init rotation matrix
	DCM_Matrix = init_RotationMatrix(e);
}
