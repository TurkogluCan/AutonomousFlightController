/*
 * IMU.h
 *
 *  Created on: 3 Ara 2019
 *      Author: kouhu
 */

#ifndef IMU_H_
#define IMU_H_

typedef struct
{
	float Rx, Ry, Rz, R;
	float RxGyro, RyGyro, RzGyro;
	float GyroXZ, GyroYZ, GyroXY;
	float Racc;
	float Axz,Ayz;

	uint8_t first_sample;
}AttitudeEstimation;

typedef struct
{
	float baro_alt;
	float baro_vel;

	float acc_alt;
	float acc_vel;

}AltitudeEstimation;

typedef struct
{
	float pitch;
	float roll;
	float heading;

}ANGLE_t;

ANGLE_t ANGLE;
AttitudeEstimation att_Est;
AltitudeEstimation alt_Est;

void attitudeEstimation();
void altitudeEstimation();
void calculate_heading(float* mag);
int16_t _atan2(float y, float x);
void getAttitudeEstimation();

#endif /* IMU_H_ */
