/*
 * MPU6050.h
 *
 *  Created on: 19 Eki 2019
 *      Author: Huseyin Koc
 */

#ifndef MPU6050_H_
#define MPU6050_H_

//#include "stdint.h"
#include "stdbool.h"

#define MPU6050_ADDR     0xD0
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define PWR_MGMT         0x6B
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define INT_PIN_CFG      0x37
#define USER_CTRL        0x6A
#define INT_ENABLE       0x38
#define INT_STATUS       0x3A
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_ID	     0x68

#define ACCEL_X_OUT_H    0x3B
#define ACCEL_X_OUT_L    0x3C
#define ACCEL_Y_OUT_H    0x3D
#define ACCEL_Y_OUT_L    0x3E
#define ACCEL_Z_OUT_H    0x3F
#define ACCEL_Z_OUT_L    0x40
#define GYRO_X_OUT_L     0x44
#define GYRO_X_OUT_H     0x43
#define GYRO_Y_OUT_L     0x46
#define GYRO_Y_OUT_H     0x45
#define GYRO_Z_OUT_L     0x48
#define GYRO_Z_OUT_H     0x47
#define TEMP_L           0x42
#define TEMP_H           0x41

#define ACCEL_SCALE 16384.0f
#define GYRO_SCALE  32.8f


struct mpu6050_raw_t
{
	int16_t accel[3];
	int16_t gyro[3];

	float accel_f32[3];
	float gyro_f32[3];
};

//struct mpu6050_data_t
//{
//	float a_total_vector;
//	float accel_roll, accel_pitch;
//	float angle_pitch, angle_roll, angle_yaw;
//};

struct MPU6050_offset_t
{
	int16_t gx,gy,gz;
	int16_t ax,ay,az;
};

typedef struct
{
	struct mpu6050_raw_t 		raw;
	struct MPU6050_offset_t 	offset;
//	struct mpu6050_data_t 		data;

}mpu6050_t;

mpu6050_t mpu6050;

void mpu6050_init(void);
void mpu6050_write(uint8_t addr, uint8_t txdata, uint8_t lenght);
uint8_t* mpu6050_read(uint8_t addr, uint8_t length);
void mpu6050_readAccelGyro();
void mpu6050_gyroCalibration(int16_t sampling);
void mpu6050_accelCalibration(int16_t sampling);
bool mpu6050_isReady();
//void attitudeEstimation(const float dt,const uint16_t wGyro);

#endif /* MPU6050_H_ */
