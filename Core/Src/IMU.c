/*
 * IMU.c
 *
 *  Created on: 3 Ara 2019
 *      Author: kouhu
 */

#include "common.h"

/**
  * @brief MPU6050 complementary filtre kullanarak( yaw/pitch/roll hesaplar.
  * Detayli bilgi icin : starlino.com/imu_guide.html
  * @param  (void)
  * @retval	(void)
  */
void attitudeEstimation()
{
	att_Est.Racc = sqrtf( sqr(mpu6050.raw.accel_f32[0]) + sqr(mpu6050.raw.accel_f32[1]) + sqr(mpu6050.raw.accel_f32[2]));

	mpu6050.raw.accel_f32[0] = mpu6050.raw.accel_f32[0] / att_Est.Racc;
	mpu6050.raw.accel_f32[1] = mpu6050.raw.accel_f32[1] / att_Est.Racc;
	mpu6050.raw.accel_f32[2] = mpu6050.raw.accel_f32[2] / att_Est.Racc;

	filter_acc(mpu6050.raw.accel_f32);
	filter_gyro(mpu6050.raw.gyro_f32);

	att_Est.GyroYZ = mpu6050.raw.gyro_f32[0] * dt;
	att_Est.GyroXZ = mpu6050.raw.gyro_f32[1] * dt;

	if(att_Est.first_sample)
	{
		if(abs(mpu6050.raw.accel_f32[1]) < att_Est.Racc)
		{
			ANGLE.roll  = asin(mpu6050.raw.accel_f32[1] / att_Est.Racc)  * RAD2DEG;
		}
		if(abs(mpu6050.raw.accel_f32[0]) < att_Est.Racc)
		{
			ANGLE.pitch = asin(mpu6050.raw.accel_f32[0] / att_Est.Racc) * RAD2DEG;
		}

		att_Est.Rx = mpu6050.raw.accel_f32[0];
		att_Est.Ry = mpu6050.raw.accel_f32[1];
		att_Est.Rz = mpu6050.raw.accel_f32[2];

		att_Est.first_sample = 0;
	}
	else
	{
		if(abs(att_Est.Rz) < 0.05f)
		{
			att_Est.RxGyro = att_Est.Rx;
			att_Est.RyGyro = att_Est.Ry;
			att_Est.RzGyro = att_Est.Rz;
		}
		else
		{
			att_Est.Axz = atan2(att_Est.Rx,att_Est.Rz) * RAD2DEG;
			att_Est.Ayz = atan2(att_Est.Ry,att_Est.Rz) * RAD2DEG;

			att_Est.Axz -= att_Est.GyroXZ;
			att_Est.Ayz += att_Est.GyroYZ;

			int8_t signRzGyro = ( cos(att_Est.Axz * DEG2RAD) >=0 ) ? 1 : -1;

			att_Est.RxGyro = sin(att_Est.Axz*DEG2RAD) / sqrt (1 + sqr(cos(att_Est.Axz*DEG2RAD)) * sqr(tan(att_Est.Ayz*DEG2RAD)));
			att_Est.RyGyro = sin(att_Est.Ayz*DEG2RAD) / sqrt (1 + sqr(cos(att_Est.Ayz*DEG2RAD)) * sqr(tan(att_Est.Axz*DEG2RAD)));
			att_Est.RzGyro = signRzGyro * sqrt(1 - sqr(att_Est.RxGyro) - sqr(att_Est.RyGyro));
		}

		if(!(abs(att_Est.Racc) >= 1.02 || abs(att_Est.Racc) <= 0.985)) // 1.12 -- 0.95
		{
			att_Est.Rx = (mpu6050.raw.accel_f32[0] + att_Est.RxGyro * wGyro ) / (1 + wGyro);
			att_Est.Ry = (mpu6050.raw.accel_f32[1] + att_Est.RyGyro * wGyro ) / (1 + wGyro);
			att_Est.Rz = (mpu6050.raw.accel_f32[2] + att_Est.RzGyro * wGyro ) / (1 + wGyro);

		}
		else
		{
			att_Est.Rx = att_Est.RxGyro;
			att_Est.Ry = att_Est.RyGyro;
			att_Est.Rz = att_Est.RzGyro;
		}

		att_Est.R = sqrt(sqr(att_Est.Rx) + sqr(att_Est.Ry) + sqr(att_Est.Rz));
		att_Est.Rx = att_Est.Rx / att_Est.R;
		att_Est.Ry = att_Est.Ry / att_Est.R;
		att_Est.Rz = att_Est.Rz / att_Est.R;

		if(abs(att_Est.Ry) < att_Est.R)
		{
			ANGLE.roll  = -asin(att_Est.Ry / att_Est.R)  * RAD2DEG;
		}
		if(abs(att_Est.Rx) < att_Est.R)
		{
			ANGLE.pitch = asin(att_Est.Rx / att_Est.R) * RAD2DEG;
		}

	}
}


void calculate_heading(float* mag)
{
	float declination_angle = 5.716666667;
    float Xh,Yh;

    Xh =  (float)mag[0]*cos(ANGLE.pitch * DEG2RAD) + (double)mag[1]*sin(ANGLE.roll * DEG2RAD)*sin(ANGLE.pitch * DEG2RAD) -
						 	 	   (double)mag[2]*cos(ANGLE.roll * DEG2RAD)*sin(ANGLE.pitch * DEG2RAD);

    Yh = (float)mag[1]*cos(ANGLE.roll * DEG2RAD) + (double)mag[2]*sin(ANGLE.roll * DEG2RAD);


	ANGLE.heading = atan2(Yh,Xh) * RAD2DEG ;
	ANGLE.heading += declination_angle;

	filter_heading(&ANGLE.heading);

	if(ANGLE.heading < 0) ANGLE.heading += 360;
}


void altitudeEstimation()
{
	  if(!(GL.now_time < GL.baro_time))
	  {
		  GL.baro_time = GL.now_time;

		  switch(GL.baro_state)
		  {
			  case 0:
			  {
				  ms5611.D1 = ms5611_readRawPressure();
				  ms5611_getRawTemp();
				  GL.baro_time += CONVERSION_OSR_4096;
				  GL.baro_state = 1;
				  break;
			  }
			  case 1:
			  {
				  ms5611.D2 = ms5611_readRawTemp();
				  ms5611_getRawPressure();
				  GL.baro_time += CONVERSION_OSR_4096;
				  GL.baro_state = 0;
				  break;
			  }
		  }
	  }
}

