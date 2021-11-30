/*
 * TrajectoryTracking.c
 *
 *  Created on: 15 May 2020
 *      Author: yusuf
 */

#include "common.h"


float get_distance_meters(float lat1, float lon1, float lat2, float lon2)
{
	float fi1 = lat1 * DEG2RAD;
	float fi2 = lat2 * DEG2RAD;
	float dfi = (lat2 - lat1) * DEG2RAD;
	float dth = (lon2 - lon1) * DEG2RAD;

	float a = sin(dfi/2) * sin(dfi/2) + cos(fi1) * cos(fi2) * sin(dth/2) * sin(dth/2);
	float c = 2 * atan2(sqrtf(a), sqrtf(1-a));
	return EarthRadius * c;

}

float get_bearing(float lat1, float lon1, float lat2, float lon2)
{
	float fi1 = lat1 * DEG2RAD;
	float fi2 = lat2 * DEG2RAD;
	float th1 = lon1 * DEG2RAD;
	float th2 = lon2 * DEG2RAD;

	float y = sin(th2-th1) * cos(fi2);
	float x = cos(fi1) * sin(fi2) - sin(fi1) * cos(fi2) * cos(th2-th1);
	float th = atan2(y,x);
	return fmod((th * RAD2DEG + 360), 360);		// return degrees
//	return th * RAD2DEG;
}

float get_midpoint(float lat1, float lon1, float lat2, float lon2)
{
	float fi1 = lat1 * DEG2RAD;
	float fi2 = lat2 * DEG2RAD;
	float th1 = lon1 * DEG2RAD;
	float th2 = lon2 * DEG2RAD;
	float Bx = cos(fi2) * cos(th2-th1);
	float By = cos(fi2) * sin(th2-th1);
	float x = sin(fi1) + sin(fi2);
	float y = sqrtf(((cos(fi1) + Bx) * (cos(fi1) + Bx)) + (By*By));
	float fi3 = atan2(x,y);
	float th3 = th1 + atan2(By, cos(fi1) + Bx);
	float lat3 = fi3 * RAD2DEG;
	float lon3 = th3 * RAD2DEG;
	return lat3 + lon3;		///////////// düzenle.
}

float get_time_flight(uint16_t radius, uint16_t RPM, float gnd_speed)
{
	return 2.0f * M_PI * (float)radius / gnd_speed * (float)RPM;
}

void get_destination_point(float lat1, float lon1, float bearing, float distance, float* position)
{
	float sigma = distance / EarthRadius;

	float fi1 = lat1 * DEG2RAD;
	float th1 = lon1 * DEG2RAD;

	float fi2 = asin(sin(fi1)*cos(sigma) + cos(fi1)*sin(sigma)*cos(bearing*DEG2RAD));
	float th2 = th1 + atan2((sin(bearing*DEG2RAD)*sin(sigma)*cos(fi1)),(cos(sigma)-sin(fi1)*sin(fi2)));

	position[0] = fi2 * RAD2DEG;
	position[1] = th2 * RAD2DEG;
}

void LLA2ECEF(float lat, float lon, uint16_t altitude, float* ECEF_position)
{
	float a = 6378137;
	float b = 6356752.31424518f;
	float e = sqrt((a*a - b*b) / (a*a));

	float N = a / sqrt(1 - (e*e*sin(lat*DEG2RAD)*sin(lat*DEG2RAD)));

	ECEF_position[0] = (N+altitude)*cos(lat*DEG2RAD)*cos(lon*DEG2RAD);
	ECEF_position[1] = (N+altitude)*cos(lat*DEG2RAD)*sin(lon*DEG2RAD);
	ECEF_position[2] = ((b*b*N)/(a*a) + altitude)*sin(lat*DEG2RAD);

}

void ECEF2LLA(float X, float Y, float Z, float* LLA_position)
{
	float a = 6378137;
	float b = 6356752.31424518f;
	float e = sqrt((a*a - b*b) / (a*a));
	float p = sqrt(X*X + Y*Y);

	static double h;
	static double lat[2];
	static double N;

	h = 0;
	lat[0] = atan2(Z,p*(1-(e*e))) * RAD2DEG;

	while(!(fabs(lat[1]-lat[0]) < 0.00000001))
	{
		lat[1] = lat[0];

		N = a / sqrt(1 - (e*e*sin(lat[1]*DEG2RAD)*sin(lat[1] * DEG2RAD)));

		h = p/cos(lat[1]*DEG2RAD) - N;
		lat[0] = atan2(Z, p*(1 - (e*e*N)/(N+h)))*RAD2DEG;

	}

	LLA_position[0] = lat[0] ;
	LLA_position[1] = atan2(Y,X) * RAD2DEG;
	LLA_position[2] = h;
}







