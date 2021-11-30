/*
 * TrajectoryTracking.h
 *
 *  Created on: 15 May 2020
 *      Author: yusuf
 */

#ifndef TRAJECTORYTRACKING_H_
#define TRAJECTORYTRACKING_H_

#define EarthRadius 6371000		// R = 6371km

#define MAX_WP_BOYUT 14
typedef struct _Waypoints
{
	uint8_t id; 		// waypoint numarasi
	float enlem;		// waypoint gps enlem
	float boylam;		// waypoint gps boylam
	uint16_t yukseklik; // hedef yukseklik
	uint8_t radius;		// waypoint ulasildi cemberi
	uint8_t mission;

	float distance;
	float bearing;
}Waypoints;

typedef enum
{
	WPOINT = 0,
	TAKEOFF_GND,
	TAKEOFF_AIR,
	LANDING,
	LOITER
}WaypointStatus_t;

typedef enum
{
    MISSION_LOITER,
    MISSION_TAKEOFF,
    MISSION_WAYPOINT,
    MISSION_LANDING
}MISSION;


Waypoints waypoints[MAX_WP_BOYUT];
Waypoints target_waypoint;
Waypoints current_waypoint;
Waypoints  last_waypoint;
WaypointStatus_t WaypointStatus;
uint8_t wp_sayisi;


float get_distance_meters(float lat1, float lon1, float lat2, float lon2);
float get_bearing(float lat1, float lon1, float lat2, float lon2);
float get_midpoint(float lat1, float lon1, float lat2, float lon2);
float get_time_flight(uint16_t radius, uint16_t RPM, float gnd_speed);
void get_destination_point(float lat1, float lon1, float bearing, float distance, float* position);
void LLA2ECEF(float lat, float lon, uint16_t altitude, float* ECEF_position);
void ECEF2LLA(float X, float Y, float Z, float* LLA_position);

#endif /* TRAJECTORYTRACKING_H_ */
