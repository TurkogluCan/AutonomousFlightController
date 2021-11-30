/*
 * pid_controller.h
 *
 *  Created on: 6 Nis 2020
 *      Author: kouhu
 */

#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_


//+++++++++++++++++++++++++++++++++ D E F I N E +++++++++++++++++++++++++++++++++++
#define s_		1.0
#define ms_  	1000.0
#define qs_ 	1000000.0

#define map_min_var 	-30				//** Valid min value which be Mapping
#define map_max_var 	30				//** Valid max value which be Mapping
#define map_min_des 	1000			//** Destination min value which be Mapping
#define map_max_des 	2000			//** Destination max value which be Mapping

#define rc_0     	    1500			//** Destination max value which be Mapping
#define min_roll_deg 	-45
#define max_roll_deg 	+45
#define min_pitch_deg 	-50
#define max_pitch_deg 	50

#define altitude_min 	0
#define takeoff_alt 	125
#define takeoff_speed 	15

#define land_time_s     10
#define land_alt_inter  10
//---------------------------------------------------------------------------------







//++++++++++++++++++++++++++ V A R I A B L E +++++++++++++++++++++++++++++++
typedef enum{

	PID_NORMAL,
	PID_CASCADE,
	PID_TYPE_ROLL,
	PID_TYPE_PITCH,
	PID_TYPE_ALTITUDE,
	PID_TYPE_HEADING,

	MODE_STABILIZE,
	MODE_AUTO,
	MODE_AUTO_TO,
	MODE_MANUEL

}pid_type_t;


typedef enum{

	TAKE_OFF_ENABLE,
	TAKE_OFF_FLY,
	TAKE_OFF_DISABLE,

}takeoff_state_t;

typedef struct{

	uint16_t position;

	uint16_t ServoDeg_0;
	uint16_t ServoDeg_n90;
	uint16_t ServoDeg_p90;

}Servo;

typedef struct{

	pid_type_t mode_flag;

}flight_mode_flag_t;

typedef struct PID_Control_Param_t{

	float 	    time_now;
	float 	    time_last;
	float 	    time_delta;
	float 	    error;
	float 	    error_total;
	float 	    error_last;
	float 	    error_delta;
	float 	    P, I, D;
	float 	    Kp, Ki, Kd;
	float 	    control;
	float 	    feedback;

	float       last_setpoint;
	int16_t     setpoint;
	int16_t     feed_min_deg;
	int16_t     feed_max_deg;

	pid_type_t  pid_type;

}PID_Control_Param;



typedef struct{

	double heading_land;
	double altitude_gnd;

//	uint8_t alt_time[land_time_s];

	uint8_t alt_time;
	uint8_t alt_T;

}Landing_Param_t;



typedef struct Flight_Control_Param_t{

	takeoff_state_t  	takeoff_state;
	flight_mode_flag_t 	flight_mode;

	Landing_Param_t		Landing_Param;

}Flight_Control_Param;



struct PID{

	PID_Control_Param 	Roll;
	PID_Control_Param 	Pitch;
	PID_Control_Param 	Altitude;
	PID_Control_Param 	Heading;
	PID_Control_Param 	Cascade_Altitude;
	PID_Control_Param 	Cascade_Heading;

	Flight_Control_Param Take_Off_t;
	Flight_Control_Param Landing_t;

	Servo	 			Servo_;
}PID_t;
//--------------------------------------------------------------------








//++++++++++++++++++++++++++ F C N +++++++++++++++++++++++++++++++
void  PIDInit(PID_Control_Param* PID, pid_type_t pid_part);
void  ServoInit	();
void  TickInit	();
float TickStart	();
float TickStop	();
void  ServoControl	      (float position, bool flag_map);
float Mapping		      (float valid_min, float valid_max, float dest_min, float dest_max, float value);

void PID_Control          (PID_Control_Param* PID, uint16_t reference, float feed_deg, float Kp, float Ki, float Kd);
void PID_CasCade_Control  (pid_type_t MODE, PID_Control_Param* PID_Cascade, PID_Control_Param PID_Out, PID_Control_Param PID_In, uint16_t ref_rc, float feed_deg, float Kp, float Ki, float Kd);

void  Alt_Head_Holder	  (PID_Control_Param* PID, float* setpoint, float feedback, uint16_t rc);
float Heading_Circular	  (PID_Control_Param PID);

void  Flight_TakeOff	   (Flight_Control_Param* Take_Off_t, float* alt_ref, float* heading_ref, float rakim, float heading,  float yer_hizi);
void  PID_Reset			   (PID_Control_Param* PID);

void  Landing			   (Landing_Param_t* Landing_Param, float* ref_altitude, float* ref_heading);
double Landing_Altitude_Calculate (double altitude_takeoff, float time, float interval_rate);
void   Auto_Landing_Algorithm_Exp (Landing_Param_t* Landing_Param);
//----------------------------------------------------------------










//----------------------------------------------------------------

#endif /* PID_CONTROLLER_H_ */
