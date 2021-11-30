/*
 * pid_controller.c
 *
 *  Created on: 6 Nis 2020
 *      Author: kouhu
 */

#include "common.h"




/**	FUNC	:	PID TIME CALCULATE Initialize
 *
 * 	AUTHOR 	: 	ECT
 */
void TickInit()
{
	CoreDebug->DEMCR 	|= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL 	   		|= DWT_CTRL_CYCCNTENA_Msk;				// ENABLE Counter - Control Register
	DWT->CYCCNT 		 = 0;									// RESET  Counter - Cycle Count Register
}



/**	FUNC	:	PID Variable Initialize
 *
 * 	AUTHOR 	: 	ECT
 */
void PIDInit(PID_Control_Param* PID, pid_type_t pid_part)
{
	// 0 0 0 0 0   P I D   0 0 0 0 0 0
	PID->time_last 			       = 0;
	PID->time_now  	               = 0;
	PID->time_delta			       = 1;
	PID->error			           = 0;
	PID->error_delta 	           = 0;
	PID->error_last	               = 0;
	PID->error_total	           = 0;
	PID->setpoint		           = 0;
	// 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0

	if		(pid_part == PID_TYPE_ROLL){
		PID->feed_min_deg	= min_roll_deg;
		PID->feed_max_deg	= max_roll_deg;
		PID->pid_type 		= PID_TYPE_ROLL;}

	else if	(pid_part == PID_TYPE_PITCH){
		PID->feed_min_deg	= min_pitch_deg;
		PID->feed_max_deg	= max_pitch_deg;
		PID->pid_type 		= PID_TYPE_PITCH;}

	else if	(pid_part == PID_TYPE_ALTITUDE)
		PID->pid_type = PID_TYPE_ALTITUDE;

	else if	(pid_part == PID_TYPE_HEADING)
		PID->pid_type = PID_TYPE_HEADING;

	else if	(pid_part == PID_CASCADE)
		PID->pid_type = PID_CASCADE;


	else{}
}



/**	FUNC	:	Servo Variable Initialize
 *
 * 	AUTHOR 	: 	ECT
 */
void ServoInit()
{
	PID_t.Servo_.ServoDeg_0 	= 1500;
	PID_t.Servo_.ServoDeg_n90 	= 1000;
	PID_t.Servo_.ServoDeg_p90 	= 2000;
}



/**	FUNC	:	PID TIME CALCULATE - START
 *
 * 	AUTHOR 	: 	ECT
 */
float TickStart()	// Return value in ms.
{
	float start_tick;

	start_tick = (float)(DWT->CYCCNT) / (float)(SystemCoreClock / ms_);

	return start_tick;
}



/**	FUNC	:	PID TIME CALCULATE - STOP
 *
 * 	AUTHOR 	: 	ECT
 */
float TickStop()	// Return value in ms.
{
	float 	stop_tick;

	stop_tick 	= (float)DWT->CYCCNT / (float)(SystemCoreClock / ms_);

	return stop_tick;
}



/**	FUNC	:	SERVO CONTROL
 *
 * 	AUTHOR 	: 	ECT
 */
void ServoControl(float position, bool flag_map)
{
	uint16_t pos_mapped;


	if(flag_map == true)
	{
		pos_mapped = 1500 + (uint16_t)position;

		pos_mapped = (pos_mapped < 1000) ? 1000 : pos_mapped;
		pos_mapped = (pos_mapped > 2000) ? 2000 : pos_mapped;


		PID_t.Servo_.position = pos_mapped;
	}

	else{ }

}



/**	FUNC	:	Mapping Function			// NEGATIF SAYILAR ICIN DUZENLE !!!!!!!!!!
 *
 * 	AUTHOR 	: 	ECT
 */
float Mapping(float valid_min, float valid_max, float dest_min, float dest_max, float value)
{
	float abs_valid = 0, abs_dest = 0, operation_1 = 0;

	if (value <= valid_min)
		return dest_min;
	else if (value >= valid_max)
		return dest_max;
	else
	{
		abs_valid = valid_max - valid_min;
		abs_dest  = dest_max  - dest_min;

		operation_1 = (valid_max - value) / abs_valid;

		return ((dest_max / abs_dest) - operation_1) * abs_dest;
	}

}





/*+++++++++++++++++++++++++++++++++++++++++++++++++++++ P I D ++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*
 * Reference	= RC Commander			  			►	 1000 			TO 		2000
 * Feedback		= Simulation Roll Degree  			►  -60_degree 		TO 		60_degree
 *
 * Error 		= Reference - Mapping(FeedbackToRC) ►	 1000 			TO 		2000
 *
 *
 */

/**	FUNC	:	PID  MAIN
 *
 * 	AUTHOR 	: 	ECT
 */
void PID_Control(PID_Control_Param* PID, uint16_t reference, float feed_deg, float Kp, float Ki, float Kd)
{

	PID->setpoint		= reference;

	if	 (PID->pid_type == PID_TYPE_ALTITUDE || PID->pid_type == PID_TYPE_HEADING)	PID->feedback	= feed_deg;																									//** If PID will be altitude CasCade control,
	else if (PID->pid_type == MODE_STABILIZE || PID->pid_type == MODE_AUTO);
	else																			PID->feedback	= Mapping(PID->feed_min_deg, PID->feed_max_deg, map_min_des, map_max_des, feed_deg);


	PID->error	=   PID->setpoint - PID->feedback;
	PID->error	=  (PID->pid_type == PID_TYPE_HEADING) ? Heading_Circular(*PID) : (PID->setpoint - PID->feedback);



	PID->error_total	+= PID->error;
    PID->error_delta	=  PID->error - PID->error_last;

    PID->Kp	        	= Kp;
    PID->Ki            	= Ki;
    PID->Kd            	= Kd;


    PID->P 				=  Kp * PID->error;
    PID->I 				= (Ki * PID->error_total) * PID->time_delta;
    PID->D 				= (Kd * PID->error_delta) / PID->time_delta;



    if(PID->pid_type == PID_TYPE_ALTITUDE || PID->pid_type == PID_TYPE_HEADING)
    	PID->control 		= PID->P + PID->I + PID->D;

    else{
    	PID->control 		= 1500 + PID->P + PID->I + PID->D;							//** ACIKLA
        PID->control 		= (PID->control < 1205) ? 1205 : PID->control;
        PID->control 		= (PID->control > 1795) ? 1795 : PID->control;
    }


    if		(PID->error_total >= 250)
    	PID->error_total = 250;
    else if (PID->error_total <=-250)
    	PID->error_total = -250;




    PID->error_last 	= PID->error;
    PID->time_delta 	= 0.05;




    /*
     *  C I K I S   I C I N
     *
     *  L I M I T L E Y I C I
     *
     *  E K L E
     *
     */
}



/**	FUNC	:	PID  CASCADE
 *
 * 	AUTHOR 	: 	ECT
 */
void PID_CasCade_Control(pid_type_t MODE, PID_Control_Param* PID_Cascade, PID_Control_Param PID_Out, PID_Control_Param PID_In, uint16_t ref_rc, float feed_deg, float Kp, float Ki, float Kd)
{

	PID_Cascade->setpoint		= PID_Out.control + ref_rc;
	PID_Cascade->feedback		= Mapping(PID_In.feed_min_deg, PID_In.feed_max_deg, map_min_des, map_max_des, feed_deg);				//** DEGIS


	if (MODE == MODE_STABILIZE){
		PID_Cascade->pid_type  = MODE_STABILIZE;
		PID_Control(PID_Cascade,PID_Cascade->setpoint, PID_Cascade->feedback, Kp, Ki, Kd);
	}
	else if (MODE == MODE_AUTO) {
		PID_Cascade->pid_type  = MODE_AUTO;
		PID_Control(PID_Cascade,PID_Cascade->setpoint, PID_Cascade->feedback, Kp, Ki, Kd);
	}

	else if (MODE == MODE_AUTO_TO){
		PID_Cascade->control = 1518;
	}


}



/**	FUNC	:	PID  RESET
 *
 * 	AUTHOR 	: 	ECT
 */
void PID_Reset(PID_Control_Param* PID)
{

	PID->P = 0;
	PID->I = 0;
	PID->D = 0;

	PID->error_total = 0;
	PID->error_last  = 0;
	PID->error_delta = 0;
	PID->error       = 0;

	PID->control	 = 0;

}



/**	FUNC	:	Stabilize Mode Altitude & Heading Holder
 *
 * 	AUTHOR 	: 	ECT
 */
void Alt_Head_Holder(PID_Control_Param* PID, float* setpoint, float feedback, uint16_t rc)
{


		if(PID_t.Take_Off_t.flight_mode.mode_flag == MODE_MANUEL || PID_t.Take_Off_t.flight_mode.mode_flag == MODE_AUTO){
			*setpoint = feedback;
			PID->last_setpoint = *setpoint;
		}

		else if( rc > 1490 && rc < 1510 ){
			*setpoint = PID->last_setpoint;
		}

		else {
			*setpoint = feedback;
			PID->last_setpoint = *setpoint;
		}
}



/**	FUNC	:	Heading 360 -> 0 and 0 -> 360 translate
 *
 * 	AUTHOR 	: 	ECT
 */
float Heading_Circular(PID_Control_Param PID)
{

	if 		(PID.error < -180)
		return PID.error + 360;

	else if (PID.error >= 180)
		return PID.error -360;

	else
		return PID.error;

}



/**	FUNC	:	Take off flight control
 *
 * 	AUTHOR 	: 	ECT
 */
void Flight_TakeOff(Flight_Control_Param* Take_Off_t, float* alt_ref, float* heading_ref, float rakim, float heading,  float yer_hizi)
{

	static uint8_t 	sayac = 0;
	static float 	heading_begin = 0;


	if (Take_Off_t->takeoff_state == TAKE_OFF_ENABLE)
	{
		if(sayac == 0){
			heading_begin = heading;
			sayac++;
		}
		else if( PID_t.Take_Off_t.flight_mode.mode_flag == MODE_MANUEL || PID_t.Take_Off_t.flight_mode.mode_flag == MODE_STABILIZE ){
			PID_Reset(&PID_t.Altitude);
			PID_Reset(&PID_t.Heading);
		}
		else if (yer_hizi < takeoff_speed){
			*heading_ref = heading_begin;
			*alt_ref	 = rakim;
		}
		else
			Take_Off_t->takeoff_state = TAKE_OFF_FLY;
	}

	else if(Take_Off_t->takeoff_state == TAKE_OFF_FLY)
	{
		*heading_ref = heading_begin;
		WaypointStatus = TAKEOFF_AIR;

		if (rakim >= takeoff_alt)
		{
			Take_Off_t->takeoff_state = TAKE_OFF_DISABLE;
			WaypointStatus = WPOINT;
		}

	}
	else;

}


double Landing_Altitude_Calculate(double altitude_takeoff, float time, float interval_rate)
{
	return altitude_takeoff*exp(-time/interval_rate);
}


void Landing(Landing_Param_t* Landing_Param, float* ref_altitude, float* ref_heading)
{

	*ref_altitude = Landing_Altitude_Calculate(Landing_Param->altitude_gnd, PID_t.Landing_t.Landing_Param.alt_time, Landing_Param->alt_T);
	*ref_heading  = Landing_Param->heading_land;

}



void Auto_Landing_Algorithm_Exp(Landing_Param_t* Landing_Param)
{
	Landing_Param->alt_T = 100;
	double h = 0;
	uint8_t flag = true;

	while(flag)
	{
		h = Landing_Param->altitude_gnd*exp(-land_time_s/Landing_Param->alt_T);

		if ( h <= 3 )
			flag = false;

		Landing_Param->alt_T--;
	}

}













































