/*
 * AFC_main.c
 *
 *  Created on: 20 Mar 2020
 *      Author: Yusuf TUNC
 */

#include "common.h"


#define SIMULASYON
//#define GERCEK_ZAMANLI

void AFC_main(void)
{
	/* Kangal Kopegi Initilize */
	HAL_IWDG_Init(&hiwdg);

	//!< Debugda iken IWDG hata vermesi engellenir, debugda iken IDWG durur.
	DBGMCU->CR |= (1 << 8);

	/* PWM cikislari aktif edilir */
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);

#ifdef GERCEK_ZAMANLI
	  /*** Sensor initialize ***/
	  mpu6050_init();
	  Barometer_init();
	  QMC5883L_init();
	  mpu6050_gyroCalibration(500);
	  pw_init();
	  /*** Sensor initialize ***/
#endif
	  /*** Enable interrupts ***/
#ifdef GERCEK_ZAMANLI
	  HAL_UART_Receive_IT(&huart1,&GPS.Data.receive,sizeof(GPS.Data.receive));
	  HAL_TIM_Base_Start(&htim1);
#endif
	  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	  HAL_UART_Receive_IT(&huart2,&comm.gelen_veri_u8,sizeof(comm.gelen_veri_u8));
	  /*** Enable interrupts ***/

	  comm.flag.UART_READY = 1;
	  att_Est.first_sample = 1;

	  target_waypoint.id = 1;
	  current_waypoint.id = 0;
	  last_waypoint.id = 0;

	  /******************* P I D   I N I T ***********************/
	  PIDInit	(&PID_t.Roll,              PID_TYPE_ROLL);
	  PIDInit	(&PID_t.Pitch,             PID_TYPE_PITCH);
	  PIDInit	(&PID_t.Altitude,          PID_TYPE_ALTITUDE);
	  PIDInit	(&PID_t.Heading,           PID_TYPE_HEADING);
	  PIDInit	(&PID_t.Cascade_Heading,   PID_CASCADE);
	  PIDInit	(&PID_t.Cascade_Altitude,  PID_CASCADE);
//	  PIDInit	(&PID_t.Take_Off_t,  	   TAKE_OFF_ENABLE);

	  PID_t.Take_Off_t.takeoff_state = TAKE_OFF_ENABLE;

	  uint8_t flag_land = false;
	  uint8_t flag_land_2 = false;
	  /**********************************************************/

	while (1)
	{

		GL.now_time = HAL_GetTick();

		if(tick.task_200hz)
		{
#ifdef GERCEK_ZAMANLI

			if(comm.calib_mode.mag)
			{
				QMC5883L_readMag(qmc5883.raw.mag);	/* Manyetometreden veri okuma */
			}
			else
			{
				TIM1->CNT = 0;

				/******************************* IMU Islemleri *******************************/
				mpu6050_readAccelGyro();			/* Ivmeolcer ve jiroskop veri okuma */
				QMC5883L_readMag(qmc5883.raw.mag);	/* Manyetometreden veri okuma */
				attitudeEstimation();
				calculate_heading(qmc5883.raw.mag);	/* Heading acisini hesaplar */
				altitudeEstimation();

				/******************************* IMU Islemleri *******************************/

				GL.elapsed_time = TIM1->CNT;
				CALIB_LED_DISABLE
			}
#endif
			tick.task_200hz = 0;

		} /*  if(tick.task_200hz) */

		if(tick.task_100hz)
		{
			HAL_IWDG_Refresh(&hiwdg);

			tick.task_100hz = 0;
		} /* if(tick.task_100hz) */

		if(tick.task_50hz)
		{
#ifdef GERCEK_ZAMANLI
			/* RC kumanda min/max kalibrasyon modu */
			if(comm.calib_mode.rc)
			{
				CALIB_LED_ENABLE
				if(comm.flag.UART_READY)
				{
					uint8_t paket_sayac = 0;
					RC_paket_olustur(&paket_sayac);
					AFC_paket_gonder(paket_sayac);
				}
			} /* if(comm.mode.rc_calib_mode) */
			else
			{
				GL.uart_cevrim_suresi++;
				static uint8_t paket_sayaci = 0;

				Barometer_calculate();

				if(!(GL.uart_cevrim_suresi % 1))
				{
					IMU_paket_olustur(&paket_sayaci);
				}

				if(!(GL.uart_cevrim_suresi % 2))
				{
					GPS_paket_olustur(&paket_sayaci);
				}

				if(!(GL.uart_cevrim_suresi % 3))
				{
					MOD_paket_olustur(&paket_sayaci);
				}

				if(!(GL.uart_cevrim_suresi % 4))
				{
					batarya_paket_olustur(&paket_sayaci);
				}

				if(comm.flag.UART_READY)
				{
					AFC_paket_gonder(paket_sayaci);
					paket_sayaci = 0;
					comm.flag.UART_READY = 0;
				}
			}
#endif

			tick.task_50hz = 0;
		} /* if(tick.task_50hz) */

		if(tick.task_20hz)
		{
			GL.uart_cevrim_suresi++;
			GL.pid_cevrim_suresi++;

			static uint8_t paket_sayaci = 0;

			switch((uint8_t)FLIGHT_MODE)
			{
				case MANUEL :
				{
					PID_t.Take_Off_t.flight_mode.mode_flag = MODE_MANUEL;

					break;
				}

				case STABILIZE :
				{
					/**************************************************************************** P I D ****************************************************************************************************************/

					if(!(GL.pid_cevrim_suresi % 2))
					{
						Alt_Head_Holder(&PID_t.Altitude, &comm.simulation.PID.alt_ref, 		comm.simulation.rakim,   ch.pitch);
						Alt_Head_Holder(&PID_t.Heading,  &comm.simulation.PID.heading_ref,  comm.simulation.heading, ch.roll );

						PID_Control(&PID_t.Altitude, comm.simulation.PID.alt_ref,     comm.simulation.rakim,    comm.simulation.PID.alt_kp,     comm.simulation.PID.alt_ki,     comm.simulation.PID.alt_kd);
						PID_Control(&PID_t.Heading,  comm.simulation.PID.heading_ref, comm.simulation.heading, 	comm.simulation.PID.heading_kp, comm.simulation.PID.heading_ki, comm.simulation.PID.heading_kd);

						GL.pid_cevrim_suresi = 0;
					}


					PID_CasCade_Control(MODE_STABILIZE, &PID_t.Cascade_Altitude, PID_t.Altitude,  PID_t.Pitch, ch.pitch, comm.simulation.pitch, comm.simulation.PID.pitch_kp, comm.simulation.PID.pitch_ki, comm.simulation.PID.pitch_kd);
					PID_CasCade_Control(MODE_STABILIZE, &PID_t.Cascade_Heading,  PID_t.Heading,   PID_t.Roll,  ch.roll,  comm.simulation.roll,  comm.simulation.PID.roll_kp,  comm.simulation.PID.roll_ki,  comm.simulation.PID.roll_kd);

					/***************************************************************************************************************************************************************************************************/

					PID_t.Take_Off_t.flight_mode.mode_flag = MODE_STABILIZE;
					break;
				}

				case AUTO :
				{
					current_waypoint.enlem = comm.simulation.enlem;
					current_waypoint.boylam = comm.simulation.boylam;
					current_waypoint.yukseklik = comm.simulation.rakim;
					current_waypoint.mission = waypoints[target_waypoint.id].mission;

					if(current_waypoint.mission == MISSION_TAKEOFF )
					{
						comm.simulation.PID.heading_ref = get_bearing(waypoints[current_waypoint.id].enlem, waypoints[current_waypoint.id].boylam,waypoints[target_waypoint.id].enlem, waypoints[target_waypoint.id].boylam );
					}

					static uint8_t flag_sil = 0;

					if (PID_t.Take_Off_t.takeoff_state 	== TAKE_OFF_ENABLE)
					{

						if(!(GL.pid_cevrim_suresi % 2))
						{
							Flight_TakeOff(&PID_t.Take_Off_t, &comm.simulation.PID.alt_ref, &comm.simulation.PID.heading_ref, comm.simulation.rakim, comm.simulation.PID.heading_ref,  comm.simulation.yer_hizi);

							PID_Control(&PID_t.Altitude, comm.simulation.PID.alt_ref,     comm.simulation.rakim,    comm.simulation.PID.alt_kp,     comm.simulation.PID.alt_ki,     comm.simulation.PID.alt_kd);
							PID_Control(&PID_t.Heading,  comm.simulation.PID.heading_ref, comm.simulation.heading, 	comm.simulation.PID.heading_kp, comm.simulation.PID.heading_ki, comm.simulation.PID.heading_kd);

							GL.pid_cevrim_suresi = 0;
						}

						PID_CasCade_Control(MODE_AUTO, &PID_t.Cascade_Altitude, PID_t.Altitude,  PID_t.Pitch, rc_0,  comm.simulation.pitch, comm.simulation.PID.pitch_kp, 	 comm.simulation.PID.pitch_ki, 	  comm.simulation.PID.pitch_kd);
						PID_CasCade_Control(MODE_AUTO, &PID_t.Cascade_Heading,  PID_t.Heading,   PID_t.Roll,  rc_0,  comm.simulation.roll,  comm.simulation.PID.takeoff_kp,  comm.simulation.PID.takeoff_ki,  comm.simulation.PID.takeoff_kd);

						WaypointStatus = TAKEOFF_GND;
					}

					else if (PID_t.Take_Off_t.takeoff_state == TAKE_OFF_FLY)
					{

						if(!(GL.pid_cevrim_suresi % 2))
						{
							Flight_TakeOff(&PID_t.Take_Off_t, &comm.simulation.PID.alt_ref, &comm.simulation.PID.heading_ref, comm.simulation.rakim, comm.simulation.PID.heading_ref,  comm.simulation.yer_hizi);
							PID_Control(&PID_t.Heading,  comm.simulation.PID.heading_ref, comm.simulation.heading, 	comm.simulation.PID.heading_kp, comm.simulation.PID.heading_ki, comm.simulation.PID.heading_kd);

							GL.pid_cevrim_suresi = 0;
						}

						PID_CasCade_Control(MODE_AUTO_TO, &PID_t.Cascade_Altitude, PID_t.Altitude,  PID_t.Pitch, rc_0,  comm.simulation.pitch, comm.simulation.PID.pitch_kp, comm.simulation.PID.pitch_ki, comm.simulation.PID.pitch_kd);
						PID_CasCade_Control(MODE_AUTO   , &PID_t.Cascade_Heading,  PID_t.Heading,   PID_t.Roll,  rc_0,  comm.simulation.roll,  comm.simulation.PID.roll_kp,  comm.simulation.PID.roll_ki,  comm.simulation.PID.roll_kd);
					}

					else
					{
						if(WaypointStatus == WPOINT)
						{
							last_waypoint.enlem 	= waypoints[last_waypoint.id].enlem;
							last_waypoint.boylam 	= waypoints[last_waypoint.id].boylam;
							last_waypoint.yukseklik = waypoints[last_waypoint.id].yukseklik;
							last_waypoint.radius 	= waypoints[last_waypoint.id].radius;


							current_waypoint.enlem = comm.simulation.enlem;
							current_waypoint.boylam = comm.simulation.boylam;
							current_waypoint.yukseklik = comm.simulation.rakim;
							current_waypoint.mission = waypoints[target_waypoint.id].mission;

							PID_t.Landing_t.Landing_Param.heading_land = get_bearing(last_waypoint.enlem, last_waypoint.boylam, target_waypoint.enlem, target_waypoint.boylam);

							target_waypoint.enlem = waypoints[target_waypoint.id].enlem;
							target_waypoint.boylam = waypoints[target_waypoint.id].boylam;
							target_waypoint.yukseklik = waypoints[target_waypoint.id].yukseklik;
							target_waypoint.radius = waypoints[target_waypoint.id].radius;

							float Ru 		= get_distance_meters(last_waypoint.enlem, last_waypoint.boylam, current_waypoint.enlem, current_waypoint.boylam);
							float theta 	= get_bearing (last_waypoint.enlem, last_waypoint.boylam, target_waypoint.enlem,  target_waypoint.boylam);
							float theta_u	= get_bearing (last_waypoint.enlem, last_waypoint.boylam, current_waypoint.enlem, current_waypoint.boylam);

							float beta = theta - theta_u;
							float R = 0;
							float delta = 100; //get_distance_meters(last_waypoint.enlem, last_waypoint.boylam, target_waypoint.enlem, target_waypoint.boylam) / 10;

							if(abs(beta) < 90)
							{
								R = sqrtf(Ru*Ru - (Ru*sin(beta*DEG2RAD)*(Ru*sin(beta*DEG2RAD))));
							}


							float virtual_point_lat_lon[2];

							get_destination_point(last_waypoint.enlem, last_waypoint.boylam,theta,R+delta,virtual_point_lat_lon);

							estimation_point_gonder(virtual_point_lat_lon[0],  virtual_point_lat_lon[1],&paket_sayaci);

							comm.simulation.PID.heading_ref = get_bearing (current_waypoint.enlem, current_waypoint.boylam, virtual_point_lat_lon[0],  virtual_point_lat_lon[1]);
							comm.simulation.PID.alt_ref = target_waypoint.yukseklik;

							target_waypoint.distance = get_distance_meters(virtual_point_lat_lon[0],  virtual_point_lat_lon[1], target_waypoint.enlem, target_waypoint.boylam);


							if(target_waypoint.distance <= target_waypoint.radius)
							{
								target_waypoint.id++;
								current_waypoint.id++;
								last_waypoint.id++;

								if(target_waypoint.id >= wp_sayisi)
								{

									//** L A N D  &  L O I T E R **//
									if (current_waypoint.mission == MISSION_LANDING)
									{
//										 PID_t.Landing_t.Landing_Param.heading_land = get_bearing(last_waypoint.enlem, last_waypoint.boylam, current_waypoint.enlem, current_waypoint.boylam);
										 WaypointStatus = LANDING;
									}
									else if (current_waypoint.mission == MISSION_LOITER)
										WaypointStatus = LOITER;
									// son noktada cember olustursun...
									// yada home point gitsin cember atsin.
								}

							 }

						  }

						  else if(WaypointStatus == LANDING)
						  {
							if (flag_sil == 0)
							{
								PID_t.Landing_t.Landing_Param.altitude_gnd = comm.simulation.altitude_from_ground;
								Auto_Landing_Algorithm_Exp(&PID_t.Landing_t.Landing_Param);

								flag_sil++;
							}

//							  if (PID_t.Landing_t.Landing_Param.alt_time <= land_time_s)
//							  {


								  Landing(&PID_t.Landing_t.Landing_Param, &comm.simulation.PID.alt_ref, &comm.simulation.PID.heading_ref);

//
								  if (flag_land_2)
								  {
									  comm.simulation.PID.alt_ref += 70;
								  }
								  else if (flag_land && comm.simulation.altitude_from_ground > 5)
								  {
									  comm.simulation.PID.alt_ref += 100;
								  }
								  else if (comm.simulation.altitude_from_ground <= 5)
								  {
									  flag_land_2 = 1;
								  }




//							  }

							  comm.simulation.rakim = comm.simulation.altitude_from_ground;
						  }

						  else if(WaypointStatus == LOITER)
						  {
						  	  current_waypoint.enlem = comm.simulation.enlem;
						  	  current_waypoint.boylam = comm.simulation.boylam;
						  	  current_waypoint.yukseklik = comm.simulation.rakim;

						  	  float center_XYZ[3];
						  	  float current_XYZ[3];

						  	  LLA2ECEF(target_waypoint.enlem,  target_waypoint.boylam,  target_waypoint.yukseklik,  center_XYZ);  // Radius merkezi ECEF koordinatlari
						  	  LLA2ECEF(current_waypoint.enlem, current_waypoint.boylam, current_waypoint.yukseklik, current_XYZ); // Current point ECEF koordinatlari

						  	  float theta = atan2(current_XYZ[1]-center_XYZ[1], current_XYZ[0]-center_XYZ[0]);					// Center ve hava araci arasindaki aci

						  	  float virtual_point_ecef[2];

						  	  virtual_point_ecef[0] = target_waypoint.radius*cos(theta+0.6f);										// tahmini noktanin (0,0)'a X uzakligi
						  	  virtual_point_ecef[1] = target_waypoint.radius*sin(theta+0.6f);										// tahmini noktanin (0,0)'a Y uzakligi

						  	  virtual_point_ecef[0] += center_XYZ[0];																// tahmini noktanin merkeze X uzakligi
						  	  virtual_point_ecef[1] += center_XYZ[1];																// tahmini noktanin merkeze Y uzakligi

						  	  float virtual_point_lat_lon_alt[2];
						  	  ECEF2LLA(virtual_point_ecef[0], virtual_point_ecef[1], center_XYZ[2], virtual_point_lat_lon_alt);   // Tahminin noktanin enlem ve boylami

						  	  last_waypoint.enlem = virtual_point_lat_lon_alt[0];
						  	  last_waypoint.boylam = virtual_point_lat_lon_alt[1];

						  	  estimation_point_gonder(virtual_point_lat_lon_alt[0],  virtual_point_lat_lon_alt[1],&paket_sayaci);

						  	  // current point ile tahmini noktanin arasindaki aci
						  	  comm.simulation.PID.heading_ref = get_bearing (current_waypoint.enlem, current_waypoint.boylam, virtual_point_lat_lon_alt[0], virtual_point_lat_lon_alt[1]);
						  	  // hedef yukseklik
						  	  comm.simulation.PID.alt_ref = target_waypoint.yukseklik;
						  }


						if(!(GL.pid_cevrim_suresi % 2))
						{
							PID_Control(&PID_t.Altitude, comm.simulation.PID.alt_ref,     comm.simulation.rakim,    comm.simulation.PID.alt_kp,     comm.simulation.PID.alt_ki,     comm.simulation.PID.alt_kd);
							PID_Control(&PID_t.Heading,  comm.simulation.PID.heading_ref, comm.simulation.heading, 	comm.simulation.PID.heading_kp, comm.simulation.PID.heading_ki, comm.simulation.PID.heading_kd);

							GL.pid_cevrim_suresi = 0;
						}

							PID_CasCade_Control(MODE_AUTO, &PID_t.Cascade_Altitude, PID_t.Altitude,  PID_t.Pitch, rc_0,  comm.simulation.pitch, comm.simulation.PID.pitch_kp, comm.simulation.PID.pitch_ki, comm.simulation.PID.pitch_kd);
							PID_CasCade_Control(MODE_AUTO, &PID_t.Cascade_Heading,  PID_t.Heading,   PID_t.Roll,  rc_0,  comm.simulation.roll,  comm.simulation.PID.roll_kp,  comm.simulation.PID.roll_ki,  comm.simulation.PID.roll_kd);

					}
					PID_t.Take_Off_t.flight_mode.mode_flag = MODE_AUTO;
					break;
				}
			}

			if(!(GL.uart_cevrim_suresi % 1))
			{
				waypoint_paket_olustur(&paket_sayaci);
				RC_paket_olustur(&paket_sayaci);
				PID_paket_olustur(&paket_sayaci);
			}

			if(!(GL.uart_cevrim_suresi % 2))
			{
				MOD_paket_olustur(&paket_sayaci);
			}

			if(comm.flag.UART_READY)
			{
				AFC_paket_gonder(paket_sayaci);
				HAL_UART_Receive_IT(&huart2,&comm.gelen_veri_u8,sizeof(comm.gelen_veri_u8));
				paket_sayaci = 0;
			}

#ifdef GERCEK_ZAMANLI
			pw_calc();
#endif
			tick.task_20hz = 0;
		} /*  if(tick.task_20hz) */

		if(tick.task_10hz)
		{
#ifdef GERCEK_ZAMANLI
			/* Ivmeolcer kalibrasyon modu */
			if(comm.calib_mode.accel)
			{
				uint8_t paket_sayac = 0;
				CALIB_LED_ENABLE
				mpu6050_accelCalibration(1000);
				accel_kalibrasyon_paket_olustur(&paket_sayac);
				HAL_UART_Transmit(&huart2,comm.giden_veri_u8,paket_sayac,10);
				CALIB_LED_DISABLE
				comm.calib_mode.accel = 0;
			} /* (comm.mode.accel_calib_mode) */
			else
			{

			}
#endif
			tick.task_10hz = 0;
		} /*  if(tick.task_10hz) */

		if(tick.task_5hz)
		{
#ifdef GERCEK_ZAMANLI
			if(GPS.gps_baglanti_durumu == true && GPS.Safe_Data.satellite >= 4)
			{
				GPS.gps_baglanti_durumu = false;
				GPS_LED_ENABLE
			}
			else
			{
				GPS_LED_DISABLE
			}
#endif
			tick.task_5hz = 0;
		} /*  if(tick.task_5hz) */

		if(tick.task_1hz)
		{



			tick.task_1hz = 0;
		} /*  if(tick.task_1hz) */



		if(tick.task_05hz)
		{

			flag_land = ~flag_land;

			//** L A N D I N G **//
			if (WaypointStatus == LANDING)
				PID_t.Landing_t.Landing_Param.alt_time++;

			tick.task_05hz = 0;
		} /*  if(tick.task_05hz) */


	}
}
