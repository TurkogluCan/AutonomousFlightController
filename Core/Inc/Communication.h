/*
 * Communication.h
 *
 *  Created on: 26 Kas 2019
 *      Author: Huseyin Koc
 */


#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

//#include "stdint.h"

#define START_OF_FRAME1 58
#define START_OF_FRAME2 34

struct data_frame_t
{
	uint8_t sof1;
	uint8_t sof2;
	uint8_t packet_counter;
	uint8_t packet_type;
	uint8_t packet_length;
	uint8_t data[255];
	uint8_t crc1;
	uint8_t crc2;
};

struct flag_t
{
	volatile uint8_t packet_end;
	volatile uint8_t UART_READY;
	volatile uint16_t abc;
};

struct calib_mode_t
{
	 uint8_t mag;
	 uint8_t accel;
	 uint8_t rc;
};

typedef struct
{
	float  roll_kp;
	float  roll_ki;
	float  roll_kd;

	float  pitch_kp;
	float  pitch_ki;
	float  pitch_kd;

	float  alt_kp;
	float  alt_ki;
	float  alt_kd;
	float  alt_ref;

	float  heading_kp;
	float  heading_ki;
	float  heading_kd;
	float  heading_ref;

	float  yaw_kp;
	float  yaw_ki;
	float  yaw_kd;

	float  takeoff_kp;
	float  takeoff_ki;
	float  takeoff_kd;

	float  throttle_kp;
	float  throttle_ki;
	float  throttle_kd;
}PID_Gain;


struct simulation_t
{
	float  enlem;
	float  boylam;
	float  rakim;
	float  yer_hizi;
	float  hava_hizi;

	float  heading;
	float  pitch;
	float  roll;
	float  altitude_from_ground;

	float RPM;


	PID_Gain PID;
};



enum packet_type_t
{
	IMU			 		= 11,
	GPS_ 				= 22,
	FLIGHT_MOD_STATUS	= 33,
	BATARYA			 	= 44,
	RC					= 55,
	SIM					= 58,
	MAG_CALIB 			= 66,
	ACCEL_CALIB			= 77,
	PARAM		        = 88,
	WAYPOINT			= 89,
	ESTIMATION			= 90,
	PID			        = 99
};

enum packet_durum_t
{
	SOF1 			= 0,
	SOF2 			= 1,
	PACKET_TYPE		= 2,
	PACKET_COUNTER 	= 3,
	PACKET_LENGTH 	= 4,
	DATA 			= 5,
	CRC1 			= 6,
	CRC2 			= 7
};

typedef struct
{
	enum	packet_type_t 	packet_type;
	enum 	packet_durum_t 	packet_durum;

	struct 	data_frame_t 	data_frame;
	struct 	flag_t  		flag;
	struct  calib_mode_t 	calib_mode;
	struct  simulation_t 	simulation;

	uint8_t giden_veri_u8[256];
	uint8_t gelen_veri_u8;
	uint8_t packet_counter;

	uint32_t CRC_hatali_paket;
	uint32_t BASLIK_hatali_paket;
}comm_t;

comm_t comm;

void paket_coz(comm_t* comm);
void IMU_paket_olustur(uint8_t* p_index);
void RC_paket_olustur(uint8_t* p_index);
void PID_paket_olustur(uint8_t* p_index);
void waypoint_paket_olustur(uint8_t* p_index);
void GPS_paket_olustur(uint8_t* p_index);
void MOD_paket_olustur(uint8_t* p_index);
void batarya_paket_olustur(uint8_t* p_index);
void AFC_paket_gonder(uint8_t veri_boyutu);
void mag_kalibrasyon_paket_olustur(uint8_t* p_index);
void accel_kalibrasyon_paket_olustur(uint8_t* p_index);
void estimation_point_gonder(float lat, float lon,uint8_t* p_index);
#endif /* COMMUNICATION_H_ */
