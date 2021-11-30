/*
 * Communication.c
 *
 *  Created on: 26 Kas 2019
 *      Author: Huseyin Koc
 */

#include "common.h"

/* UART Interrupt Callback Functions */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		if(!comm.flag.UART_READY)
		{
			comm.flag.UART_READY = 1;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		GPS_main();
		HAL_UART_Receive_IT(&huart1,&GPS.Data.receive,sizeof(GPS.Data.receive));
	}

	if(huart->Instance == USART2)
	{
//		comm.gelen_veri_u8 = USART2->DR;
		paket_coz(&comm);
		HAL_UART_Receive_IT(&huart2,&comm.gelen_veri_u8,sizeof(comm.gelen_veri_u8));
	}
}
/* UART Interrupt Callback Functions */


/**
  * @brief  IMU haberlesme paketini olusturur
  * @param  indeks
  * @retval (void)
  */
void IMU_paket_olustur(uint8_t* p_index)
{
	uint8_t index = *p_index;
	int16_t  roll_t,pitch_t;
	int16_t altitude_from_ground_t,sicaklik_t;
	uint16_t yaw_t;
	static uint8_t packet_counter = 0;
	uint8_t CRC_BUFFER_LENGTH = index + 2;	// +2 sebebi: sof1 ve sof2 eklenmez.
	uint16_t CRC_BUF = 0;

	roll_t 	= ANGLE.roll    * DEG2RAD * 10000;
	pitch_t = ANGLE.pitch   * DEG2RAD * 10000;
	yaw_t 	= ANGLE.heading * DEG2RAD * 10000;

	altitude_from_ground_t 	= ms5611.relative_altitude * 10;
	sicaklik_t 				= ms5611.temperature;

	comm.giden_veri_u8[index++] = START_OF_FRAME1;
	comm.giden_veri_u8[index++] = START_OF_FRAME2;
	comm.giden_veri_u8[index++] = IMU;
	comm.giden_veri_u8[index++] = packet_counter++;
	comm.giden_veri_u8[index++] = 10;

	INT16_ayir(comm.giden_veri_u8,&index,roll_t);
	INT16_ayir(comm.giden_veri_u8,&index,pitch_t);
	UINT16_ayir(comm.giden_veri_u8,&index,yaw_t);
	INT16_ayir(comm.giden_veri_u8,&index,altitude_from_ground_t);
	INT16_ayir(comm.giden_veri_u8,&index,sicaklik_t);

	while(CRC_BUFFER_LENGTH < index)
	{
		CRC_BUF += comm.giden_veri_u8[CRC_BUFFER_LENGTH++];
	}

	UINT16_ayir(comm.giden_veri_u8,&index,CRC_BUF);

	*p_index = index;
}

/**
  * @brief  GPS haberlesme paketini olusturur
  * @param  indeks
  * @retval (void)
  */
void GPS_paket_olustur(uint8_t* p_index)
{
	uint8_t index = *p_index;
	uint32_t enlem,boylam;
	uint16_t rakim;
	uint16_t yer_hizi;
	uint8_t uydu_sayisi;
	static uint8_t packet_counter = 0;
	uint8_t CRC_BUFFER_LENGTH = index + 2;
	uint16_t CRC_BUF = 0;

	enlem  = GPS.Safe_Data.latitude  * 1000000;
	boylam = GPS.Safe_Data.longitude * 1000000;
	rakim  = GPS.Safe_Data.altitude  * 10;

	uydu_sayisi = GPS.Safe_Data.satellite;
	yer_hizi = GPS.Safe_Data.speed * 0.277777778 * 100;

	comm.giden_veri_u8[index++] = START_OF_FRAME1;
	comm.giden_veri_u8[index++] = START_OF_FRAME2;
	comm.giden_veri_u8[index++] = GPS_;
	comm.giden_veri_u8[index++] = packet_counter++;
	comm.giden_veri_u8[index++] = 13;

	UINT32_ayir(comm.giden_veri_u8,&index,enlem);
	UINT32_ayir(comm.giden_veri_u8,&index,boylam);
	UINT16_ayir(comm.giden_veri_u8,&index,rakim);
	UINT8_ayir(comm.giden_veri_u8,&index,uydu_sayisi);
	UINT16_ayir(comm.giden_veri_u8,&index,yer_hizi);

	while(CRC_BUFFER_LENGTH < index)
	{
		CRC_BUF += comm.giden_veri_u8[CRC_BUFFER_LENGTH++];
	}

	UINT16_ayir(comm.giden_veri_u8,&index,CRC_BUF);

	*p_index = index;
}

/**
  * @brief  Flight MODE paketini olusturur
  * @param  indeks
  * @retval (void)
  */
void MOD_paket_olustur(uint8_t* p_index)
{
	uint8_t index = *p_index;
	static uint8_t packet_counter = 0;
	uint8_t CRC_BUFFER_LENGTH = index + 2;
	uint16_t CRC_BUF = 0;

	comm.giden_veri_u8[index++] = START_OF_FRAME1;
	comm.giden_veri_u8[index++] = START_OF_FRAME2;
	comm.giden_veri_u8[index++] = FLIGHT_MOD_STATUS;
	comm.giden_veri_u8[index++] = packet_counter++;
	comm.giden_veri_u8[index++] = 1;
	UINT8_ayir(comm.giden_veri_u8,&index,(uint8_t)FLIGHT_MODE);

	while(CRC_BUFFER_LENGTH < index)
	{
		CRC_BUF += comm.giden_veri_u8[CRC_BUFFER_LENGTH++];
	}

	UINT16_ayir(comm.giden_veri_u8,&index,CRC_BUF);

	*p_index = index;
}

/**
  * @brief  Batarya bilgisinin paketini olusturur
  * @param  indeks
  * @retval (void)
  */
void batarya_paket_olustur(uint8_t* p_index)
{
	uint8_t index = *p_index;
	static uint8_t packet_counter = 0;
	uint8_t CRC_BUFFER_LENGTH = index + 2;
	uint16_t CRC_BUF = 0;

	uint16_t volt,amper;
	volt  = pw.filt_volt  * 100;
	amper = pw.filt_amper * 100;

	comm.giden_veri_u8[index++] = START_OF_FRAME1;
	comm.giden_veri_u8[index++] = START_OF_FRAME2;
	comm.giden_veri_u8[index++] = BATARYA;
	comm.giden_veri_u8[index++] = packet_counter++;
	comm.giden_veri_u8[index++] = 4;

	UINT16_ayir(comm.giden_veri_u8,&index,volt);
	UINT16_ayir(comm.giden_veri_u8,&index,amper);

	while(CRC_BUFFER_LENGTH < index)
	{
		CRC_BUF += comm.giden_veri_u8[CRC_BUFFER_LENGTH++];
	}

	UINT16_ayir(comm.giden_veri_u8,&index,CRC_BUF);

	*p_index = index;
}

/**
  * @brief  RC haberlesme paketini olusturur
  * @param  indeks
  * @retval (void)
  */
void RC_paket_olustur(uint8_t* p_index)
{
	uint8_t index = *p_index;
	static uint8_t packet_counter = 0;
	uint8_t CRC_BUFFER_LENGTH = index + 2;
	uint16_t CRC_BUF = 0;

	comm.giden_veri_u8[index++] = START_OF_FRAME1;
	comm.giden_veri_u8[index++] = START_OF_FRAME2;
	comm.giden_veri_u8[index++] = RC;
	comm.giden_veri_u8[index++] = packet_counter++;
	comm.giden_veri_u8[index++] = 16;

	UINT16_ayir(comm.giden_veri_u8,&index,ch.roll);
	UINT16_ayir(comm.giden_veri_u8,&index,ch.pitch);
	UINT16_ayir(comm.giden_veri_u8,&index,ch.throttle);
	UINT16_ayir(comm.giden_veri_u8,&index,ch.yaw);
	UINT16_ayir(comm.giden_veri_u8,&index,ch.swA);
	UINT16_ayir(comm.giden_veri_u8,&index,ch.swB);
	UINT16_ayir(comm.giden_veri_u8,&index,ch.swC);
	UINT16_ayir(comm.giden_veri_u8,&index,ch.swD);

	while(CRC_BUFFER_LENGTH < index)
	{
		CRC_BUF += comm.giden_veri_u8[CRC_BUFFER_LENGTH++];
	}

	UINT16_ayir(comm.giden_veri_u8,&index,CRC_BUF);

	*p_index = index;
}

/**
  * @brief  PID kontrol sinyali paketini olusturur
  * @param  indeks
  * @retval (void)
  */
void PID_paket_olustur(uint8_t* p_index)
{

	uint8_t index = *p_index;
	static uint8_t packet_counter = 0;
	uint8_t CRC_BUFFER_LENGTH = index + 2;
	uint16_t CRC_BUF = 0;

	comm.giden_veri_u8[index++] = START_OF_FRAME1;
	comm.giden_veri_u8[index++] = START_OF_FRAME2;
	comm.giden_veri_u8[index++] = PID;
	comm.giden_veri_u8[index++] = packet_counter++;
	comm.giden_veri_u8[index++] = 36;


	INT16_ayir(comm.giden_veri_u8,&index,(int16_t)PID_t.Roll.setpoint);
	INT16_ayir(comm.giden_veri_u8,&index,(int16_t)PID_t.Roll.error);
	INT16_ayir(comm.giden_veri_u8,&index,(int16_t)PID_t.Roll.control);

	INT16_ayir(comm.giden_veri_u8,&index,(int16_t)PID_t.Pitch.setpoint);
	INT16_ayir(comm.giden_veri_u8,&index,(int16_t)PID_t.Pitch.error);
	INT16_ayir(comm.giden_veri_u8,&index,(int16_t)PID_t.Pitch.control);

	INT16_ayir(comm.giden_veri_u8,&index,(int16_t)PID_t.Altitude.setpoint);
	INT16_ayir(comm.giden_veri_u8,&index,(int16_t)PID_t.Altitude.error);
	INT16_ayir(comm.giden_veri_u8,&index,(int16_t)PID_t.Altitude.control);

	INT16_ayir(comm.giden_veri_u8,&index,(int16_t)PID_t.Heading.setpoint);
	INT16_ayir(comm.giden_veri_u8,&index,(int16_t)PID_t.Heading.error);
	INT16_ayir(comm.giden_veri_u8,&index,(int16_t)PID_t.Heading.control);

	INT16_ayir(comm.giden_veri_u8,&index,(int16_t)PID_t.Cascade_Altitude.setpoint);
	INT16_ayir(comm.giden_veri_u8,&index,(int16_t)PID_t.Cascade_Altitude.error);
	INT16_ayir(comm.giden_veri_u8,&index,(int16_t)PID_t.Cascade_Altitude.control);

	INT16_ayir(comm.giden_veri_u8,&index,(int16_t)PID_t.Cascade_Heading.setpoint);
	INT16_ayir(comm.giden_veri_u8,&index,(int16_t)PID_t.Cascade_Heading.error);
	INT16_ayir(comm.giden_veri_u8,&index,(int16_t)PID_t.Cascade_Heading.control);


	if(PID_t.Pitch.error_total > 1000)
		PID_t.Pitch.error_total = PID_t.Pitch.error_total;

	while(CRC_BUFFER_LENGTH < index)
	{
		CRC_BUF += comm.giden_veri_u8[CRC_BUFFER_LENGTH++];
	}

	UINT16_ayir(comm.giden_veri_u8,&index,CRC_BUF);

	*p_index = index;

}

void waypoint_paket_olustur(uint8_t* p_index)
{
	uint8_t index = *p_index;
	static uint8_t packet_counter = 0;
	uint8_t CRC_BUFFER_LENGTH = index + 2;
	uint16_t CRC_BUF = 0;

	comm.giden_veri_u8[index++] = START_OF_FRAME1;
	comm.giden_veri_u8[index++] = START_OF_FRAME2;
	comm.giden_veri_u8[index++] = WAYPOINT;
	comm.giden_veri_u8[index++] = packet_counter++;
	comm.giden_veri_u8[index++] = 3;

	UINT8_ayir(comm.giden_veri_u8,&index,target_waypoint.id);
	UINT8_ayir(comm.giden_veri_u8,&index,current_waypoint.id);
	UINT8_ayir(comm.giden_veri_u8,&index,(uint8_t)WaypointStatus);

	while(CRC_BUFFER_LENGTH < index)
	{
		CRC_BUF += comm.giden_veri_u8[CRC_BUFFER_LENGTH++];
	}

	UINT16_ayir(comm.giden_veri_u8,&index,CRC_BUF);

	*p_index = index;
}

void estimation_point_gonder(float lat, float lon,uint8_t* p_index)
{
	uint8_t index = *p_index;
	static uint8_t packet_counter = 0;
	uint8_t CRC_BUFFER_LENGTH = index + 2;
	uint16_t CRC_BUF = 0;

	comm.giden_veri_u8[index++] = START_OF_FRAME1;
	comm.giden_veri_u8[index++] = START_OF_FRAME2;
	comm.giden_veri_u8[index++] = ESTIMATION;
	comm.giden_veri_u8[index++] = packet_counter++;
	comm.giden_veri_u8[index++] = 8;

	FLOAT32_ayir(comm.giden_veri_u8,&index,lat);
	FLOAT32_ayir(comm.giden_veri_u8,&index,lon);

	while(CRC_BUFFER_LENGTH < index)
	{
		CRC_BUF += comm.giden_veri_u8[CRC_BUFFER_LENGTH++];
	}

	UINT16_ayir(comm.giden_veri_u8,&index,CRC_BUF);

	*p_index = index;
}


/**
  * @brief  MAG Kalibrasyon paketini olusturur
  * @param  indeks
  * @retval (void)
  */
void mag_kalibrasyon_paket_olustur(uint8_t* p_index)
{
	uint8_t index = *p_index;
	static uint8_t packet_counter = 0;
	uint8_t CRC_BUFFER_LENGTH = index + 2;
	uint16_t CRC_BUF = 0;
	int16_t 	mag_kalib_offset[3];
	uint16_t	mag_kalib_sf[3];

	mag_kalib_offset[0] = qmc5883.calib.mag_offset[0];
	mag_kalib_offset[1] = qmc5883.calib.mag_offset[1];
	mag_kalib_offset[2] = qmc5883.calib.mag_offset[2];

	mag_kalib_sf[0] = qmc5883.calib.mag_scale_factor[0] * 10000;
	mag_kalib_sf[1] = qmc5883.calib.mag_scale_factor[1] * 10000;
	mag_kalib_sf[2] = qmc5883.calib.mag_scale_factor[2] * 10000;

	comm.giden_veri_u8[index++] = START_OF_FRAME1;
	comm.giden_veri_u8[index++] = START_OF_FRAME2;
	comm.giden_veri_u8[index++] = MAG_CALIB;
	comm.giden_veri_u8[index++] = packet_counter++;
	comm.giden_veri_u8[index++] = 12;

	INT16_ayir(comm.giden_veri_u8,&index,mag_kalib_offset[0]);
	INT16_ayir(comm.giden_veri_u8,&index,mag_kalib_offset[1]);
	INT16_ayir(comm.giden_veri_u8,&index,mag_kalib_offset[2]);
	UINT16_ayir(comm.giden_veri_u8,&index,mag_kalib_sf[0]);
	UINT16_ayir(comm.giden_veri_u8,&index,mag_kalib_sf[1]);
	UINT16_ayir(comm.giden_veri_u8,&index,mag_kalib_sf[2]);


	while(CRC_BUFFER_LENGTH < index)
	{
		CRC_BUF += comm.giden_veri_u8[CRC_BUFFER_LENGTH++];
	}

	UINT16_ayir(comm.giden_veri_u8,&index,CRC_BUF);

	*p_index = index;
}

/**
  * @brief  ACCEL Kalibrasyon paketini olusturur
  * @param  indeks
  * @retval (void)
  */
void accel_kalibrasyon_paket_olustur(uint8_t* p_index)
{
	uint8_t index = *p_index;
	static uint8_t packet_counter = 0;
	uint8_t CRC_BUFFER_LENGTH = index + 2;
	uint16_t CRC_BUF = 0;
	int16_t accel_kalib_offset[3];

	accel_kalib_offset[0] = mpu6050.offset.ax;
	accel_kalib_offset[1] = mpu6050.offset.ay;
	accel_kalib_offset[2] = mpu6050.offset.az;

	comm.giden_veri_u8[index++] = START_OF_FRAME1;
	comm.giden_veri_u8[index++] = START_OF_FRAME2;
	comm.giden_veri_u8[index++] = ACCEL_CALIB;
	comm.giden_veri_u8[index++] = packet_counter++;
	comm.giden_veri_u8[index++] = 6;

	INT16_ayir(comm.giden_veri_u8,&index,accel_kalib_offset[0]);
	INT16_ayir(comm.giden_veri_u8,&index,accel_kalib_offset[1]);
	INT16_ayir(comm.giden_veri_u8,&index,accel_kalib_offset[2]);

	while(CRC_BUFFER_LENGTH < index)
	{
		CRC_BUF += comm.giden_veri_u8[CRC_BUFFER_LENGTH++];
	}

	UINT16_ayir(comm.giden_veri_u8,&index,CRC_BUF);

	*p_index = index;
}

/**
  * @brief  Gonderme islemini baslatir.
  * @param  boyut
  * @retval (void)
  */
void AFC_paket_gonder(uint8_t veri_boyutu)
{
	if(veri_boyutu > 0)
	{
		comm.flag.UART_READY = 0;
		HAL_UART_Transmit_IT(&huart2,comm.giden_veri_u8,veri_boyutu);
	}
}

/**
  * @brief  Haberlesme paketini cozer
  * @param  Communication struct type
  * @retval (void)
  */
void paket_coz(comm_t* comm)
{
    /******* HABERLESME DURUM MAKINASI *******/
	switch(comm->packet_durum)
	{
		static uint16_t CRC_BUFFER = 0;

		case SOF1:
		{
			if(comm->gelen_veri_u8 == START_OF_FRAME1)
			{
				comm->data_frame.sof1 = comm->gelen_veri_u8;
				comm->packet_durum = SOF2;
			}
			else
			{
				comm->BASLIK_hatali_paket++;
				comm->packet_durum = SOF1;
			}
			break;
		}
		case SOF2:
		{
			if(comm->gelen_veri_u8 == START_OF_FRAME2)
			{
				comm->data_frame.sof2 = comm->gelen_veri_u8;
				comm->packet_durum = PACKET_TYPE;
			}
			else
			{
				comm->packet_durum = SOF1;
				comm->BASLIK_hatali_paket++;
			}
			break;
		}
		case PACKET_TYPE:
		{
			comm->data_frame.packet_type = comm->gelen_veri_u8;
			CRC_BUFFER += comm->gelen_veri_u8;
			comm->packet_durum = PACKET_COUNTER;
			break;
		}
		case PACKET_COUNTER:
		{
			comm->data_frame.packet_counter = comm->gelen_veri_u8;
			CRC_BUFFER += comm->gelen_veri_u8;
			comm->packet_durum = PACKET_LENGTH;
			break;
		}
		case PACKET_LENGTH:
		{
			comm->data_frame.packet_length = comm->gelen_veri_u8;
			CRC_BUFFER += comm->gelen_veri_u8;
			comm->packet_durum = DATA;
			break;
		}
		case DATA:
		{
			static uint8_t DATA_LENGTH = 0;

			if(comm->data_frame.packet_length == 0)
			{
				comm->packet_durum = SOF1;
				DATA_LENGTH = 0;
			}
			else
			{
				comm->data_frame.data[DATA_LENGTH] = comm->gelen_veri_u8;
				DATA_LENGTH++;
				CRC_BUFFER += comm->gelen_veri_u8;

				if(DATA_LENGTH == comm->data_frame.packet_length)
				{
					comm->packet_durum = CRC1;
					DATA_LENGTH = 0;
				}
			}
			break;
		}
		case CRC1:
		{
			if(comm->gelen_veri_u8 == (CRC_BUFFER & 0xFF))
			{
				comm->data_frame.crc1 = comm->gelen_veri_u8;
				comm->packet_durum = CRC2;
			}
			else
			{
				comm->CRC_hatali_paket++;
				comm->packet_durum = SOF1;
				CRC_BUFFER = 0;

				if(comm->CRC_hatali_paket > 100)
					while(1);

			}
			break;
		}
		case CRC2:
		{
			if(comm->gelen_veri_u8 == ((CRC_BUFFER >> 8) & 0xFF))
			{
				comm->data_frame.crc2 = comm->gelen_veri_u8;
				CRC_BUFFER = 0;
				comm->packet_durum = SOF1;
				comm->flag.packet_end = 1;
			}
			else
			{
				comm->CRC_hatali_paket++;
				comm->packet_durum = SOF1;
				CRC_BUFFER = 0;

				if(comm->CRC_hatali_paket > 100)
					while(1);
			}

			break;
		}
	}
    /******* HABERLESME DURUM MAKINASI *******/


	/*************  PAKET BIRLESTIR  **************/
	if(comm->flag.packet_end)
	{
		comm->flag.packet_end = 0;
		uint8_t paket_sayac = 0; 	// kalibrasyon paketi gondermek icin olan sayac.
		uint8_t paket_indeks = 0;	// paket birlestir fonk icin olan sayac

		switch(comm->data_frame.packet_type)
		{
			case MAG_CALIB:
			{
				UINT8_birlestir(comm->data_frame.data,&paket_indeks, &comm->calib_mode.mag);

				if(!comm->calib_mode.mag)
				{
					mag_kalibrasyon_paket_olustur(&paket_sayac);
					HAL_UART_Transmit(&huart2,comm->giden_veri_u8,paket_sayac,10);
				}
				break;
			}
			case ACCEL_CALIB:
			{
				UINT8_birlestir(comm->data_frame.data,&paket_indeks, &comm->calib_mode.accel);
				break;

			}
			case RC:
			{
				UINT8_birlestir(comm->data_frame.data,&paket_indeks, &comm->calib_mode.rc);
				break;
			}
			case SIM:
			{
				FLOAT32_birlestir(comm->data_frame.data,&paket_indeks, &comm->simulation.hava_hizi);
				FLOAT32_birlestir(comm->data_frame.data,&paket_indeks, &comm->simulation.yer_hizi);
				FLOAT32_birlestir(comm->data_frame.data,&paket_indeks, &comm->simulation.enlem);
				FLOAT32_birlestir(comm->data_frame.data,&paket_indeks, &comm->simulation.boylam);
				FLOAT32_birlestir(comm->data_frame.data,&paket_indeks, &comm->simulation.rakim);
				FLOAT32_birlestir(comm->data_frame.data,&paket_indeks, &comm->simulation.altitude_from_ground);
				FLOAT32_birlestir(comm->data_frame.data,&paket_indeks, &comm->simulation.roll);
				FLOAT32_birlestir(comm->data_frame.data,&paket_indeks, &comm->simulation.pitch);
				FLOAT32_birlestir(comm->data_frame.data,&paket_indeks, &comm->simulation.heading);
				FLOAT32_birlestir(comm->data_frame.data,&paket_indeks, &comm->simulation.RPM);
				break;
			}
			case PID:
			{
				FLOAT32_birlestir(comm->data_frame.data,&paket_indeks, &comm->simulation.PID.roll_kp);
				FLOAT32_birlestir(comm->data_frame.data,&paket_indeks, &comm->simulation.PID.roll_ki);
				FLOAT32_birlestir(comm->data_frame.data,&paket_indeks, &comm->simulation.PID.roll_kd);

				FLOAT32_birlestir(comm->data_frame.data,&paket_indeks, &comm->simulation.PID.pitch_kp);
				FLOAT32_birlestir(comm->data_frame.data,&paket_indeks, &comm->simulation.PID.pitch_ki);
				FLOAT32_birlestir(comm->data_frame.data,&paket_indeks, &comm->simulation.PID.pitch_kd);

				FLOAT32_birlestir(comm->data_frame.data,&paket_indeks, &comm->simulation.PID.alt_kp);
				FLOAT32_birlestir(comm->data_frame.data,&paket_indeks, &comm->simulation.PID.alt_ki);
				FLOAT32_birlestir(comm->data_frame.data,&paket_indeks, &comm->simulation.PID.alt_kd);

				FLOAT32_birlestir(comm->data_frame.data,&paket_indeks, &comm->simulation.PID.heading_kp);
				FLOAT32_birlestir(comm->data_frame.data,&paket_indeks, &comm->simulation.PID.heading_ki);
				FLOAT32_birlestir(comm->data_frame.data,&paket_indeks, &comm->simulation.PID.heading_kd);

				FLOAT32_birlestir(comm->data_frame.data,&paket_indeks, &comm->simulation.PID.takeoff_kp);
				FLOAT32_birlestir(comm->data_frame.data,&paket_indeks, &comm->simulation.PID.takeoff_ki);
				FLOAT32_birlestir(comm->data_frame.data,&paket_indeks, &comm->simulation.PID.takeoff_kd);

				FLOAT32_birlestir(comm->data_frame.data,&paket_indeks, &comm->simulation.PID.throttle_kp);
				FLOAT32_birlestir(comm->data_frame.data,&paket_indeks, &comm->simulation.PID.throttle_ki);
				FLOAT32_birlestir(comm->data_frame.data,&paket_indeks, &comm->simulation.PID.throttle_kd);
				break;
			}
			case WAYPOINT:
			{
				UINT8_birlestir(comm->data_frame.data, &paket_indeks, &wp_sayisi);

				for(uint8_t i = 0; i < wp_sayisi; i++)
				{
					UINT8_birlestir  (comm->data_frame.data, &paket_indeks,&waypoints[i].id);
					FLOAT32_birlestir(comm->data_frame.data, &paket_indeks,&waypoints[i].enlem);
					FLOAT32_birlestir(comm->data_frame.data, &paket_indeks,&waypoints[i].boylam);
					UINT16_birlestir (comm->data_frame.data, &paket_indeks,&waypoints[i].yukseklik);
					UINT8_birlestir  (comm->data_frame.data, &paket_indeks,&waypoints[i].radius);
					UINT8_birlestir  (comm->data_frame.data, &paket_indeks,&waypoints[i].mission);

                    /* mission == 0 --> LOITER
                     * mission == 1 --> TAKEOFF
                     * mission == 2 --> WAYPOINT
                     * mission == 3 --> LANDING
                     */
				}

				break;
			}
			case PARAM:
			{
				INT16_birlestir(comm->data_frame.data,&paket_indeks,&mpu6050.offset.ax);
				INT16_birlestir(comm->data_frame.data,&paket_indeks,&mpu6050.offset.ay);
				INT16_birlestir(comm->data_frame.data,&paket_indeks,&mpu6050.offset.az);
				INT16_birlestir(comm->data_frame.data,&paket_indeks,&qmc5883.calib.mag_offset[0]);
				INT16_birlestir(comm->data_frame.data,&paket_indeks,&qmc5883.calib.mag_offset[1]);
				INT16_birlestir(comm->data_frame.data,&paket_indeks,&qmc5883.calib.mag_offset[2]);
				FLOAT32_birlestir(comm->data_frame.data,&paket_indeks,&qmc5883.calib.mag_scale_factor[0]);
				FLOAT32_birlestir(comm->data_frame.data,&paket_indeks,&qmc5883.calib.mag_scale_factor[1]);
				FLOAT32_birlestir(comm->data_frame.data,&paket_indeks,&qmc5883.calib.mag_scale_factor[2]);
				UINT16_birlestir(comm->data_frame.data,&paket_indeks,&ch.pitch_max);
				UINT16_birlestir(comm->data_frame.data,&paket_indeks,&ch.pitch_min);
				UINT16_birlestir(comm->data_frame.data,&paket_indeks,&ch.roll_max);
				UINT16_birlestir(comm->data_frame.data,&paket_indeks,&ch.roll_min);
				UINT16_birlestir(comm->data_frame.data,&paket_indeks,&ch.throttle_max);
				UINT16_birlestir(comm->data_frame.data,&paket_indeks,&ch.throttle_min);
				UINT16_birlestir(comm->data_frame.data,&paket_indeks,&ch.yaw_max);
				UINT16_birlestir(comm->data_frame.data,&paket_indeks,&ch.yaw_min);
				UINT16_birlestir(comm->data_frame.data,&paket_indeks,&ch.swA_max);
				UINT16_birlestir(comm->data_frame.data,&paket_indeks,&ch.swA_min);
				UINT16_birlestir(comm->data_frame.data,&paket_indeks,&ch.swB_max);
				UINT16_birlestir(comm->data_frame.data,&paket_indeks,&ch.swB_min);
				UINT16_birlestir(comm->data_frame.data,&paket_indeks,&ch.swC_max);
				UINT16_birlestir(comm->data_frame.data,&paket_indeks,&ch.swC_min);
				UINT16_birlestir(comm->data_frame.data,&paket_indeks,&ch.swD_max);
				UINT16_birlestir(comm->data_frame.data,&paket_indeks,&ch.swD_min);
				break;
			}

		} /* switch(comm->data_frame.packet_type) */

	} /* if(comm->flag.packet_end) */

	/*************  PAKET BIRLESTIR  **************/
}

void crc16_hesapla(uint8_t* veriler, uint8_t paket_sayac, uint16_t* crc16)
{
	crc16 = 0;

	for (uint8_t i = 0; i < paket_sayac; i++)
	{
		crc16 += veriler[i];
	}
}


