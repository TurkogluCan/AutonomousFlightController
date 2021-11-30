/*
 * GPS.c
 *
 *  Created on: 19 Eki 2019
 *      Author: ecT
 */


// - - - - Include Library Here - - - -
//#include "gps.h"
//#include "defines.h"
//#include "math.h"

#include "common.h"
// - - - - - - - - - - - - - - - - - - -

/*//////////////////////////////////// F U N C T I O N S \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*/
void Initialise_Handler(void)													// ->> initial values
{
	GPS.Flag.case_central	= 58;
	GPS.Flag.start			= true;
	GPS.Flag.ack			= false;
}


void Clear_Tag_Counter(void)													// !!! Maybe input data can be struct (GPS Struct)->> Clear array counter of Tag Data
{
	GPS.Tag.GNVTG.cnt_array = 0;
	GPS.Tag.GNGGA.cnt_array = 0;
	GPS.Tag.GNGLL.cnt_array = 0;
	GPS.Tag.GNRMC.cnt_array = 0;
}


uint16_t Tag_Data_Sum(uint8_t *tag_data)										// ->> Organize by Classes
{
	uint8_t 	cnt_data_sum 	= 0;
	uint16_t	sum_data 		= 0;

	for(cnt_data_sum = 0; cnt_data_sum < 5; cnt_data_sum++)
		sum_data +=  tag_data[cnt_data_sum];

	return sum_data;
}




double Latitude_Calculator(char* raw_latitude)								// Function input must be STRUCT !!!
{
	char received_latitude[] = {raw_latitude[0],raw_latitude[1],raw_latitude[2],raw_latitude[3],raw_latitude[4],raw_latitude[5],raw_latitude[6],raw_latitude[7],raw_latitude[8],raw_latitude[9]};
	struct{
		uint8_t     latitude_deg;
		double 	    latitude_M;
		double 		latitude_mM;
		double 		latitude_m;
		double		latitude;

		uint8_t		cnt_array;
	}CompData_t;	// Computing Data


	for(CompData_t.cnt_array = 0; CompData_t.cnt_array < 10; CompData_t.cnt_array++)
		received_latitude[CompData_t.cnt_array] -= 48;																	// Char to decimal


	//** Format of receiving latitude data ---> ddMM.mmmmm

	CompData_t.latitude_deg	= (received_latitude[0] * 10) + received_latitude[1];
	CompData_t.latitude_M	= (received_latitude[2] * 10) + received_latitude[3];
	CompData_t.latitude_m	= (received_latitude[5]*10000 + received_latitude[6] *1000 + received_latitude[7] *100 + received_latitude[8] *10 + received_latitude[9] *1);
	CompData_t.latitude_mM	= (CompData_t.latitude_M + (CompData_t.latitude_m *0.01)*0.001);

	CompData_t.latitude 	= CompData_t.latitude_deg + (CompData_t.latitude_mM / 60);

	return CompData_t.latitude;
}


double Longitude_Calculator(char* raw_longitude)								// Function input must be STRUCT !!!
{
	char received_longitude[] = {raw_longitude[0],raw_longitude[1],raw_longitude[2],raw_longitude[3],raw_longitude[4],raw_longitude[5],raw_longitude[6],raw_longitude[7],raw_longitude[8],raw_longitude[9],raw_longitude[10]};

	struct{
		uint8_t     longitude_deg;
		double 	    longitude_M;
		double 		longitude_mM;
		double 		longitude_m;
		double		longitude;

		uint8_t		cnt_array;
	}CompData_t;


	for(CompData_t.cnt_array = 0; CompData_t.cnt_array <= 10; CompData_t.cnt_array++)
		received_longitude[CompData_t.cnt_array] -= 48;


	//** Format of receiving latitude data ---> dddMM.mmmmm

	CompData_t.longitude_deg	= received_longitude[0] *100 + received_longitude[1] *10 + received_longitude[2];
	CompData_t.longitude_M		= (received_longitude[3] * 10) + received_longitude[4];
	CompData_t.longitude_m		= (received_longitude[6]*10000 + received_longitude[7] *1000 + received_longitude[8] *100 + received_longitude[9] *10 + received_longitude[10] *1);
	CompData_t.longitude_mM		= (CompData_t.longitude_M + (CompData_t.longitude_m *0.01)*0.001);

	CompData_t.longitude 		= CompData_t.longitude_deg + (CompData_t.longitude_mM / 60);

	return CompData_t.longitude;
}


float Altitude_Calculator(char raw_altitude[5])
{
	char 	received_altitude[] = {raw_altitude[0],raw_altitude[1],raw_altitude[2],raw_altitude[3],raw_altitude[4],raw_altitude[5]};
	uint8_t cnt_array 	= 0;;
	uint8_t seq_dot 	= 0;
	float 	altitude 	= 0;
	int 	i = 0;
	float 	j = 100;
	for(cnt_array = 0; cnt_array <= 5; cnt_array++)
	{
		if(received_altitude[cnt_array] == '.')
			seq_dot = cnt_array;

		received_altitude[cnt_array] -= 48;
	}


	while(j>=0.1)
	{
		if(i != seq_dot){

			altitude += received_altitude[i]*j;
			j/=10;
		}
		i++;
	}
	return altitude;
}


uint8_t Satellite_Calculator(char raw_satellite[2])
{
	char received_satellite[] = {raw_satellite[0],raw_satellite[1]};
	uint8_t		cnt_array;
	uint8_t		satellite;

	for(cnt_array = 0; cnt_array <= 1; cnt_array++)
		received_satellite[cnt_array] -= 48;

	satellite = received_satellite[0]*10 + received_satellite[1];


	return satellite;
}


float Speed_Calculator(char raw_speed[7])
{
	char received_speed[] = {raw_speed[0],raw_speed[1],raw_speed[2],raw_speed[3],raw_speed[4],raw_speed[5],raw_speed[6]};
	uint8_t		cnt_array;
	uint8_t		seq_dot = 0;
	float		speed 	= 0;
	uint8_t		flag = true;
	for(cnt_array = 0; cnt_array <= 6; cnt_array++)
	{
		if(received_speed[cnt_array] != 0)
		{
			if(received_speed[cnt_array] == '.')
				seq_dot = cnt_array;

			received_speed[cnt_array] -= 48;
		}
	}

	// Noktayi diziye almadan da yapilabilir
	// 123.456


	for(int sayac = 0; sayac <= 6; sayac++){

		if(sayac == seq_dot && flag == true)	{
			seq_dot++; sayac++;
			flag = false;
		}


		speed += received_speed[sayac]*pow(10,seq_dot-sayac-1);
	}


	flag = true;

	return speed;
}



void Separation_GNVTG(uint8_t cnt_gnvtg)										// ->> GNVTG Function
{

	switch (cnt_gnvtg)
	{

		case ( 0 ):{			// Course over ground (true)

			GPS.Tag.GNVTG.course_gnd[GPS.Tag.GNVTG.cnt_array] = GPS.Data.receive;

			GPS.Tag.GNVTG.cnt_array++;
			break;
		}

		case ( 1 ):{			// Fixed field: true
			GPS.Tag.GNVTG.fixed_crs_gnd = GPS.Data.receive;

			break;
		}

		case ( 2 ):{			// Course over ground (magnetic), NOT OUTPUT !!!!

			GPS.Tag.GNVTG.course_gnd_mag[GPS.Tag.GNVTG.cnt_array] = GPS.Data.receive;

			GPS.Tag.GNVTG.cnt_array++;
			break;
		}

		case ( 3 ):{			// Fixed field: magnetic

			GPS.Tag.GNVTG.fixed_crs_mag = GPS.Data.receive;

			break;
		}

		case ( 4 ):{			// Speed over ground - knots

			GPS.Tag.GNVTG.speed_gnd_knt[GPS.Tag.GNVTG.cnt_array] = GPS.Data.receive;

			GPS.Tag.GNVTG.cnt_array++;
			break;
		}

		case ( 5 ):{			// Fixed field: knots

			GPS.Tag.GNVTG.fixed_knt = GPS.Data.receive;

			break;
		}

		case ( 6 ):{			// Speed over ground - km/h

			GPS.Tag.GNVTG.speed_gnd_kmh[GPS.Tag.GNVTG.cnt_array] = GPS.Data.receive;


			GPS.Tag.GNVTG.cnt_array++;
			break;
		}

		case ( 7 ):{			// Fixed field: kilometers per hour

			GPS.Tag.GNVTG.flag_speed = true;
			GPS.Tag.GNVTG.fixed_kmh = GPS.Data.receive;

			break;
		}

		case ( 8 ):{			// Mode Indicator

			GPS.Tag.GNVTG.mode = GPS.Data.receive;

			break;
		}
	}
}


void Separation_GNGGA(uint8_t cnt_gngga)										// ->> GNGGA Function
{
	switch (cnt_gngga)
	{

		case ( 0 ):{													// UTC time
			GPS.Tag.GNGGA.time_utc[GPS.Tag.GNGGA.cnt_array] = GPS.Data.receive;

			GPS.Tag.GNGGA.cnt_array++;
			break;
		}

		case ( 1 ):{													// Latitude (degrees & minutes)
			GPS.Tag.GNGGA.latitude[GPS.Tag.GNGGA.cnt_array] = GPS.Data.receive;

			GPS.Tag.GNGGA.cnt_array++;
			break;
		}

		case ( 2 ):{													// North/South indicator
			GPS.Tag.GNGGA.indicator_NS = GPS.Data.receive;

			break;
		}

		case ( 3 ):{													// Longitude (degrees & minutes)
			GPS.Tag.GNGGA.longitude[GPS.Tag.GNGGA.cnt_array] = GPS.Data.receive;

			GPS.Tag.GNGGA.cnt_array++;
			break;
		}

		case ( 4 ):{													// East/West indicator
			GPS.Tag.GNGGA.indicator_EW = GPS.Data.receive;

			break;
		}

		case ( 5 ):{													// Quality indicator for position fix
			GPS.Tag.GNGGA.indicator_quality_pos = GPS.Data.receive;

			break;
		}

		case ( 6 ):{													// Number of satellites used (range: 0-12)
			GPS.Tag.GNGGA.sattle_num[GPS.Tag.GNGGA.cnt_array] = GPS.Data.receive;

			GPS.Tag.GNGGA.flag_satellite = true;
			GPS.Tag.GNGGA.cnt_array++;
			break;
		}

		case ( 7 ):{													// Horizontal Dilution of Precision
			GPS.Tag.GNGGA.horz_prec[GPS.Tag.GNGGA.cnt_array] = GPS.Data.receive;

			GPS.Tag.GNGGA.cnt_array++;
			break;
		}

		case ( 8 ):{													// Altitude above mean sea level
			GPS.Tag.GNGGA.altitude_sea[GPS.Tag.GNGGA.cnt_array] = GPS.Data.receive;

			GPS.Tag.GNGGA.cnt_array++;
			break;
		}

		case ( 9 ):{													// Altitude units: M (meters, fixed field)

			GPS.Tag.GNGGA.flag_altitude = true;
			GPS.Tag.GNGGA.unit_altitude = GPS.Data.receive;

			break;
		}

		case ( 10 ):{													// Geoid separation: difference between ellipsoid and mean sea level
			GPS.Tag.GNGGA.geoid_seperation[GPS.Tag.GNGGA.cnt_array] = GPS.Data.receive;

			GPS.Tag.GNGGA.cnt_array++;
			break;
		}

		case ( 11 ):{													// Geoid separation units: M (meters, fixed field)
			GPS.Tag.GNGGA.unit_geoid = GPS.Data.receive;

			break;
		}

		case ( 12 ):{													// Maybe NOT USE ',,' !!! -  Age of differential corrections (null whenDGPS is not used)
			GPS.Tag.GNGGA.diff_age[GPS.Tag.GNGGA.cnt_array] = GPS.Data.receive;

			GPS.Tag.GNGGA.cnt_array++;
			break;
		}

		case ( 13 ):{													// ID of station providing differential corrections(null when DGPS is not used)
			GPS.Tag.GNGGA.diff_id[GPS.Tag.GNGGA.cnt_array] = GPS.Data.receive;

			GPS.Tag.GNGGA.cnt_array++;
			break;
		}
	}
}


void Separation_GNGLL(uint8_t cnt_gngll)										// ->> GNGLL Function
{

	switch (cnt_gngll)
	{

		case ( 0 ):{			// Latitude, Degrees + minutes

			GPS.Tag.GNGLL.latitude[GPS.Tag.GNGLL.cnt_array] = GPS.Data.receive;

			GPS.Tag.GNGLL.cnt_array++;
			break;
		}

		case ( 1 ):{			// N/S Indicator, hemisphere N=north or S=south
			GPS.Tag.GNGLL.indicator_NS = GPS.Data.receive;

			break;
		}

		case ( 2 ):{			// Longitude, Degrees + minutes,

			GPS.Tag.GNGLL.longitude[GPS.Tag.GNGLL.cnt_array] = GPS.Data.receive;

			GPS.Tag.GNGLL.cnt_array++;
			break;
		}

		case ( 3 ):{			// E/W indicator, E=east or W=west

			GPS.Tag.GNGLL.indicator_EW = GPS.Data.receive;

			break;
		}

		case ( 4 ):{			// UTC Time, Current time

			GPS.Tag.GNGLL.time_utc[GPS.Tag.GNGLL.cnt_array] = GPS.Data.receive;

			GPS.Tag.GNGLL.cnt_array++;
			break;
		}

		case ( 5 ):{			// V = Data invalid or receiver warning, A = Data valid

			GPS.Tag.GNGLL.cntrl_valid = GPS.Data.receive;

			break;
		}

		case ( 6 ):{			// Positioning Mode

			GPS.Tag.GNGLL.mode_pos = GPS.Data.receive;

			break;
		}
	}
}


void Separation_GNRMC(uint8_t cnt_gnrmc)										// ->> GNRMC Function
{

	switch (cnt_gnrmc)
	{

		case ( 0 ):{			// UTC Time, Time of position fix

			GPS.Tag.GNRMC.time_utc[GPS.Tag.GNRMC.cnt_array] = GPS.Data.receive;

			GPS.Tag.GNRMC.cnt_array++;
			break;
		}

		case ( 1 ):{			// Status, V = Navigation receiver warning, A = Datavalid
			GPS.Tag.GNRMC.status = GPS.Data.receive;

			break;
		}

		case ( 2 ):{			// Latitude, Degrees + minutes,

			GPS.Tag.GNRMC.latitude[GPS.Tag.GNRMC.cnt_array] = GPS.Data.receive;

			GPS.Tag.GNRMC.cnt_array++;
			break;
		}

		case ( 3 ):{			// N/S indicator, hemisphere N=north or S=south

			GPS.Tag.GNRMC.flag_latitude 	= true;											// Latitude Data came - Data maybe be empty or not
			GPS.Tag.GNRMC.indicator_NS 	= GPS.Data.receive;

			break;
		}

		case ( 4 ):{			// Longitude, Degrees + minutes,

			GPS.Tag.GNRMC.longitude[GPS.Tag.GNRMC.cnt_array] = GPS.Data.receive;

			GPS.Tag.GNRMC.cnt_array++;
			break;
		}

		case ( 5 ):{			// E/W indicator, E=east or W=west

			GPS.Tag.GNRMC.flag_longitude 	= true;											// Longitude Data came - Data maybe be empty or not
			GPS.Tag.GNRMC.indicator_EW	= GPS.Data.receive;

			break;
		}

		case ( 6 ):{			// Speed over ground

			GPS.Tag.GNRMC.speed_gnd[GPS.Tag.GNRMC.cnt_array] = GPS.Data.receive;

			GPS.Tag.GNRMC.cnt_array++;
			break;
		}

		case ( 7 ):{			// Course over ground

			GPS.Tag.GNRMC.course_gnd[GPS.Tag.GNRMC.cnt_array] = GPS.Data.receive;

			GPS.Tag.GNRMC.cnt_array++;
			break;
		}

		case ( 8 ):{			// Date in day, month, year format

			GPS.Tag.GNRMC.date_utc[GPS.Tag.GNRMC.cnt_array] = GPS.Data.receive;

			GPS.Tag.GNRMC.cnt_array++;
			break;
		}

		case ( 9 ):{			// ? ? ? ? ?  Unnecessary information

			GPS.Tag.GNRMC.mag_var[GPS.Tag.GNRMC.cnt_array] = GPS.Data.receive;

			GPS.Tag.GNRMC.cnt_array++;
			break;
		}

		case ( 10 ):{			// ? ? ? ? ?  Unnecessary information

			GPS.Tag.GNRMC.indicator_mag = GPS.Data.receive;

			break;
		}

		case ( 11 ):{			// Mode indicator

			GPS.Tag.GNRMC.indicator_mode = GPS.Data.receive;

			break;
		}
	}
}


void Tag_Separation(uint16_t SumData)											// ->> Separates by the Tag Data
{

	if( (GPS.Data.receive == Comma) && (GPS.Tag.GNTXT.flag_receive != true) ){				// Counter clear , Case sequence , ACK data flag operations and Don't receive GNTXT data
		GPS.Count.tag_case++;
		Clear_Tag_Counter();
	}

	if( (GPS.Data.receive == Star) && (GPS.Tag.GNTXT.flag_receive != true) ){					// Don't receive the ACK data Because ACK data is none of my business - Reset GPS.Count.tag_case because data package is over ,  Don't receive GNTXT data
		GPS.Flag.ack 		= true;
		GPS.Count.tag_case 	= 0;

		Clear_Tag_Counter();
	}



	if(GPS.Data.receive != Comma && GPS.Flag.ack != true)									// if comma data didn't come, don't get in - - - Don't receive ACK data
	{
		switch ( SumData ){

			case ( 390 ):{	// GNVTG

				Separation_GNVTG(GPS.Count.tag_case);
				break;
			}

			case ( 356 ):{	// GNGGA

				Separation_GNGGA(GPS.Count.tag_case);
				break;
			}

			case ( 372 ):{	// GNGLL

				Separation_GNGLL(GPS.Count.tag_case);
				break;
			}

			case ( 375 ):{	// GNRMC

				Separation_GNRMC(GPS.Count.tag_case);
				break;
			}

			case ( 405 ):{	// GNTXT

				GPS.Tag.GNTXT.flag_receive = true;
				break;
			}
		}
	}
}


void GPS_main(void)
{
	if(GPS.Data.receive == Dollar && GPS.Flag.start == true){

		GPS.Flag.case_central 	= 0;
		GPS.Flag.start 			= false;
	}



	switch (GPS.Flag.case_central){
		// - - - - C A S E ( 0 )
		case (0):																								// 'Tag' --> Case of Tag
		{
			if( (GPS.Data.receive != Comma) && (GPS.Data.receive != Dollar) && (GPS.Count.tag < 5) ){								// not accept another than ',' or '$' and cnt_tag is smaller than 5

				GPS.Data.tag[GPS.Count.tag]	= GPS.Data.receive;																// Data of Tag episode
				GPS.Count.tag++;
			}


			if(GPS.Data.receive == Comma && GPS.Count.tag == 5){															// if meaningless data came when tag data time, discontinue (count.tag == 5) && if comma come (GPS.Data.receive == ',')

				GPS.Data.sum 			= Tag_Data_Sum(GPS.Data.tag);
																															// Clear counter and Set case sequence
				GPS.Count.tag 			= 0;
				GPS.Flag.case_central 	= 1;																				// Must be another flag_case
			}

			break;
		}

		// - - - - C A S E ( 1 )
		case (1):																								// ',' --> Case Separating by comma, Case accepts after the comma data, because comma data doesn't matter to Tag_Separation fcn.
		{
			Tag_Separation(GPS.Data.sum);

			break;
		}
	}




	// FIX THE BUG - (array length)
	// - - - - - BIG DATA - - - A N D - - - END OF FRAME - - - - -
	GPS.Data.big[GPS.Count.bigdata] 	= GPS.Data.receive;																		// All of Data

	if( (GPS.Data.big[GPS.Count.bigdata-1] == CR_) && (GPS.Data.big[GPS.Count.bigdata] == LF) )									// END of DATA PACKAGE *** <CR><LF>
	{
		GPS.Flag.start				= true;
		GPS.Flag.ack				= false;																					// Separation fcn. --- Not enter Tag data separation case
		GPS.Tag.GNTXT.flag_receive  = false;
		GPS.Data.sum				= 0;
		GPS.Count.bigdata 			= -1;
	}

	GPS.Count.bigdata++;
	if(GPS.Count.bigdata > 253)	GPS.Count.bigdata = 0;
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -



	// - - - - - - S A F E - D A T A - P R O C E S S - - - - - - -
	if( (GPS.Tag.GNRMC.flag_latitude == true) && (GPS.Tag.GNRMC.latitude[4] == Dot) ){						// LOOK THE TAG which receiving latitude data

		GPS.Tag.GNRMC.flag_latitude 	= false;
		GPS.Safe_Data.latitude 			= Latitude_Calculator(GPS.Tag.GNRMC.latitude);
	}

	if ( (GPS.Tag.GNRMC.flag_longitude == true) && (GPS.Tag.GNRMC.longitude[5] == Dot) ){
		GPS.Tag.GNRMC.flag_longitude 	= false;
		GPS.Safe_Data.longitude			= Longitude_Calculator(GPS.Tag.GNRMC.longitude);
	}

	if( (GPS.Tag.GNGGA.flag_altitude == true) ){

		GPS.Tag.GNGGA.flag_altitude 	= false;
		GPS.Safe_Data.altitude 			= Altitude_Calculator(GPS.Tag.GNGGA.altitude_sea);

	}

	if( (GPS.Tag.GNGGA.flag_satellite == true) ){

		GPS.Tag.GNGGA.flag_satellite 	= false;
		GPS.gps_baglanti_durumu 		= true;
		GPS.Safe_Data.satellite 		= Satellite_Calculator(GPS.Tag.GNGGA.sattle_num);
	}

	if( (GPS.Tag.GNVTG.flag_speed == true) ){

		GPS.Tag.GNVTG.flag_speed 	= false;
		GPS.Safe_Data.speed 		= Speed_Calculator(GPS.Tag.GNVTG.speed_gnd_kmh);

	}

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -




}
/*//////////////////////////////////// - - - - - E N D  O F  F U N C T I O N S - - - - - \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\*/







