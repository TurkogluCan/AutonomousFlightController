/*
 * GPS.h
 *
 *  Created on: 19 Eki 2019
 *      Author: ecT
 */

#ifndef GPS_H_
#define GPS_H_

#include "common.h"

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

 #define  Dollar  0x24	// $
 #define  Comma   0x2c	// ,
 #define  Star    0x2A	// *
 #define  Dot 	  0x2E	// *
 #define  CR_ 	  0x0D	// <CR>
 #define  LF 	  0x0A	// <LF>

//** S T R U C T
 struct GNVTG_t{
	char course_gnd[8];
	char fixed_crs_gnd;
	char course_gnd_mag[8];					// NOT OUTPUT , Maybe we can delete this data
	char fixed_crs_mag;
	char speed_gnd_knt[8];
	char fixed_knt;
	char speed_gnd_kmh[8];
	char fixed_kmh;
	char mode;
	uint8_t	flag_speed;

	uint8_t cnt_array;

};

 struct GNGGA_t{
	char time_utc[10];
	char latitude[10];
	char indicator_NS;
	char longitude[11];
	char indicator_EW;
	char indicator_quality_pos;
	char sattle_num[2];
	char horz_prec[4];
	char altitude_sea[7];
	char unit_altitude;
	char geoid_seperation[10];
	char unit_geoid;
	char diff_age[5];
	char diff_id[4];
	uint8_t	flag_altitude;
	uint8_t	flag_satellite;

	uint8_t cnt_array;

};

 struct GNGLL_t{
	char latitude[10];
	char indicator_NS;
	char longitude[11];
	char indicator_EW;
	char time_utc[10];
	char cntrl_valid;
	char mode_pos;

	uint8_t cnt_array;

};

 struct GNRMC_t{
	char 	time_utc[10];					// UTC time in hhmmss.sss format (000000.000 ~ 235959.999)
	char 	status;							// Status �V� = Navigation receiver warning -  �A� = Data Valid
	char 	latitude[10];					// Latitude in dddmm.mmmm format
	char 	indicator_NS;					// �N� = North; �S� = South
	char	longitude[11];					// Longitude in dddmm.mmmm format.
	char	indicator_EW;					// E� = East; �W� = West
	char	speed_gnd[5];					// Speed over ground in knots (000.0 ~ 999.9)
	char	course_gnd[5];					// Course over ground in degrees (000.0 ~ 359.9)
	char 	date_utc[6];					// Date in day, month, year format
	char 	mag_var[5];						// ? ? ?  unnecessary information
	char 	indicator_mag;					// ? ? ?  unnecessary information
	char 	indicator_mode;					// Mode indicator �N� = Data not valid,  �A� = Autonomous mode,  �D� = Differential mode,  �E� = Estimated (dead reckoning) mode
	uint8_t	flag_latitude;
	uint8_t	flag_longitude;

	uint8_t cnt_array;

};

struct GNTXT_t {
	uint8_t flag_receive;
};


struct SafeData_t{
	double latitude;
	double longitude;
	double altitude;
	double speed;
	uint8_t satellite;

};


// F L A G
struct Flag_t{

	uint8_t		case_central;
	uint8_t		start;
	uint8_t		ack;
};




// C O U N T E R
struct Count_t{

	uint8_t		tag		;
	int16_t		bigdata	;
	uint8_t		data_sum;
	uint8_t		tag_case;
};



// D A T A
struct Data_t{

	uint8_t		big[254];		// All of receiving data
	uint8_t 	receive ;		// Received Data
	uint8_t 	tag[5]  ;		// Tag Data
	uint16_t	sum	 	;		// Sum of Tag Data
};



struct Tag_t{

	struct GNVTG_t 	GNVTG;
	struct GNGGA_t	GNGGA;
	struct GNGLL_t 	GNGLL;
	struct GNRMC_t 	GNRMC;
	struct GNTXT_t 	GNTXT;
};



typedef struct{

	struct Tag_t 		Tag;
	struct SafeData_t	Safe_Data;

	struct Flag_t		Flag;
	struct Count_t		Count;
	struct Data_t		Data;

	volatile uint8_t gps_baglanti_durumu;
}GPS_t;

// - - - - - - - -   F U N C T I O N   P R O T O T Y P E S   - - - - - - - -
void 		Initialise_Handler(void);

uint16_t 	Tag_Data_Sum(uint8_t *tag_data);

void 		Clear_Tag_Counter(void);

void 		Tag_Separation(uint16_t SumData);

void 		Separation_GNVTG(uint8_t cnt_gnvtg);

void 		Separation_GNGGA(uint8_t cnt_gngga);

void 		Separation_GNGLL(uint8_t cnt_gngll);

void 		Separation_GNRMC(uint8_t cnt_gnrmc);

double 		Latitude_Calculator(char raw_latitude[9]);

double 		Longitude_Calculator(char raw_longitude[10]);

float 		Altitude_Calculator(char raw_altitude[5]);

uint8_t 	Satellite_Calculator(char raw_satellite[2]);

float 		Speed_Calculator(char raw_satellite[7]);

void 		GPS_main(void);
// - - - - - - - -   F U N C T I O N   P R O T O T Y P E S   - - - - - - - -
GPS_t GPS;


#endif /* GPS_H_ */
