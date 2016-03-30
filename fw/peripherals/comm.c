/*
 * comm.c
 *
 *  Created on: 10.08.2015
 *      Author: Sam
 */

#include "../../Board.h"
#include "comm.h"
#include "hal/rockblock.h"
#include "hal/rn2483.h"
#include "hal/sim800.h"
#include "hal/vc0706.h"
#include "../lib/printf.h"


/* libraries to get status info from other peripherals */
#include "gps.h"
#include "imu.h"
#include "weather.h"
#include "../core/eps.h"


/* Struct definitions */
typedef struct _rover_status_comm {
	float gps_lat;
	float gps_long;
	uint8_t gps_fix_quality;
	uint32_t system_seconds;
	uint16_t v_bat;
	uint16_t v_solar;
	uint16_t i_in;
	uint16_t i_out;
	uint8_t imu_calib_status;
	int16_t imu_heading; //converted from double
	int16_t imu_roll; //converted from double
	int16_t imu_pitch; //converted from double
	int int_temperature;
	unsigned int int_pressure;
	unsigned int int_humidity;
	int ext_temperature;
	unsigned int ext_pressure;
	unsigned int ext_humidity; //converted from float
} rover_status_comm;


void comm_init(rover_status_comm* stat)
{
	Task_sleep(5000);
	cli_printf("reset occured.\n", 0);

	if(rn2483_begin()){
	#if VERBOSE==1
		cli_printf("rn2483 begin OK.\n", 0);
	#endif
	}else{
		cli_printf("rn2483 begin NOK\n", 0);
		rn2483_end();
	}


	if(sim800_begin()){
	#if VERBOSE==1
		cli_printf("sim800 begin OK.\n", 0);
	#endif
	}else{
		cli_printf("sim800 begin NOK\n", 0);
		sim800_end();
	}


	stat->gps_lat = -1.0;
	stat->gps_long = -1.0;
	stat->gps_fix_quality = -1;
	stat->system_seconds = -1;
	stat->v_bat = 0;
	stat->v_solar = 0;
	stat->i_in = 0;
	stat->i_out = 0;
	stat->imu_calib_status = 0;
	stat->imu_heading = -1;
	stat->imu_roll = -1;
	stat->imu_pitch = -1;
	stat->int_temperature = -274;
	stat->int_pressure = -1;
	stat->int_humidity = -1;
	stat->ext_temperature = -274;
	stat->ext_pressure = -1;
	stat->ext_humidity = -1;
}


void comm_poll_status(rover_status_comm* stat)
{
	/*Fill in struct with status information */
	stat->gps_lat = gps_get_lat();
	stat->gps_long = gps_get_lon();
	stat->gps_fix_quality = gps_get_fix_quality();
	stat->system_seconds = Seconds_get();
	stat->v_bat = eps_get_vbat();
	stat->v_solar = eps_get_vsolar();
	stat->i_in = eps_get_iin();
	stat->i_out = eps_get_iout();
	stat->imu_calib_status = imu_get_calib_status();
	stat->imu_heading = imu_get_heading();
	stat->imu_roll = imu_get_roll();
	stat->imu_pitch = imu_get_pitch();
	stat->int_temperature = weather_get_int_temp();
	stat->int_pressure = weather_get_int_press();
	stat->int_humidity = weather_get_int_humid();
	stat->ext_temperature = weather_get_ext_temp();
	stat->ext_pressure = weather_get_ext_press();
	stat->ext_humidity = weather_get_ext_humid();
}


void comm_send_status(rover_status_comm* stat, COMM_DESTINATION destination)
{
	/* create Hexstring buffer from struct */
	int stringlength=0;
	char txdata[COMM_STRING_SIZE] = "";

	stringlength += ftoa(stat->gps_lat, &txdata[stringlength], 7); //convert gps latitude to string with sign and 7 afterpoint
	txdata[stringlength++] = ','; 					//plus a comma

	stringlength += ftoa(stat->gps_long, &txdata[stringlength], 7); //convert gps long to string with sign and 7 afterpoint
	txdata[stringlength++] = ','; 					//plus a comma

	stringlength += tfp_sprintf(&(txdata[stringlength]), "%d,%u,%u,%u,%u,%u,%u,%d,%d,%d,%d,%u,%u,%d,%d",
											stat->gps_fix_quality,
											stat->system_seconds,
											stat->v_bat,
											stat->v_solar,
											stat->i_in,
											stat->i_out,
											stat->imu_calib_status,
											stat->imu_heading,
											stat->imu_roll,
											stat->imu_pitch,
											stat->int_temperature,
											stat->int_pressure,
											stat->int_humidity,
											stat->ext_temperature,
											stat->ext_pressure,
											stat->ext_humidity);

	if(stringlength > COMM_FRAME_SIZE) //should never happen! corrupt memory will be the result!
	{
		cli_printf("status string overflow! %u",stringlength);
		stringlength = COMM_FRAME_SIZE;
	}

	char hex_string_byte[2];
	char hex_string[COMM_FRAME_SIZE]; //TODO: ATTENTION: this is too small! need to change this
	memset(&hex_string, 0, sizeof(hex_string));

	int i;
	for(i=0; i<stringlength; i++){
		memset(&hex_string_byte, 0, sizeof(hex_string_byte));
		tfp_sprintf(hex_string_byte, "%02x", txdata[i]);
		strcat(hex_string, hex_string_byte);
	}



	switch(destination) {
	   case DESTINATION_LORA_TTN:
	      //lora test
		  rn2483_send_receive(hex_string, 2*stringlength);
		  rn2483_end();
		  //lora test end
	      break;

	   case DESTINATION_GSM:
		  sim800_send_http(hex_string, strlen(hex_string), MIME_TEXT_PLAIN);
	      break;

	}

}



void comm_task(){

	#ifdef CONFIG_MODE
		int comm_result=rn2483_config();
		if(comm_result)
			cli_printf("LoRa config failed: %d", comm_result);
		else
			cli_printf("LoRa config success: %d", comm_result);
	#endif


	rover_status_comm my_rover_status;
    comm_init(&my_rover_status);

    while(1){

		comm_poll_status(&my_rover_status);
		comm_send_status(&my_rover_status, DESTINATION_GSM);
#ifndef CAMERA_BOARD
		comm_send_status(&my_rover_status, DESTINATION_LORA_TTN);
#endif

#ifdef CAMERA_BOARD
		//camera instead of lora module
		if(vc0706_begin()){
			vc0706_gprs_upload_jpeg();
		}
		vc0706_end();
#endif


		Task_sleep(5000);

	}

}
