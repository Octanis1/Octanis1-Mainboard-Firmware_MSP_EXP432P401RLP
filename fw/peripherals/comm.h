/*
 * comm.h
 *
 *  Created on: 10.08.2015
 *      Author: Sam
 */
 
#ifndef __COMM_H
#define __COMM_H


//max size of mobile originated messages
#define COMM_MO_SIZE 340
//max size of mobile terminated messages
#define COMM_MT_SIZE 270
//max size of internal comm frame (hex string for LoRa)
#define COMM_FRAME_SIZE 350
//max size of status string
#define COMM_STRING_SIZE 175


//available destinations
typedef enum {COMM_CLI, COMM_IRIDIUM, COMM_GSM, COMM_VHF} comm_destination_t;

/* Enum definitions */
/* Struct definitions */
typedef struct _rover_status_comm {
	float gps_lat;
	float gps_long;
	int gps_fix_quality;
	uint32_t system_seconds;
	unsigned char imu_calib_status;
	int imu_heading; //converted from double
	int imu_roll; //converted from double
	int imu_pitch; //converted from double
	int int_temperature;
	unsigned int int_pressure;
	unsigned int int_humidity;
	int ext_temperature;
	unsigned int ext_pressure;
	unsigned int ext_humidity; //converted from float
} rover_status_comm;


//public communications frame for encapsulation
typedef struct {
	comm_destination_t destination;
	int message_length;
	uint8_t message_buffer[COMM_MO_SIZE];
} comm_frame_t;


void comm_init();
void comm_task();

int comm_post_message(comm_frame_t frame);

#endif
