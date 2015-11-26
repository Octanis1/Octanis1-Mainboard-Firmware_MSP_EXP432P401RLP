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
//max size of internal comm frame
#define COMM_FRAME_SIZE 350

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
	double imu_heading;
	double imu_roll;
	double imu_pitch;
	signed int int_temperature;
	unsigned int int_pressure;
	unsigned int int_humidity;
	uint16_t ext_temperature;
	uint32_t ext_pressure;
	float ext_humidity
} rover_status_comm;


//public communications frame for encapsulation
typedef struct {
	comm_destination_t destination;
	int message_length;
	uint8_t message_buffer[COMM_MO_SIZE];
} comm_frame_t;


void comm_task();

int comm_post_message(comm_frame_t frame);

#endif
