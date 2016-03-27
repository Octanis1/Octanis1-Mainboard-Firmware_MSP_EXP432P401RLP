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
//max size of internal comm frame (hex string)
#define COMM_FRAME_SIZE 350
//max size of status string
#define COMM_STRING_SIZE 175

typedef enum comm_dest {
	DESTINATION_LORA_TTN,
	DESTINATION_LORA_SWISSCOM,
	DESTINATION_ROCKBLOCK,
	DESTINATION_GSM,
	DESTINATION_DEBUG_UART
} COMM_DESTINATION;


void comm_task();


#endif
