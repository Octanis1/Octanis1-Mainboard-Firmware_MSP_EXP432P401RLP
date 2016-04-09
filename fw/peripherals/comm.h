/*
 * comm.h
 *
 *  Created on: 10.08.2015
 *      Author: Sam
 */
 
#ifndef __COMM_H
#define __COMM_H

#include <stdint.h>

//max size of mobile originated messages
#define COMM_MO_SIZE 340
//max size of mobile terminated messages
#define COMM_MT_SIZE 270
//max size of internal comm frame (hex string)
#define COMM_FRAME_SIZE 350
//max size of status string
#define COMM_STRING_SIZE 175
//how many times do we poll for received commands per sending a status message (RX: every 50ms, TX every 5s)
#define RX_TO_TX_RATIO	100


typedef enum comm_dest {
	DESTINATION_LORA_TTN,
	DESTINATION_LORA_SWISSCOM,
	DESTINATION_ROCKBLOCK,
	DESTINATION_GSM,
	DESTINATION_GSM_SMS,
	DESTINATION_BLE,
	DESTINATION_DEBUG_UART
} COMM_DESTINATION;

typedef struct comm_condition_ {
	enum variable_{
		IMUX,IMUY,IMUZ,
		IMUP,IMUR,IMUH,
		TEMP,PRES,HUMI
	} variable;
	enum op_{
		GREATER,
		SMALLER,
		EQUAL,
		NEQUAL,
		INT
	} op;
	int threshold;
} COMM_CONDITION;


typedef struct comm_led_control_ {
	COMM_CONDITION cond;
	uint16_t frequency;
} COMM_LED_CONTROL;

#define USER_MSG_SIZE	4
typedef struct comm_msg_control_ {
	COMM_CONDITION cond;
	char message[USER_MSG_SIZE];
	int msglength;
} COMM_MSG_CONTROL;



void comm_task();


#endif
