/*
 * comm.h
 *
 *  Created on: 10.08.2015
 *      Author: Sam
 */
 
#ifndef __COMM_H
#define __COMM_H

#include <stdint.h>
//mavlink wire protocol
#include "../lib/mavlink/common/mavlink.h"
#include "../lib/mavlink/mavlink_types.h"

// Mavlink system ID's for all subsystem currently in the rover.
#define SBC_SYSTEM_ID			13 // Single board computer: Olimex
#define FBS_SYSTEM_ID			24 // Field base station
#define APM_PLANNER_SYSTEM_ID	255 // AMP planner on operator's PC

typedef enum comm_channels {
	CHANNEL_APP_UART,
	CHANNEL_LORA,
	CHANNEL_ROCKBLOCK,
	CHANNEL_GSM,

	// IMPORTANT: these channel ID's are currently used in Mavlink to access a buffer array.
	// This array size is defined MAVLINK_COMM_NUM_BUFFERS (currently = 4), so this is the
	// maximum number of channels to define!
//	CHANNEL_BLE,
	N_COMM_CHANNELS
} COMM_CHANNEL;
#define COMM_CHANNEL_NONE -1


typedef enum comm_channel_direction {
	CHANNEL_IN,
	CHANNEL_OUT
} COMM_CHANNEL_DIRECTION;

typedef struct {
   mavlink_message_t mavlink_message;
   unsigned int channel : 7;
   unsigned int direction : 1;
} COMM_FRAME;

typedef struct{
	uint8_t system;
	uint8_t component;
} COMM_MAV_MSG_TARGET;

typedef enum comm_mav_result{
	NO_ANSWER,
	REPLY_TO_SENDER,
	FORWARD_MESSAGE
} COMM_MAV_RESULT;

extern mavlink_system_t mavlink_system;
mavlink_heartbeat_t comm_get_mavlink_heartbeat();
char comm_mainboard_armed(); //read only from outside this function
void comm_arm_mainboard();
void comm_disarm_mainboard();

char comm_sbc_running();
char comm_sbc_armed();
void comm_arm_disarm_subsystems(float arm_disarm);

#define BROADCAST_SUCCESS		0x00
#define BROADCAST_MSG_UNKNOWN	0xFF
#define BROADCAST_ALL_BUSY		0xF0
#define BROADCAST_IRR_MSG_NOT_IN_SET		0x10
#define BROADCAST_ERROR			0xE0
uint8_t comm_mavlink_broadcast(COMM_FRAME* frame); //posts to mailbox for all available channel slots for a given component

void comm_mavlink_post_inbox(COMM_CHANNEL channel, mavlink_message_t *message); //post to mailbox for incoming messages

#endif
