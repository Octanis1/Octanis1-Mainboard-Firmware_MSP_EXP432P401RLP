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

typedef enum comm_channels {
	CHANNEL_LORA_TTN,
	CHANNEL_LORA_SWISSCOM,
	CHANNEL_ROCKBLOCK,
	CHANNEL_GSM,
	CHANNEL_GSM_SMS,
	CHANNEL_BLE,
	CHANNEL_APP_UART
} COMM_CHANNEL;

typedef enum comm_channel_direction {
	CHANNEL_IN,
	CHANNEL_OUT
} COMM_CHANNEL_DIRECTION;

typedef struct {
   mavlink_message_t mavlink_message;
   unsigned int channel : 7;
   unsigned int direction : 1;
} COMM_FRAME;


int comm_tx_slot_open(MAV_COMPONENT component); //check if outgoing message can be sent for a given destination and component id
void comm_mavlink_post_outbox(COMM_CHANNEL channel, mavlink_message_t *message); //post to mailbox for outgoing messages
void comm_mavlink_post_inbox(COMM_CHANNEL channel, mavlink_message_t *message); //post to mailbox for incoming messages

void comm_task();


#endif
