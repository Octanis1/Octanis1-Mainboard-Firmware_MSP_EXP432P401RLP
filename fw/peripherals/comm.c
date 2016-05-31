/*
 * comm.c
 *
 *  Created on: 10.08.2015
 *      Author: Sam
 */

#include "../../Board.h"
#include "comm.h"
#include <string.h>

/* PUBLIC */
int comm_tx_slot_open(MAV_COMPONENT component); //check if outgoing message can be sent for a given destination and component id
void comm_mavlink_post_outbox(COMM_CHANNEL channel, mavlink_message_t *message); //post to mailbox for outgoing messages
void comm_mavlink_post_inbox(COMM_CHANNEL channel, mavlink_message_t *message); //post to mailbox for incoming messages


/* PRIVATE */
void comm_send(COMM_CHANNEL channel, mavlink_message_t *msg){

	switch(channel) {
#ifdef MAVLINK_ON_UART0_ENABLED
	   case CHANNEL_APP_UART:
		  {

		  }
 	      break;
#endif

	   case CHANNEL_GSM:
		   //sim800_send_http(msg)
 	      break;

	   default: break;

 	}

}



void comm_mavlink_handler(mavlink_message_t *msg){

	switch(msg->msgid){
	  case MAVLINK_MSG_ID_HEARTBEAT:
	        {
		    // E.g. read GCS heartbeat and go into
            // comm lost mode if timer times out
	        }
	        break;
	case MAVLINK_MSG_ID_COMMAND_LONG:
		// EXECUTE ACTION
		break;
	default:
		//Do nothing
		break;
	}

}


void comm_init(){
	cli_init();
}



void comm_task(){

	COMM_FRAME mail;

	comm_init();

	while(1){

	/* MAVLINK GENERIC SETUP */
	mavlink_system_t mavlink_system;

	mavlink_system.sysid = 25;                   ///< ID 25 for this rover
	mavlink_system.compid = MAV_COMP_ID_ALL;     ///< The component sending the message is all, it could be also a Linux process

	// Define the system type, in this case an airplane
	uint8_t system_type = MAV_TYPE_GROUND_ROVER;
	uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;

	uint8_t system_mode = MAV_MODE_MANUAL_DISARMED; ///< Booting up
	uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

	uint64_t system_time = 1000 * 1000 * (1234); //TODO get time on each message packing

	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t mavlink_msg_len;

	/* MAVLINK HEARTBEAT */
	// Pack the message
	mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, system_type, autopilot_type, system_mode, 0, system_state);

	// Copy the message to the send buffer and send
	mavlink_msg_len = mavlink_msg_to_send_buffer(buf, &msg);

	serial_write(stdout, buf, mavlink_msg_len);



//		if(Mailbox_pend(comm_mailbox, &mail, BIOS_WAIT_FOREVER)){
			  //if INCOMING then
					   //  comm_mavlink_handler()
					   //else
					   //  comm_send()

//		}

	Task_sleep(500);
	}

}
