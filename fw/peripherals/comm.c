/*
 * comm.c
 *
 *  Created on: 10.08.2015
 *      Author: Sam
 */

#include "../../Board.h"
#include "comm.h"
#include <string.h>

#ifdef MAVLINK_ON_UART0_ENABLED
	static UART_Handle application_uart = NULL;
#endif

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
			  UART_write(application_uart,msg,msg->len);
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

#ifdef MAVLINK_ON_UART0_ENABLED
	static UART_Params uartParams;

	/* Create a UART with data processing off. */
	UART_Params_init(&uartParams);
	uartParams.writeDataMode = UART_DATA_BINARY;
	uartParams.readDataMode = UART_DATA_BINARY;
	uartParams.readReturnMode = UART_RETURN_FULL;
	uartParams.writeMode = UART_MODE_BLOCKING;
	uartParams.readEcho = UART_ECHO_OFF;
	uartParams.baudRate = 115200;
	application_uart = UART_open(Board_UART0_DEBUG, &uartParams);

	if (application_uart == NULL) {
		System_abort("Error opening the UART");
	}

#endif

}



void comm_task(){

	comm_init();

	while(1){
		//pend COMM RX TX mailbox

		   //if INCOMING then
		   //  comm_mavlink_handler()
		   //else
		   //  comm_send()
	}
}
