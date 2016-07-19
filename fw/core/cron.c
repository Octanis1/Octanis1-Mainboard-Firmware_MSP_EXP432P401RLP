/*
 * cron.c
 *
 *  Created on: 13 Aug 2015
 *      Author: Sam
 */

#include "../../Board.h"
#include "cron.h"
//
//mavlink test
#include "../peripherals/comm.h"
#include "../lib/mavlink/common/mavlink.h"

//called periodically
Void cron_quick_clock(UArg arg){
	// flash led
	GPIO_toggle(Board_LED_GREEN); // use red led for user inputs


	/* MAVLINK HEARTBEAT */
	// Initialize the message buffer
	static COMM_FRAME frame;

	mavlink_heartbeat_t hb=comm_get_mavlink_heartbeat();
	// Pack the message
	mavlink_msg_heartbeat_encode(mavlink_system.sysid, mavlink_system.compid, &(frame.mavlink_message), &hb);

#ifdef MAVLINK_ON_UART0_ENABLED
	comm_set_tx_flag(CHANNEL_APP_UART, mavlink_system.compid);
#endif

#ifdef LORA_ENABLED
	comm_set_tx_flag(CHANNEL_LORA, mavlink_system.compid);
#endif
	comm_mavlink_broadcast(&frame); //send heartbeat for all available channel slots

}


Void cron_hourly_clock(UArg arg){

}
