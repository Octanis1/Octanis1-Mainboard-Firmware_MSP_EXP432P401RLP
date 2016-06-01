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

  //TODO remove:

  		  // Mavlink heartbeat
  		  // Define the system type, in this case an airplane
  		  	uint8_t system_type = MAV_TYPE_GROUND_ROVER;
  		  	uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;

  		  	uint8_t system_mode = MAV_MODE_MANUAL_DISARMED; ///< Booting up
  		  	uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

  		  	/* MAVLINK HEARTBEAT */

  			// Initialize the required buffers
  			mavlink_message_t msg;

  			COMM_FRAME frame;
  			frame.channel = CHANNEL_APP_UART;
  			frame.direction = 1;


  			// Pack the message
  			mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, system_type, autopilot_type, system_mode, 0, system_state);

  			frame.mavlink_message = msg;

  			Mailbox_post(comm_mailbox, &frame, BIOS_NO_WAIT);
}


Void cron_hourly_clock(UArg arg){

}
