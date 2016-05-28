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

<<<<<<< b0b0d739b114ef802d5d4db2ca787daf655aae42
	while(1){
		//pend COMM RX TX mailbox
=======
    		// Poll for received commands
		#ifdef BLE_ENABLED
    			comm_receive_command(DESTINATION_BLE);
		#endif

    		rx_counter++;
		Task_sleep(50);

		// Status TX part:
		comm_poll_status(&my_rover_status);
		comm_execute_thresholds(&my_rover_status);

    		if(rx_counter > RX_TO_TX_RATIO)
    		{
    			rx_counter=0;

			#ifdef MAVLINK_ENABLED
    			/* MAVLINK GENERIC SETUP */
    			   mavlink_system_t mavlink_system;

			   mavlink_system.sysid = 25;                   ///< ID 25 for this rover
			   mavlink_system.compid = MAV_COMP_ID_ALL;     ///< The component sending the message is all, it could be also a Linux process

			   // Define the system type, in this case an airplane
			   uint8_t system_type = MAV_TYPE_GROUND_ROVER;
			   uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;

			   uint8_t system_mode = MAV_MODE_MANUAL_DISARMED; ///< Booting up
			   uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

			   uint64_t system_time = 1000 * 1000 * (my_rover_status.system_seconds); //TODO get time on each message packing




			   // Initialize the required buffers
			   mavlink_message_t msg;
			   uint8_t buf[MAVLINK_MAX_PACKET_LEN];
			   uint16_t mavlink_msg_len;



			  /* MAVLINK HEARTBEAT */
			   // Pack the message
			   mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, system_type, autopilot_type, system_mode, 0, system_state);

			   // Copy the message to the send buffer and send
			   mavlink_msg_len = mavlink_msg_to_send_buffer(buf, &msg);
 			   cli_send_mavlink(buf, mavlink_msg_len);


  			  /* MAVLINK AUTOPILOT_VERSION */
			   // Pack the message
			   mavlink_msg_autopilot_version_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
										   MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET | MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION | MAV_PROTOCOL_CAPABILITY_COMMAND_INT | MAV_PROTOCOL_CAPABILITY_MISSION_INT,
										   1,
										   1,
										   1,
										   1,
										   NULL,
										   NULL,
										   NULL,
										   1024,
										   1,
										   1
									   );

			   // Copy the message to the send buffer and send
			   mavlink_msg_len = mavlink_msg_to_send_buffer(buf, &msg);
			   cli_send_mavlink(buf, mavlink_msg_len);



 			  /* MAVLINK GPS_RAW_INT */
			  mavlink_msg_gps_raw_int_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
					  	  	  	  	  	  system_time,
					 					  my_rover_status.gps_fix_quality,
					  	  	  	  	  	  my_rover_status.gps_lat,
										  my_rover_status.gps_long,
										  my_rover_status.altitude,
										  UINT16_MAX,
										  UINT16_MAX,
										  my_rover_status.speed,
										  UINT16_MAX,
										  255);
			  // Copy the message to the send buffer and send
			  mavlink_msg_len = mavlink_msg_to_send_buffer(buf, &msg);
			  cli_send_mavlink(buf, mavlink_msg_len);


 			  /* MAVLINK HIGHRES_IMU */
			  mavlink_msg_highres_imu_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
					  	  	  	  	  	  system_time,
										  my_rover_status.accel_x,
										  my_rover_status.accel_y,
										  my_rover_status.accel_z,
										  0, //x gyro
										  0, //y gyro
										  0, //z gyro
										  0, //x mag
										  0, //y mag
										  0, //z mag
										  my_rover_status.int_pressure, //absolute pressure
										  0, //differential pressure
										  0, //altitude from pressure
										  my_rover_status.int_temperature,
										  0xffff //bitmask for fields that have been updated since last message (0=xacc, 12=temp)
			  	  	  	  	  	  	  	  );
			  // Copy the message to the send buffer and send
			  mavlink_msg_len = mavlink_msg_to_send_buffer(buf, &msg);
			  cli_send_mavlink(buf, mavlink_msg_len);


			  /* MAVLINK ATTITUDE  */
			  mavlink_msg_attitude_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
					  0,
					  my_rover_status.imu_roll,
					  my_rover_status.imu_pitch,
					  my_rover_status.imu_heading,
					  0, //rollspeed
					  0, //pitchspeed
					  0  //yawspeed
			  	  	  );

			  // Copy the message to the send buffer and send
			  mavlink_msg_len = mavlink_msg_to_send_buffer(buf, &msg);
			  cli_send_mavlink(buf, mavlink_msg_len);



			  /* MAVLINK GLOBAL_POSITION_INT */
			  mavlink_msg_global_position_int_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
			  					  	  	  	  	  	  0,
			  					 					  my_rover_status.gps_fix_quality,
			  					  	  	  	  	  	  my_rover_status.gps_lat,
			  										  my_rover_status.gps_long,
			  										  my_rover_status.altitude,
			  										  0,
			  										  0,
			  										  0,
			  										  my_rover_status.imu_heading);

			  // Copy the message to the send buffer and send
			  mavlink_msg_len = mavlink_msg_to_send_buffer(buf, &msg);
			  cli_send_mavlink(buf, mavlink_msg_len);


			  /*
			  											stat->v_bat,
			  											stat->v_solar,
			  											stat->i_in,
			  											stat->i_out,
			  											stat->im	u_calib_status,
			  											stat->imu_heading,

			  											stat->int_humidity,
			  											stat->ext_temperature,
			  											stat->ext_pressure,
			  											stat->ext_humidity,

			  											stat->angle_target);
			  										*/





 			  /* MAVLINK SYS_STATUS */
			  mavlink_msg_sys_status_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
					                      0,
										  0,
										  0,
										  0,
										  my_rover_status.v_bat,
										  my_rover_status.i_out,
										  -1,
										  0,
										  0,
										  0,
										  0,
										  0,
										  0
			  	  	  	  	  	  	  	  );

			  // Copy the message to the send buffer and send
			  mavlink_msg_len = mavlink_msg_to_send_buffer(buf, &msg);
			  cli_send_mavlink(buf, mavlink_msg_len);


			  /* MAVLINK SYSTEM_TIME */
			  mavlink_msg_system_time_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
					  	  	  	  	  	 system_time,
										 0
			  	  	  	  	  	  	  	 );

			  // Copy the message to the send buffer and send
			  mavlink_msg_len = mavlink_msg_to_send_buffer(buf, &msg);
			  cli_send_mavlink(buf, mavlink_msg_len);



			  /* MAVLINK STATUS_TEXT (can be used like printf, 50B max) */


			  #endif



			#ifdef GSM_ENABLED
			comm_send_status(&my_rover_status, DESTINATION_GSM);
//			if(i > 10){
//				Task_sleep(2000);
//				comm_send_status(&my_rover_status, DESTINATION_GSM_SMS);
//				i=0;
//			}
			i++;
			Task_sleep(5000);
			#endif


			#ifdef LORA_ENABLED
			lora_tx_counter++;
			if(lora_tx_counter > LORA_TX_RATIO)
			{
				comm_send_status(&my_rover_status, DESTINATION_LORA_TTN);
				lora_tx_counter = 0;
			}
			#endif


			#ifdef UARTCAM_ENABLED
			if(vc0706_begin()){
				vc0706_gprs_upload_jpeg();
			}
			vc0706_end();
			#endif


			#ifdef BLE_ENABLED
			comm_send_status(&my_rover_status, DESTINATION_BLE);
			#endif

            #ifndef MAVLINK_ENABLED
			comm_send_status(&my_rover_status, DESTINATION_APPLICATION_UART);
			#endif
    		}
    }
>>>>>>> adding other important mavlink messages

		   //if INCOMING then
		   //  comm_mavlink_handler()
		   //else
		   //  comm_send()
	}
}
