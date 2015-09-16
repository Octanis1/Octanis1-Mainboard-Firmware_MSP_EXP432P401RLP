/*
 * comm.c
 *
 *  Created on: 10.08.2015
 *      Author: Sam
 */

#include "../../Board.h"
#include "comm.h"
#include "hal/rockblock.h"
#include "hal/rn2483.h"
#include "../lib/printf.h"

//protobuf DEBUG TO REMOVE

#include "../lib/nanopb/pb_encode.h"
#include "../lib/nanopb/pb_decode.h"
#include "../protobuf/rover_status.pb.h"


//dispatches the message to its destination where it will be sent
int dispatch_message(comm_destination_t dest, size_t message_length,
													uint8_t * message_buffer){

	switch(dest){
		case COMM_CLI:
			cli_printf("COMM msg received: %d \n", message_length);
		break;


		case COMM_IRIDIUM:
			cli_printf("RB open \n",0);

					if(rockblock_open()){

						if(rockblock_begin()){ //can timeout

							cli_printf("RB begin done \n",0);

							int csq = rockblock_get_signal_quality();

							cli_printf("RB sig: %d \n", csq);

							if(csq > 1){
							    GPIO_write(Board_LED1, Board_LED_ON);
							}

							//send msg here



							//check for RX messages
						}else{
							cli_printf("RB begin problem \n",0);

						}

						rockblock_close();
					}

		break;


		case COMM_GSM:
			//RESERVED FOR FUTURE
		break;


		case COMM_VHF:
			//TODO, LOW PRIO
		break;
	}

	return 1;
}


//waits for message to arrive on queue, unpacks frame, dispatches message.
// BLOCKS UNTIL MESSAGE ARRIVES IN MAILBOX AND IS SUBSEQUENTLY SENT
int pend_message(){

	char frame_buffer[COMM_FRAME_SIZE];
	comm_frame_t frame;

	Mailbox_pend(comm_tx_mailbox, frame_buffer, BIOS_WAIT_FOREVER);

	//is this memcpy dodgy?
	memcpy(&frame, frame_buffer, sizeof(frame));

	cli_printf("rxmsgl %d \n", frame.message_length);


	//try to print the buffer in hex

	//test buffer to get byte order right
	uint8_t deadbeef[4] = {222,173,190,239};

	char hex_string_byte[2];
	char hex_string[COMM_FRAME_SIZE]; //TODO: ATTENTION: this is too small! need to change this
	memset(&hex_string, 0, sizeof(hex_string));

	int i;
	for(i=0; i<4; i++){
		memset(&hex_string_byte, 0, sizeof(hex_string_byte));
	    tfp_sprintf(hex_string_byte, "%02x", deadbeef[i]);
	    strcat(hex_string, hex_string_byte);
	}


	//lora test

	 	if(rn2483_begin()){
			cli_printf("rn2483.\n", 0);
		}else{
			cli_printf("adiedrn2483.\n", 0);
		}


	 	rn2483_send_receive(hex_string, strlen(hex_string));


		rn2483_end();

	//lora test end


	/* But because we are lazy, we will just decode it immediately. */
	{
		/* Allocate space for the decoded message. */
		rover_status message;

		/* Create a stream that reads from the buffer. */
		pb_istream_t stream = pb_istream_from_buffer(frame.message_buffer, frame.message_length);

		/* Now we are ready to decode the message. */
		int status = pb_decode(&stream, rover_status_fields, &message);

		/* Check for errors... */
		if (!status)
		{
			cli_printf("fail\n", 0);
			return 1;
		}

		/* Print the data contained in the message. */
		cli_printf("bv %d!\n", message.bv);
		cli_printf("rb %d!\n", message.rockblock_health);
		cli_printf("gps %d!\n", message.gps_health);
	}





	/*
	//hardcode CLI for now
	dispatch_message(COMM_CLI, frame.message_length, frame.message_buffer);

	cli_printf("msg pended  \n",0);
	*/

	//DISPATCH TEST
	//rockblock_send_receive_SBD(NULL,NULL,NULL,NULL);

	return 1;
}

//this command is posted by any task that wishes to send a message
int comm_post_message(comm_frame_t frame){


	unsigned char frame_buffer[COMM_FRAME_SIZE];
	int frame_size = sizeof(frame);

	//copy frame struct to frame buffer
	memcpy(frame_buffer, &frame, frame_size);

	cli_printf("frame size %d \n", frame_size);

	//post the frame
    return Mailbox_post(comm_tx_mailbox, frame_buffer, BIOS_NO_WAIT);

	return 1;

}



void comm_task(){

	while(1){
 	    GPIO_write(MSP_EXP432P401RLP_LED_RED, Board_LED_ON);

		pend_message();


		GPIO_write(MSP_EXP432P401RLP_LED_RED, Board_LED_OFF);

		Task_sleep(10000);



	}

}
