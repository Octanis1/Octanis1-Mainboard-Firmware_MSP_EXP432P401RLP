/*
 * comm.c
 *
 *  Created on: 10.08.2015
 *      Author: Sam
 */

#include "../../Board.h"
#include "comm.h"
#include "hal/rockblock.h"
#include "hal/hx1.h"


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
/*
	char frame_buffer[COMM_FRAME_SIZE];
	comm_frame_t frame;

	Mailbox_pend(comm_tx_mailbox, frame_buffer, BIOS_WAIT_FOREVER);

	memcpy(frame_buffer, &frame, sizeof(frame_buffer));

	//hardcode CLI for now
	dispatch_message(COMM_CLI, frame.message_length, frame.message_buffer);

	cli_printf("msg pended  \n",0);
	Task_sleep(1000);
	return 1;
	*/
}

int comm_post_message(comm_frame_t frame){

/*
	unsigned char frame_buffer[COMM_FRAME_SIZE];

	//copy frame struct to frame buffer
	memcpy(&frame, frame_buffer, sizeof(frame));

	//post the frame
    return Mailbox_post(comm_tx_mailbox, frame_buffer, BIOS_NO_WAIT);

	return 1;
	*/
}



void comm_task(){

	while(1){
		Task_sleep(220);

		//pend_message();




	}

}
