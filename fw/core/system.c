/*
 * system.c
 *
 *  Created on: 11.08.2015
 *      Author: Sam
 */

#include "../../Board.h"
#include "../peripherals/comm.h"
#include "../peripherals/hal/rockblock.h"

//protobuf

#include "../lib/nanopb/pb_encode.h"
#include "../lib/nanopb/pb_decode.h"
#include "../protobuf/rover_status.pb.h"


//holds system status struct

void system_dumpTask(Task_Handle task)
{
    Task_Stat stat;

    Task_stat(task, &stat);

#if VERBOSE==1
    cli_printf(Task_Handle_name(task), 0);
    cli_printf(" = %d \n\n", stat.mode);
#endif

}


void system_listTasks()
{
    Task_Object * task;
    Int i;

    for (i = 0; i < Task_Object_count(); i++) {
        task = Task_Object_get(NULL, i);
        system_dumpTask(task);
    }

    task = Task_Object_first();
    while (task) {
    	system_dumpTask(task);
        task = Task_Object_next(task);
    }

}


int system_chartoint(char c){
	return c - '0';
}


void system_update_rover_status(){
	//get status from various places


}


int system_communicate_rover_status(){


	comm_frame_t frame;


	  /* This is the buffer where we will store our message. */
	    uint8_t buffer[128];
	    int message_length;
	    bool status;

	    /* Encode our message */
	    {
	        /* Allocate space on the stack to store the message data.
	         *
	         * Nanopb generates simple struct definitions for all the messages.
	         * - check out the contents of simple.pb.h! */
	        rover_status message;

	        /* Create a stream that will write to our buffer. */
	        pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

	        /* Fill in the lucky number */
	        message.bv = 698;
	        message.gps_health = 5;
	        message.rockblock_health = 5;

	        /* Now we are ready to encode the message! */
	        status = pb_encode(&stream, rover_status_fields, &message);
	        message_length = stream.bytes_written;

	        /* Then just check for any errors.. */
	        if (!status)
	        {
	            cli_printf("fail\n", 0);
	            return 1;
	        }
	    }

	    /* Now we could transmit the message over network, store it in a file or
	     * wrap it to a pigeon's leg.
	     */
        frame.message_length = message_length;
		memcpy(&frame.message_buffer, buffer, sizeof(buffer));



    //post the buffer to the communications queue

	    if(comm_post_message(frame)){
	#if VERBOSE==1
    	cli_printf("posted comm to sbd\n",0);
    #endif
    }



    return 1;
}
