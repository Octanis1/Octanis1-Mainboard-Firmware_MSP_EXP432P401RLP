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
static rover_status system_status;

void system_dumpTask(Task_Handle task)
{
    Task_Stat stat;

    Task_stat(task, &stat);


    cli_printf(Task_Handle_name(task), 0);
    cli_printf(" = %d \n\n", stat.mode);

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
	system_status.rockblock_health = rockblock_get_health();
}


int system_communicate_rover_status(){

	comm_frame_t status_frame;

	uint8_t message_buffer[COMM_MO_SIZE];
	size_t message_length;

	//update status message
	system_update_rover_status();

    //create a stream that will write to our buffer.
    pb_ostream_t stream = pb_ostream_from_buffer(message_buffer, sizeof(message_buffer));

    //now we are ready to encode the message!
    pb_encode(&stream, rover_status_fields, &system_status);
    message_length = stream.bytes_written;

    //put message in frame
    status_frame.destination = COMM_CLI;
    status_frame.message_length = message_length;
    memcpy(&status_frame.message_buffer, message_buffer, COMM_MO_SIZE);

    //post the buffer to the communications queue
    if(comm_post_message(status_frame)){
    	cli_printf("posted comm to sbd\n",0);
    }


    return 1;
}
