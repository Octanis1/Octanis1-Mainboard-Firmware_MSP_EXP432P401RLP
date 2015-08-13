/*
 * system.c
 *
 *  Created on: 11.08.2015
 *      Author: Sam
 */

#include "../../Board.h"
#include "../peripherals/comm.h"

//protobuf
#include "../lib/nanopb/pb_encode.h"
#include "../lib/nanopb/pb_decode.h"
#include "../protobuf/rover_status.pb.h"


//holds system status struct
static rover_status status;


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

int system_send_status(){

	//get status from various places

	//send to comms queue

}