/*
 * comm.c
 *
 *  Created on: 10.08.2015
 *      Author: Sam
 */
#include "../../Board.h"
#include "hal/rockblock.h"

void comm_task(){

	while(1){
		Task_sleep(10440);

		cli_printf("RB open",0);

		if(rockblock_open()){

			rockblock_begin();

			cli_printf("RB begin done",0);


			rockblock_close();
		}

	}

}
