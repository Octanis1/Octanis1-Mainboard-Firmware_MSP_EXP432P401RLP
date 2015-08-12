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

		cli_printf("RB open \n",0);

		if(rockblock_open()){

			if(rockblock_begin()){ //can timeout

				cli_printf("RB begin done \n",0);

				int csq = rockblock_get_signal_quality();

				cli_printf("RB sig: %d \n", csq);

			}else{
				cli_printf("RB begin problem \n",0);

			}

			rockblock_close();
		}

	}

}
