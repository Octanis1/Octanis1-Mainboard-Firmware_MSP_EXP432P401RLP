/*
 * cron.c
 *
 *  Created on: 13 Aug 2015
 *      Author: Sam
 */

#include "../../Board.h"
#include "cron.h"
#include "system.h"


//called periodically
Void cron_quick_clock(UArg arg){
  //flash led
  GPIO_toggle(Board_LED_RED);

  //print load periodically
  int cpuLoad = Load_getCPULoad();
  cli_printf("CPU load: %d \n", cpuLoad);
  //cli_printf("Epoch: %d \n", Seconds_get());




	//sends out system status message over the air
	system_communicate_rover_status();

}


Void cron_hourly_clock(UArg arg){

}
