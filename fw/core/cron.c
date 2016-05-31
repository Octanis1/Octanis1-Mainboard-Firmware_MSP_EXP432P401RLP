/*
 * cron.c
 *
 *  Created on: 13 Aug 2015
 *      Author: Sam
 */

#include "../../Board.h"
#include "cron.h"


//called periodically
Void cron_quick_clock(UArg arg){
  //flash led
  GPIO_toggle(Board_LED_GREEN); // use red led for user inputs

}


Void cron_hourly_clock(UArg arg){

}
