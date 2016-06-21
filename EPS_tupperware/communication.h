/*
 * communication.h
 *
 *  Created on: 09.05.2016
 *      Author: beat
 */

#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

void i2c_callback(char *buffer);
void i2c_check_command(); // sets the response.
int mainboard_poke_iterate(int *mainboard_poke_counter);

//offer callback function
//implement all the highlevel communication without care of low level protocol.

#endif /* COMMUNICATION_H_ */
