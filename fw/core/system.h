/*
 * system.h
 *
 *  Created on: 11.08.2015
 *      Author: Sam
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_

void system_dumpTask(Task_Handle task);
void system_listTasks();

int system_chartoint(char c);

int system_communicate_rover_status();

#endif /* SYSTEM_H_ */
