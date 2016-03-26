/*
 * mailbox_test.h
 *
 *  Created on: Mar 9, 2016
 *      Author: oronard
 */

#ifndef FW_HARDWARE_TEST_MAILBOX_TEST_H_
#define FW_HARDWARE_TEST_MAILBOX_TEST_H_

#include <stdint.h>
#include "../../Board.h"


typedef struct mailbox_test_sample_message{
	int16_t something;
	float something_else;
	char something_array[8];
}mailbox_test_sample_message;

uint8_t mailbox_test_post_message(mailbox_test_sample_message *m);
uint8_t mailbox_test_pop_message (mailbox_test_sample_message *m);


#endif /* FW_HARDWARE_TEST_MAILBOX_TEST_H_ */
