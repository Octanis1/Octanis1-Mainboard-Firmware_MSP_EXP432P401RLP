/*
 * mailbox_test.c
 *
 *  Created on: Mar 9, 2016
 *      Author: oronard
 */

#include "mailbox_test.h"

uint8_t mailbox_test_post_message(mailbox_test_sample_message *m){
    return Mailbox_post(logging_mailbox, m, BIOS_NO_WAIT);
}

uint8_t mailbox_test_pop_message (mailbox_test_sample_message *m){
	return Mailbox_pend(logging_mailbox, m, BIOS_NO_WAIT);
}

