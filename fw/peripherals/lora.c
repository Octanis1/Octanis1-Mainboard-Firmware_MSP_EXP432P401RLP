/*
 * lora.c
 *
 */

#include "../../Board.h"
#include "comm.h"
#include "lora.h"
#include <string.h>
#include "hal/rn2483.h"
#include "../lib/printf.h"

#define LORA_RETRY_INIT_AFTER_N_CYCLES	20

void lora_send_mavlink(uint8_t* txdata, uint16_t stringlength)
{
	/** Prepare hex string for LoRa **/
	char hex_string_byte[2];
	char hex_string[LORA_FRAME_SIZE];
	memset(&hex_string, 0, sizeof(hex_string));

	int i;
	for(i=0; i<stringlength; i++){
		memset(&hex_string_byte, 0, sizeof(hex_string_byte));
		tfp_sprintf(hex_string_byte, "%02x", txdata[i]);
		strcat(hex_string, hex_string_byte);
	}
	/** end hex string **/

	rn2483_send_receive(hex_string, 2*stringlength);
}



int lora_init(){
	cli_init();

#ifdef LORA_ENABLED
	#ifdef CONFIG_MODE
		int comm_result=rn2483_config();
		if(comm_result)
			serial_printf(cli_stdout,"LoRa config failed: %d", comm_result);
		else
			serial_printf(cli_stdout,"LoRa config success: %d", comm_result);
		return 0;
	#else
		return rn2483_begin();
	#endif
#endif

}

void lora_task(){
	int is_initialized = lora_init();
	int init_counter = 0;

	static uint8_t buf[MAVLINK_MAX_PACKET_LEN + 5]; //todo: +5 is to test if we need some more space. remove if not helpful.
	static uint16_t mavlink_msg_len;
	static COMM_FRAME mail;


	while(1){
		if(Mailbox_pend(lora_mailbox, &mail, BIOS_WAIT_FOREVER)){

			if(is_initialized)
			{
				if((mail.direction) == CHANNEL_OUT)
				{
					if(mail.channel == CHANNEL_LORA)
					{
						mavlink_msg_len = mavlink_msg_to_send_buffer(buf, &mail.mavlink_message);
						lora_send_mavlink(buf, mavlink_msg_len);
					}
					else
					{
						serial_printf(cli_stdout,"ERROR: outgoing msg for channel %d in lora_mailbox. id=%d \n", mail.channel, mail.mavlink_message.msgid);
					}
				}
				else
				{
					serial_printf(cli_stdout,"ERROR: incoming msg in lora_mailbox. id=%d \n", mail.mavlink_message.msgid);
					// should never happen, but just in case: redirect message!
					Mailbox_post(comm_mailbox, &mail, BIOS_NO_WAIT);
				}
			}
			else
			{
				init_counter++;
				if(init_counter > LORA_RETRY_INIT_AFTER_N_CYCLES)
				{
					init_counter = 0;
					is_initialized = rn2483_begin();
				}
			}



		}
 	}

}
