/*
 *  File: rockblock.c
 *  Description: Model for Iridium Satellite Modem "Rockblock"
 *  Author: Sam
 *
 *  Wouldn't have been possible without the great help from:
 *  {
	 *  IridiumSBD - An Arduino library for Iridium SBD ("Short Burst Data") Communications
	 *	Suggested and generously supported by Rock Seven Location Technology
	 *	(http://rock7mobile.com), makers of the brilliant RockBLOCK satellite modem.
	 *	Copyright (C) 2013 Mikal Hart
	 *	All rights reserved.
	 *
	 *	The latest version of this library is available at http://arduiniana.org.
 *  }
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "../../Board.h"
#include "../lib/mavlink/common/mavlink.h"
#include "comm.h"

#include "rockblock.h"


#define ROCKBLOCK_WAKE 1
#define ROCKBLOCK_WAKE_WAIT 600
#define ROCKBLOCK_SLEEP 0
#define ROCKBLOCK_RXBUFFER_SIZE 20
#define ROCKBLOCK_READ_TIMEOUT 10000
#define ROCKBLOCK_BAUD_RATE 9600

#define ROCKBLOCK_CSQ_MINIMUM        2
//following time units in seconds
#define ROCKBLOCK_CSQ_INTERVAL       20
#define ROCKBLOCK_SBDIX_INTERVAL     300
#define ROCKBLOCK_SENDRECEIVE_TIME   300
#define ROCKBLOCK_STARTUP_MAX_TIME   240


static UART_Handle uart;
static UART_Params uartParams;

/* Iridium 9602 Modem AT commands */
//responses from the modem are "encapsulated" in "\r\n"
static const char rockblock_at[] = "AT\r";
static const char rockblock_at_echo_off[] = "ATE0\r";
static const char rockblock_at_csq[] = "AT+CSQ\r";
static const char rockblock_at_msstm[] = "AT-MSSTM\r";
static const char rockblock_at_sbdix[] = "AT+SBDIX\r";
static const char rockblock_at_sbdrb[] = "AT+SBDRB\r";

static int last_powerup_time = 0;
static int rockblock_health = 0;
static int rockblock_remaining_msg = 0;

static uint8_t rockblock_rx_buffer[ROCKBLOCK_MESSAGE_SIZE];
int rockblock_rx_buffer_offset = 0;
static uint8_t rockblock_tx_buffer[ROCKBLOCK_MESSAGE_SIZE];
int rockblock_tx_buffer_offset = 0;


int rockblock_internal_decode_SBD_binary();
int rockblock_internal_MSSTM_workaround();
int rockblock_do_SBDIX(uint16_t *moCode, uint16_t *moMSN, uint16_t *mtCode, uint16_t *mtMSN, uint16_t *mtLen, uint16_t *mtRemaining);

int rockblock_open(){

	//wake rockblock from sleep
    GPIO_write(Board_ROCKBLOCK_SLEEP, ROCKBLOCK_WAKE);

    //it takes time for the rockblock to wake up from its beauty sleep
    Task_sleep(ROCKBLOCK_WAKE_WAIT);


	//these uart configs are good for writing to and reading from the RB module
	//the RB must first be configured to work at 9600baud and local echo off
	UART_Params_init(&uartParams);
	uartParams.writeDataMode = UART_DATA_BINARY;
	uartParams.readDataMode = UART_DATA_BINARY;
	uartParams.readReturnMode = UART_RETURN_FULL;
	uartParams.readTimeout = ROCKBLOCK_READ_TIMEOUT;
	uartParams.readEcho = UART_ECHO_OFF;
	uartParams.baudRate = ROCKBLOCK_BAUD_RATE;

	uart = UART_open(Board_UART2_COMM, &uartParams);

	if (uart == NULL) {
		return 0;
	}else{
		//store power up time
		last_powerup_time = Seconds_get();
		return 1;
	}
}


//must be called from within a task - this function will block!
int rockblock_begin(){

	if(!rockblock_open()){
//		serial_printf(cli_stdout, "RB open error \n",0);
		return 0;
	}


	char rxBuffer[ROCKBLOCK_RXBUFFER_SIZE];

	//set local echo off first
	UART_write(uart, rockblock_at_echo_off, sizeof(rockblock_at_echo_off));
	//give the RB some time to respond
	UART_read(uart, rxBuffer, sizeof(rxBuffer));
	//clear the buffer
	memset(rxBuffer, 0, sizeof(rxBuffer));

	//send an AT for confirmation that local echo has been turned off
	UART_write(uart, rockblock_at, sizeof(rockblock_at));
	UART_read(uart, rxBuffer, sizeof(rxBuffer));


	if(!strcmp("\r\nOK\r\n", rxBuffer)){
		return 1; //modem can now communicate with us
	}else{
		return 0;
	}

}


void rockblock_close(){
	int elapsed_time = Seconds_get() - last_powerup_time;

	// best practices guide suggests waiting at least 2 seconds
	// before powering off again
	if(elapsed_time < 2){
		Task_sleep(2 - elapsed_time);
	}

	UART_close(uart);
	uart = NULL;
    GPIO_write(Board_ROCKBLOCK_SLEEP, ROCKBLOCK_SLEEP);
}


//returns 0 (no signal) to 5 (best). iridium recommends at least a 2 before transmit
// 1 works in some conditions
int rockblock_get_signal_quality(){

	char rxBuffer[ROCKBLOCK_RXBUFFER_SIZE];
	static int csq_val = 0;

	//get new signal quality only if UART is open. otherwise display stored value.
	if(uart == NULL){
		return -1;
	}

	static int times_called = 1; //stores the amount of times this function was called
	static int rockblock_health_sum = 0; //contains the sum of all values this function ever saw

	//clear the buffer
	memset(rxBuffer, 0, sizeof(rxBuffer));

	UART_write(uart, rockblock_at_csq, sizeof(rockblock_at_csq));
	UART_read(uart, rxBuffer, sizeof(rxBuffer));


	//signal quality value is at index 7
	csq_val = (char)rxBuffer[7] - '0';
	/*
	rockblock_health_sum += rockblock_get_signal_quality();
	rockblock_health = (int)(rockblock_health_sum/times_called + 0.5);
	times_called++;
	*/

	return csq_val;
}

int rockblock_internal_MSSTM_workaround(){
	/*Before attempting any of the following commands: +SBDDET, +SBDREG, +SBDI, +SBDIX, +SBDIXA the field application
   should issue the AT command MSSTM to the transceiver and evaluate the response to determine if it is valid or not:

   Valid Response: "---MSSTM: XXXXXXXX" where XXXXXXXX is an eight---digit hexadecimal number.

   Invalid Response: "---MSSTM: no network service"

   If the response is invalid, the field application should wait and recheck system time until a valid response is
   obtained before proceeding. */
	char rxBuffer[ROCKBLOCK_RXBUFFER_SIZE];

	//get new signal quality only if UART is open. otherwise display stored value.
	if(uart == NULL){
		return -1;
	}

	//clear the buffer
	memset(rxBuffer, 0, sizeof(rxBuffer));

	//Issue command and read response
	UART_write(uart, rockblock_at_msstm, sizeof(rockblock_at_msstm));
	UART_read(uart, rxBuffer, sizeof(rxBuffer));

	//compare response to known invalid message, return 0 if invalid message detected
	if(strstr(rxBuffer, "-MSSTM: no network") != NULL){
		return 0;
	}
	return 1;
}


////sends an SBD, then checks the inbox (checking costs 1 credit!)
//int rockblock_send_receive_SBD(const uint8_t *tx_buffer, size_t tx_buffersize,
//								uint8_t *rx_buffer, size_t *rx_buffersizePtr){
////	serial_printf(cli_stdout, "RBsr\n",0);
//
//	if(!rockblock_begin()){
////		serial_printf(cli_stdout, "RB begin error \n",0);
//		return 0;
//	}
//
//	//write our data into the buffer of the rockblock (max 340B)
//
//	 //"AT+SBDWB=120\r" 120Bytes to be sent
//	 //wait for "READY\r\n"
//	/* write data to rockblock and compute checksum
//	      uint16_t checksum = 0;
//	      for (int i=0; i<txDataSize; ++i)
//	      {
//	         stream.write(txData[i]);
//	         checksum += (uint16_t)txData[i];
//	      }
//	*/
//	//write computed checksum to RB
//	/*
//	      stream.write(checksum >> 8);
//	      stream.write(checksum & 0xFF);
//	*/
//	//WAIT FOR "0\r\n\r\nOK\r\n"
//
//
//	//now start long SBD session to transmit message
//	for(int starttime = Seconds_get(); (Seconds_get() - starttime) < ROCKBLOCK_SENDRECEIVE_TIME; ){
//
//		int csq = rockblock_get_signal_quality();
//		if(csq > ROCKBLOCK_CSQ_MINIMUM){
//			//perform MSST workaround
//
//					   	   //AT+SBDIX\r
//					   	   //wait for +SBDIX:
//					   	   //store response of SBDIX
//
//					   	   //if moCode >= 0 && moCode <= 4
//					   	   	   //SBDIX successful
//					   	   	   //(mtCode == 1 && rxBuffer) -> retrieve RX message (SRDB)
//					   	   //else if (moCode == 12 || moCode == 14 || moCode == 16) fatal failure: no retry
//					   	   //else RETRY SBDIX
//		}
//
//		//wait for CSQ retry
//		serial_printf(cli_stdout, "csq in %d\n", ROCKBLOCK_CSQ_INTERVAL);
//		Task_sleep(ROCKBLOCK_CSQ_INTERVAL*1000);
//
//	//else timeout loop
//	}
//
//	serial_printf(cli_stdout, "RB s/r timeout\n",0);
//	return 0;
//}

//TODO: take care of split messages. So far message has to end within same SBD as it starts, otherwise it gets lost.
//decodes incoming messages into mavlink messages and posts them
//returns amount of decoded mavlink messages
int rockblock_internal_decode_SBD_binary(){

	int i;
	int msg_count = 0;
	mavlink_status_t status;
	COMM_FRAME frame;
	frame.direction = CHANNEL_IN;
	frame.channel = CHANNEL_ROCKBLOCK;

	for(i = 0; i < rockblock_rx_buffer_offset && i < ROCKBLOCK_MESSAGE_SIZE; i++){
		if(mavlink_parse_char(CHANNEL_ROCKBLOCK, (uint8_t)rockblock_rx_buffer[i], &(frame.mavlink_message), &status)){
			// --> deal with received message...
			msg_count++;
			Mailbox_post(comm_mailbox, &frame, BIOS_NO_WAIT);
		}
	}
	return msg_count;
}

int rockblock_do_SBDIX(uint16_t *moCode, uint16_t *moMSN, uint16_t *mtCode, uint16_t *mtMSN, uint16_t *mtLen, uint16_t *mtRemaining)
{

	char rxBuffer_uart[2 * ROCKBLOCK_RXBUFFER_SIZE];
	char *data_ptr;
	uint16_t *values[6] = { moCode, moMSN, mtCode, mtMSN, mtLen, mtRemaining };

	UART_write(uart, rockblock_at_sbdix, sizeof(rockblock_at_sbdix));
	UART_read(uart, rxBuffer_uart, sizeof(rxBuffer_uart));

	data_ptr = strstr(rxBuffer_uart, "+SBDIX: "); //find beginning of important information in buffer

	if(data_ptr == NULL){
		return 0; //ERROR, string not found in buffer
	}

	//decode returned numbers
	for (int i = 0; i < 6; i++)
	{
		char *p = strtok(i == 0 ? data_ptr + 8 : NULL, ", ");
		if (p == NULL){
			return 0;
		}
		*values[i] = atol(p);
	}
	return 1;
}

int rockblock_do_SBDRB(){

	char rxBuffer_uart[ROCKBLOCK_RXBUFFER_SIZE];
	uint16_t length;

	//make sure uart buffer is empty
	UART_read(uart, rxBuffer_uart, sizeof(rxBuffer_uart));

	//start sbdrb (binary message read)
	UART_write(uart, rockblock_at_sbdrb, sizeof(rockblock_at_sbdrb));

	//read back size of incoming message
	rxBuffer_uart[0] = 0xff; rxBuffer_uart[1] = 0xff;
	UART_read(uart, rxBuffer_uart, sizeof(2));
	length = (((uint16_t)rxBuffer_uart[0])<<8) + (uint16_t)rxBuffer_uart[1];

	//timeout (no 2 bytes read) if length longer than max possible length
	if(length > ROCKBLOCK_MESSAGE_SIZE)
		return 0;

	//prepare buffer
	rockblock_rx_buffer_offset = 0;
	memset(rockblock_rx_buffer, 0, sizeof(rockblock_rx_buffer));

	//read back message
	UART_read(uart, rockblock_rx_buffer, length);
	//save message length if successfull
	rockblock_rx_buffer_offset = length;

	//read back checksum
	UART_read(uart, rxBuffer_uart, sizeof(2));

	//return length of read message if success
	return length;
}

int rockblock_flush(){

	char rxBuffer_uart[ROCKBLOCK_RXBUFFER_SIZE];
	char rockblock_sbdwb[20]; //Buffer for at command
	int rockblock_sbdwb_length;
	int i;
	uint16_t checksum = 0;
	uint8_t temp;

	//start up rockblock
	if(!rockblock_begin() || uart == NULL){
//		serial_printf(cli_stdout, "RB begin error \n",0);
		rockblock_close();
		return 0;
	}

	//DEBUG
	//rockblock_tx_buffer_offset = 10;

	//initiate binary write to rockblock buffer
	int csq_temp = rockblock_get_signal_quality();
	while(csq_temp < 2 || csq_temp > 5)
		csq_temp = rockblock_get_signal_quality();

	rockblock_sbdwb_length = snprintf(rockblock_sbdwb, sizeof(rockblock_sbdwb), "AT+SBDWB=%d\r", rockblock_tx_buffer_offset);
	UART_write(uart, rockblock_sbdwb, rockblock_sbdwb_length);

	UART_read(uart, rxBuffer_uart, sizeof(rxBuffer_uart));


//	snprintf(rockblock_sbdwb, sizeof(rockblock_sbdwb), "1234567890");
//	rockblock_sbdwb_length = 10;
//	UART_write(uart, rockblock_sbdwb, rockblock_sbdwb_length);

	UART_write(uart, rockblock_tx_buffer, rockblock_tx_buffer_offset);
	//write message
	for (i = 0; i < rockblock_tx_buffer_offset; i++){
		checksum += (uint16_t)rockblock_tx_buffer[i];
	}

	//send checksum at end (2 bytes)
	//checksum = '1'+'2'+'3'+'4'+'5'+'6'+'7'+'8'+'9'+'0';
	temp = (uint8_t)(checksum >> 8);
	UART_write(uart, &temp, 1);
	temp = (uint8_t)(checksum & 0xFF);
	UART_write(uart, &temp, 1);

	UART_read(uart, rxBuffer_uart, sizeof(rxBuffer_uart));

	//wait for rockblock to get ready, return failed otherwise
	if(strstr(rxBuffer_uart, "READY\r\n") == NULL){
		rockblock_close();
		return 0;
	}

	////verify response, cancel if not correct
	//UART_read(uart, rxBuffer_uart, sizeof(rxBuffer_uart));
	//if(strstr(rxBuffer_uart, "0\r\n\r\nOK\r\n") == NULL){
	//	rockblock_close();
	//	return 0;
	//}

	//now start long SBD session to transmit message
	for(int starttime = Seconds_get(); (Seconds_get() - starttime) < ROCKBLOCK_SENDRECEIVE_TIME; ){

		int csq = rockblock_get_signal_quality();
		if(csq > ROCKBLOCK_CSQ_MINIMUM){
			if(rockblock_internal_MSSTM_workaround())
			{

		        uint16_t moCode = -1, moMSN = 0, mtCode = 0, mtMSN = 0, mtLen = 0, mtRemaining = 0;
		     	int mavlink_count = 0;

		        rockblock_do_SBDIX(&moCode, &moMSN, &mtCode, &mtMSN, &mtLen, &mtRemaining);

		        if(moCode >= 0 && moCode <= 4){
	        		serial_printf(cli_stdout, "SBDIX was success!\n");
	        		serial_printf(cli_stdout, "sent %d bytes\n", rockblock_tx_buffer_offset);
	        		rockblock_tx_buffer_offset = 0;

		        	rockblock_remaining_msg = mtRemaining; //update amount of messages stored on server
		        	if (mtCode == 1) // retrieved 1 message
					{
			        	//TODO, check incoming messages
		        		serial_printf(cli_stdout, "Incoming message!\n");
			        	rockblock_do_SBDRB();
			        	mavlink_count = rockblock_internal_decode_SBD_binary();
			        	serial_printf(cli_stdout, "RB received %d mavlink messages\n", mavlink_count);
					}
		        	rockblock_close();
		        	return 1;
		        }
		        else if(moCode == 12 || moCode == 14 || moCode == 16){
					serial_printf(cli_stdout, "RB s/r fatal error code: %d\n", moCode);
					rockblock_close();
					return 0;
		        }
			}
		}

		//wait for CSQ retry
		serial_printf(cli_stdout, "csq in %d\n", ROCKBLOCK_CSQ_INTERVAL);
		Task_sleep(ROCKBLOCK_CSQ_INTERVAL);

	//else timeout loop
	}

	serial_printf(cli_stdout, "RB s/r timeout\n",0);
	rockblock_close();
	return 0;
}



//add to rockblock buffer, send automatically if full
int rockblock_add_SBD_binary(uint8_t *tx_buffer, uint16_t *tx_buffersizePtr){
	int i = 0;
	int sbd_counter = 0;//measure how many SBD were sent
	if(tx_buffer && tx_buffersizePtr){
			while(ROCKBLOCK_MESSAGE_SIZE - rockblock_tx_buffer_offset < *tx_buffersizePtr - i){
				memcpy(&(rockblock_tx_buffer[rockblock_tx_buffer_offset]), &(tx_buffer[i]), ROCKBLOCK_MESSAGE_SIZE - rockblock_tx_buffer_offset);
				i += ROCKBLOCK_MESSAGE_SIZE - rockblock_tx_buffer_offset;
				rockblock_tx_buffer_offset = ROCKBLOCK_MESSAGE_SIZE;

				if(!rockblock_flush()){
					*tx_buffersizePtr = *tx_buffersizePtr - i;
					return -1;
				}
				sbd_counter++;
			}
			memcpy(&(rockblock_tx_buffer[rockblock_tx_buffer_offset]), &(tx_buffer[i]), *tx_buffersizePtr - i);
			rockblock_tx_buffer_offset += *tx_buffersizePtr - i;
	}

	*tx_buffersizePtr = 0;
	return sbd_counter;
}


//returns an average of signal quality
int rockblock_get_health(){
 	return rockblock_health;
}

int rockblock_get_remaining_msg(){
	return rockblock_remaining_msg;
}

int rockblock_get_sleep_status(){
	return GPIO_read(Board_ROCKBLOCK_SLEEP);
}


int rockblock_get_net_availability(){
	return 0; //TODO: decide on pin and initialize GPIO
//	return GPIO_read(Board_ROCKBLOCK_NET);
}

int rockblock_get_tx_buffer_fill(){
	return rockblock_tx_buffer_offset;
}

void rockblock_task(){
	int is_initialized = 0;
	static uint8_t buf[MAVLINK_MAX_PACKET_LEN + 5]; //todo: +5 is to test if we need some more space. remove if not helpful.
	static uint16_t mavlink_msg_len;
	static COMM_FRAME mail;

#ifdef ROCKBLOCK_ENABLED
	// init here:
	is_initialized = 1; //TODO: replace '1' by init function which shall return 1 if init successful


#endif

	while(1){

		if(Mailbox_pend(rockblock_mailbox, &mail, BIOS_WAIT_FOREVER)){

			if(is_initialized)
			{
				if((mail.direction) == CHANNEL_OUT)
				{
					if(mail.channel == CHANNEL_ROCKBLOCK)
					{
						mavlink_msg_len = mavlink_msg_to_send_buffer(buf, &mail.mavlink_message);
						rockblock_add_SBD_binary(buf, &mavlink_msg_len);
					}
					else
					{
						serial_printf(cli_stdout,"ERROR: outgoing msg for channel %d in rockblock_mailbox. id=%d \n", mail.channel, mail.mavlink_message.msgid);
					}
				}
				else
				{
					serial_printf(cli_stdout,"ERROR: incoming msg in rockblock_mailbox. id=%d \n", mail.mavlink_message.msgid);
					// should never happen, but just in case: redirect message!
					Mailbox_post(comm_mailbox, &mail, BIOS_NO_WAIT);
				}
			}
		}
 	}
}



