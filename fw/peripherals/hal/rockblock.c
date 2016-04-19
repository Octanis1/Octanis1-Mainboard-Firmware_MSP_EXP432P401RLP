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

#include "../../../Board.h"
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

static int last_powerup_time = 0;
static int rockblock_health = 0;

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
//		cli_printf("RB open error \n",0);
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
		Task_sleep(elapsed_time);
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
	if(uart != NULL){

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
	}

	return csq_val;
}


//sends an SBD, then checks the inbox (checking costs 1 credit!)
int rockblock_send_receive_SBD(const uint8_t *tx_buffer, size_t tx_buffersize,
								uint8_t *rx_buffer, size_t *rx_buffersizePtr){
//	cli_printf("RBsr\n",0);

	if(!rockblock_begin()){
//		cli_printf("RB begin error \n",0);
		return 0;
	}

	//write our data into the buffer of the rockblock (max 340B)

	 //"AT+SBDWB=120\r" 120Bytes to be sent
	 //wait for "READY\r\n"
	/* write data to rockblock and compute checksum
	      uint16_t checksum = 0;
	      for (int i=0; i<txDataSize; ++i)
	      {
	         stream.write(txData[i]);
	         checksum += (uint16_t)txData[i];
	      }
	*/
	//write computed checksum to RB
	/*
	      stream.write(checksum >> 8);
	      stream.write(checksum & 0xFF);
	*/
	//WAIT FOR "0\r\n\r\nOK\r\n"


	//now start long SBD session to transmit message
	for(int starttime = Seconds_get(); (Seconds_get() - starttime) < ROCKBLOCK_SENDRECEIVE_TIME; ){

		int csq = rockblock_get_signal_quality();
		if(csq > ROCKBLOCK_CSQ_MINIMUM){
			//perform MSST workaround

					   	   //AT+SBDIX\r
					   	   //wait for +SBDIX:
					   	   //store response of SBDIX

					   	   //if moCode >= 0 && moCode <= 4
					   	   	   //SBDIX successful
					   	   	   //(mtCode == 1 && rxBuffer) -> retrieve RX message (SRDB)
					   	   //else if (moCode == 12 || moCode == 14 || moCode == 16) fatal failure: no retry
					   	   //else RETRY SBDIX
		}

		//wait for CSQ retry
		cli_printf("csq in %d\n", ROCKBLOCK_CSQ_INTERVAL);
		Task_sleep(ROCKBLOCK_CSQ_INTERVAL*1000);

	//else timeout loop
	}

	cli_printf("RB s/r timeout\n",0);
	return 0;

}

//gets an SBD message from rockblocks buffer
int rockblock_get_SBD_binary(uint8_t *rx_buffer, size_t *rx_buffersizePtr){

	return 0;
}


//returns an average of signal quality
int rockblock_get_health(){
 	return rockblock_health;
}



int rockblock_get_sleep_status(){
	return GPIO_read(Board_ROCKBLOCK_SLEEP);
}


int rockblock_get_net_availability(){
	return GPIO_read(Board_ROCKBLOCK_NET);
}

