/*
 *  File: rn2483.c
 *  Description: Model for LoRaWAN tranceiver "Microchip RN2483"
 *  Author: Sam
 */

#include "../../../Board.h"
#include "rn2483.h"
#include "../comm.h"
#include "uart_helper.h"
#include <serial.h>
#include "../../lib/printf.h"
#include "../../lib/mavlink/mavlink_helpers.h"

#define RN2483_RXBUFFER_SIZE 40
#define RN2483_READ_TIMEOUT_OPEN 1000 //during opening phase, should be shorter since we rely on it.
#define RN2483_READ_TIMEOUT 15000 //to avoid getting trapped waiting for ever.
#define RN2483_BAUD_RATE 57600

static UART_Handle rn2483_uart;
SerialDevice *lora_serialdev;
static UART_SerialDevice lora_uart_dev;
static int rn2483_initialised = 0;

#ifdef CONFIG_MODE
//responses from the modem are "encapsulated" in "\r\n"
#define RN2483_CONFIG_LENGTH 50 //maximum length of the below 4 strings
	#ifdef CONFIG_SWISSCOM
		static const char rn_set_nwkskey[] ="mac set nwkskey A4F613D0D65CC3DA3D874D82E12265B8\r\n";
		static const char rn_set_appskey[] ="mac set appskey 32BC3ABB9E1ADC0594811AEB1CCA704F\r\n";
		static const char rn_set_devaddr[] ="mac set devaddr 08050046\r\n";
		static const char rn_set_deveui[] ="mac set deveui F03D291000000046\r\n";
		static const char rn_save[] ="mac save\r\n";
	#else //thethingsnetwork login:
//		static const char rn_set_nwkskey[] ="mac set nwkskey 2B7E151628AED2A6ABF7158809CF4F3C\r\n"; //(old config values)
//		static const char rn_set_appskey[] ="mac set appskey 2B7E151628AED2A6ABF7158809CF4F3C\r\n";
//		static const char rn_set_devaddr[] ="mac set devaddr 08050046\r\n";
		static const char rn_set_nwkskey[] ="mac set nwkskey FAE2006DB71C34F2DDA6C33C19D92858\r\n";
		static const char rn_set_appskey[] ="mac set appkey 01020304050607080910111213141516\r\n";
		static const char rn_set_devaddr[] ="mac set devaddr 020312A1\r\n";
		static const char rn_set_deveui[] =	"mac set deveui F03D291000000046\r\n";
		static const char rn_set_appeui[] =	"mac set appeui 70b3d57ed0000172\r\n"; // Octanis 1 App on the Field Station
		static const char rn_get_dr[] =	"mac get dr\r\n"; //
		static char rn_get_duty1[] =	"mac get ch dcycle 0\r\n"; //
		static char rn_set_duty10[] =	"mac set ch dcycle 10 9\r\n"; //
		static char rn_set_duty1[] =	"mac set ch dcycle 0 9\r\n"; //

		static char rn_get_duty10[] =	"mac get ch dcycle 10\r\n"; //
		static char rn_get_freq1[] =	"mac get ch freq 1\r\n"; //
		static char rn_get_drrange1[] =	"mac get ch drrange 1\r\n"; //



		static const char rn_save[] ="mac save\r\n";
	#endif
#endif


static const char rn_get_devaddr[] = "mac get devaddr\r\n";
static const char rn_get_deveui[] = "mac get deveui\r\n";
static const char rn_get_nwkskey[] = "mac get nwkskey\r\n";
static const char rn_get_appskey[] = "mac get appskey\r\n";


static const char rn_txbeacon[] = "mac tx uncnf 1 deadbeef\r\n";
static const char rn_txstart[] = "mac tx uncnf 1 ";
static const char rn_txstop[] = "\r\n";

static const char rn_join_abp[] = "mac join abp\r\n";
static const char rn_join_otaa[] = "mac join otaa\r\n";
static const char rn_reset[] = "sys reset\r\n";

//buffer for command responses
static char rn2483_rxBuffer[LORA_FRAME_SIZE + 20]; //plus some margin for "mac_rx 1 .... \r\n". it's too much but...

/************ from the RN2483 Command Reference User's Guide: ****************
 * "All commands need to be terminated with <CR><LF> and any replies they
 * generate will also be terminated by the same sequence.
 * The default settings for the UART interface are 57600 bps,
 * 8 bits, no parity, 1 Stop bit, no flow control
*/

static void rn2483_uart_open(UART_SerialDevice *dev, unsigned int read_timeout_mode) {
	static UART_Params uartParams;

	UART_Params_init(&uartParams);
	uartParams.writeDataMode = UART_DATA_BINARY;
#ifdef CONFIG_MODE
	uartParams.readDataMode = UART_DATA_BINARY;
	uartParams.readReturnMode = UART_RETURN_FULL;
#else
	uartParams.readDataMode = UART_DATA_TEXT;
	uartParams.readReturnMode = UART_RETURN_NEWLINE;
#endif
	uartParams.readTimeout = read_timeout_mode;
    uartParams.writeMode = UART_MODE_BLOCKING;
	uartParams.readEcho = UART_ECHO_OFF;
	uartParams.baudRate = 57600;
	rn2483_uart = UART_open(Board_UART3_LORACOMM, &uartParams);

	 if (rn2483_uart == NULL) {
		System_abort("Error opening the UART");
	}

	dev->fntab = &UART_SerialDevice_fntab;
	dev->uart = rn2483_uart;
}

/*this function is intended to use with commands that expect an "ok\r\n" as first reply.
*	If it is the case, the second answer will be stored in the global rxBuffer. fx returns its length (excluding the \r).
*	If not, the first answer will remain and the function returns 0.
*/
int rn2483_read_command_response(int read_length2)
{
	int length=0;
	char dummy;
	length=UART_read(rn2483_uart, rn2483_rxBuffer, sizeof(rn2483_rxBuffer));

	if(length<RN2483_RXBUFFER_SIZE)
		UART_read(rn2483_uart, &dummy, 1); //read the \n character

	if(!strncmp("ok", rn2483_rxBuffer, 2))
	{
		length = UART_read(rn2483_uart, rn2483_rxBuffer, read_length2);
		char dummy;
		UART_read(rn2483_uart, &dummy, 1); //read the \n character

		return length - 1;
	}
	else
	{
		return 0;
	}
}

//must be called from within a task - this function will block!
int rn2483_begin(){
	unsigned int n_rx;

	//	reset the rn2483
    GPIO_write(Board_LORA_RESET_N, 0);

	Task_sleep(500);

#ifndef CONFIG_MODE
	rn2483_uart_open(&lora_uart_dev, RN2483_READ_TIMEOUT_OPEN);
	lora_serialdev = (SerialDevice *)&lora_uart_dev;
#endif

	GPIO_write(Board_LORA_RESET_N, 1);

//	UART_write(rn2483_uart, rn_reset, sizeof(rn_reset));
//	UART_read(rn2483_uart, rn2483_rxBuffer, sizeof(rn2483_rxBuffer));

	Task_sleep(500);
	UART_read(rn2483_uart, rn2483_rxBuffer, sizeof(rn2483_rxBuffer)); //clean the reset message
	memset(&rn2483_rxBuffer, 0, sizeof(rn2483_rxBuffer));

#ifndef CONFIG_MODE
	//the reset message does not contain a newline character. all other command responses do.
	// therefore we reinitialize the uart with a wait-forever readMode.
	rn2483_end();
	rn2483_uart_open(&lora_uart_dev, RN2483_READ_TIMEOUT);
#endif


//	/*check parameters*/
//	Task_sleep(500);
//	UART_write(rn2483_uart, rn_get_devaddr, sizeof(rn_get_devaddr));
//	UART_read(rn2483_uart, rn2483_rxBuffer, sizeof(rn2483_rxBuffer));
//	Task_sleep(500);

	UART_write(rn2483_uart, rn_get_deveui, sizeof(rn_get_deveui));
	UART_read(rn2483_uart, rn2483_rxBuffer, sizeof(rn2483_rxBuffer));
	UART_read(rn2483_uart, rn2483_rxBuffer, sizeof(rn2483_rxBuffer));

	serial_printf(cli_stdout, "rn2483 deveui: %s.\r\n", rn2483_rxBuffer);
	memset(&rn2483_rxBuffer, 0, sizeof(rn2483_rxBuffer));

	UART_write(rn2483_uart, rn_join_otaa, sizeof(rn_join_otaa));

	if((rn2483_read_command_response(RN2483_RXBUFFER_SIZE)>0) && (strncmp("accepted", rn2483_rxBuffer,8)==0)){
		serial_printf(cli_stdout, "rn2483 OTAA success: %s\r\n", rn2483_rxBuffer);
		rn2483_initialised = 1;
		GPIO_write(Board_LED_RED,1);
		return 1; //modem can now communicate with us
	}else{
		serial_printf(cli_stdout, "rn2483 error: OTAA failed. Message: %s\r\n", rn2483_rxBuffer);
		rn2483_end();
		return 0;
	}

}


void rn2483_end(){

	rn2483_initialised = 0;
	UART_close(rn2483_uart);
	rn2483_uart = NULL;
}

#ifdef CONFIG_MODE
/*Used to configure the module with network data etc. */
int rn2483_config()
{
	int comm_result = 0;

	//	reset the rn2483
    GPIO_write(Board_LORA_RESET_N, 0);
    GPIO_write(Board_LORA_RESET_N, 1);
    	uint8_t rn2483_rxBuffer[RN2483_RXBUFFER_SIZE];

	rn2483_uart_open(&lora_uart_dev, 1000);

	Task_sleep(500);
	UART_read(rn2483_uart, rn2483_rxBuffer, sizeof(rn2483_rxBuffer)); //clean the reset message
	UART_read(rn2483_uart, rn2483_rxBuffer, sizeof(rn2483_rxBuffer));
	memset(&rn2483_rxBuffer, 0, sizeof(rn2483_rxBuffer));

	Task_sleep(500);
	UART_write(rn2483_uart, rn_set_devaddr, sizeof(rn_set_devaddr));
	UART_read(rn2483_uart, rn2483_rxBuffer, sizeof(rn2483_rxBuffer));
	Task_sleep(500);
	comm_result += !(rn2483_rxBuffer[0] == 'o' && rn2483_rxBuffer[1] == 'k');
	rn2483_rxBuffer[0] = 0;

	// needed for OTAA:
	UART_write(rn2483_uart, rn_set_deveui, sizeof(rn_set_deveui));
	UART_read(rn2483_uart, rn2483_rxBuffer, sizeof(rn2483_rxBuffer));
	Task_sleep(500);
	comm_result += !(rn2483_rxBuffer[0] == 'o' && rn2483_rxBuffer[1] == 'k');
	rn2483_rxBuffer[0] = 0;

	UART_write(rn2483_uart, rn_set_appeui, sizeof(rn_set_appeui));
	UART_read(rn2483_uart, rn2483_rxBuffer, sizeof(rn2483_rxBuffer));
	Task_sleep(500);
	comm_result += !(rn2483_rxBuffer[0] == 'o' && rn2483_rxBuffer[1] == 'k');
	rn2483_rxBuffer[0] = 0;

	// NOT needed for OTAA:
//	UART_write(uart, rn_set_nwkskey, sizeof(rn_set_nwkskey));
//	UART_read(uart, rn2483_rxBuffer, sizeof(rn2483_rxBuffer));
//	Task_sleep(500);
//	comm_result += !(rn2483_rxBuffer[0] == 'o' && rn2483_rxBuffer[1] == 'k');
//	rn2483_rxBuffer[0] = 0;

	UART_write(rn2483_uart, rn_set_appskey, sizeof(rn_set_appskey));
	UART_read(rn2483_uart, rn2483_rxBuffer, sizeof(rn2483_rxBuffer));
	Task_sleep(500);
	comm_result += !(rn2483_rxBuffer[0] == 'o' && rn2483_rxBuffer[1] == 'k');
	rn2483_rxBuffer[0] = 0;

	UART_write(rn2483_uart, rn_get_dr, sizeof(rn_get_dr));
	Task_sleep(500);
	UART_read(rn2483_uart, rn2483_rxBuffer, sizeof(rn2483_rxBuffer));
	Task_sleep(500);
	serial_printf(cli_stdout, "bitrate %s \r\n", rn2483_rxBuffer);

#ifdef CONFIG_HIGH_DUTY
	int ch_id;
	for(ch_id = 0;ch_id<10;ch_id++)
	{
		UART_write(rn2483_uart, rn_set_duty1, sizeof(rn_set_duty1));
		Task_sleep(100);
		UART_read(rn2483_uart, rn2483_rxBuffer, sizeof(rn2483_rxBuffer));
		Task_sleep(100);
		serial_printf(cli_stdout, "duty %u %s \r\n",ch_id, rn2483_rxBuffer);
		rn_set_duty1[18]++;
	}

	for(ch_id = 10;ch_id<16;ch_id++)
	{
		UART_write(rn2483_uart, rn_set_duty10, sizeof(rn_set_duty10));
		Task_sleep(100);
		UART_read(rn2483_uart, rn2483_rxBuffer, sizeof(rn2483_rxBuffer));
		Task_sleep(100);
		serial_printf(cli_stdout, "duty %u %s \r\n",ch_id, rn2483_rxBuffer);
		rn_set_duty10[19]++;
	}

	for(ch_id = 0;ch_id<10;ch_id++)
	{
		UART_write(rn2483_uart, rn_get_duty1, sizeof(rn_get_duty1));
		Task_sleep(100);
		UART_read(rn2483_uart, rn2483_rxBuffer, sizeof(rn2483_rxBuffer));
		Task_sleep(100);
		serial_printf(cli_stdout, "duty %u %s \r\n",ch_id, rn2483_rxBuffer);
		rn_get_duty1[18]++;
	}

	for(ch_id = 10;ch_id<16;ch_id++)
	{
		UART_write(rn2483_uart, rn_get_duty10, sizeof(rn_get_duty10));
		Task_sleep(100);
		UART_read(rn2483_uart, rn2483_rxBuffer, sizeof(rn2483_rxBuffer));
		Task_sleep(100);
		serial_printf(cli_stdout, "duty %u %s \r\n",ch_id, rn2483_rxBuffer);
		rn_get_duty10[19]++;
	}

	for(ch_id = 0;ch_id<10;ch_id++)
	{
		UART_write(rn2483_uart, rn_get_drrange1, sizeof(rn_get_drrange1));
		Task_sleep(100);
		UART_read(rn2483_uart, rn2483_rxBuffer, sizeof(rn2483_rxBuffer));
		Task_sleep(100);
		serial_printf(cli_stdout, "drrange %u %s \r\n",ch_id, rn2483_rxBuffer);
		rn_get_drrange1[19]++;
	}

	for(ch_id = 0;ch_id<10;ch_id++)
	{
		UART_write(rn2483_uart, rn_get_freq1, sizeof(rn_get_freq1));
		Task_sleep(100);
		UART_read(rn2483_uart, rn2483_rxBuffer, sizeof(rn2483_rxBuffer));
		Task_sleep(100);
		serial_printf(cli_stdout, "freq %u %s \r\n",ch_id, rn2483_rxBuffer);
		rn_get_freq1[19]++;
	}

#endif
	UART_write(rn2483_uart, rn_save, sizeof(rn_save));
	Task_sleep(500);
	UART_read(rn2483_uart, rn2483_rxBuffer, sizeof(rn2483_rxBuffer));
	Task_sleep(500);
	comm_result += !(rn2483_rxBuffer[0] == 'o' && rn2483_rxBuffer[1] == 'k');

	return comm_result;
}
#endif



int rn2483_send_receive(char * tx_buffer, int tx_size)
{
//	UART_read(rn2483_uart, rn2483_rxBuffer, sizeof(rn2483_rxBuffer)); //clear the buffer: after 1st "ok", it receives a \n before "ok".
												// this is a pretty bad hack and we should look where the character comes from .

	char txBuffer[tx_size+18]; //18 for the aditionnal lora commands
	memset(&txBuffer, 0, sizeof(txBuffer)); //important! always clean buffer before tx

	if(!rn2483_initialised) return 0;

	strcat(txBuffer, rn_txstart);
	strcat(txBuffer, tx_buffer);
	strcat(txBuffer, rn_txstop);

	int tx_ret = UART_write(rn2483_uart, txBuffer, strlen(txBuffer));

	int rx_ret = rn2483_read_command_response(sizeof(rn2483_rxBuffer));  //read the whole line

	if(rx_ret)
	{
		serial_printf(cli_stdout, "LoRa TX: %d \n", tx_ret);
		GPIO_toggle(Board_LED_GREEN);

		if(!strncmp("mac_rx", rn2483_rxBuffer, 6))
		{
			// we received downlink message
			serial_printf(cli_stdout, "RX successful. RX: %s\r\n", rn2483_rxBuffer);

			int i;

			uint8_t mav_byte;

			// TODO: read port number, if relevant

			COMM_FRAME frame;
			frame.direction = CHANNEL_IN;
			frame.channel = CHANNEL_LORA;
			mavlink_status_t status;

			for(i=9;i<rx_ret;i++)
			{
//				serial_printf(cli_stdout, "%c,", c);
//				serial_printf(cli_stdout, "%d,", c);
				mav_byte = ((uint8_t)a2d(rn2483_rxBuffer[i]))<<4;
				i++;
				if(rn2483_rxBuffer[i] > '\r')
				{
//					serial_printf(cli_stdout, "%c,", c);
//					serial_printf(cli_stdout, "%d,", c);
					mav_byte += ((uint8_t)a2d(rn2483_rxBuffer[i]));}
				else
					break;
//
//				serial_printf(cli_stdout, "%x\n", mav_byte);

				if(mavlink_parse_char(CHANNEL_LORA, mav_byte, &(frame.mavlink_message), &status)){
					serial_printf(cli_stdout, "mavlink msg received over lora: ID=%d\r\n", frame.mavlink_message.msgid);
					Mailbox_post(comm_mailbox, &frame, BIOS_NO_WAIT);
				}
			}
			serial_printf(cli_stdout, "# hex string char=%d\r\n", i);


		}
		else if(!strncmp("mac_tx_ok", rn2483_rxBuffer,9))
		{
			// no downlink message
			serial_printf(cli_stdout, "TX successful \r\n");
		}
		else
		{
			serial_printf(cli_stdout, "unknown confirmation: %s \r\n", rn2483_rxBuffer);
		}

		return 1;
	}
	else
	{
		serial_printf(cli_stdout, "TX rejected: %s \r\n",rn2483_rxBuffer);
		return 0;
	}


}


