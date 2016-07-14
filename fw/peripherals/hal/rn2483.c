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

#define RN2483_RXBUFFER_SIZE 20
#define RN2483_READ_TIMEOUT 1000
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

static void rn2483_uart_open(UART_SerialDevice *dev) {
	static UART_Params uartParams;

	UART_Params_init(&uartParams);
	uartParams.writeDataMode = UART_DATA_BINARY;
	uartParams.readDataMode = UART_DATA_BINARY;
	uartParams.readReturnMode = UART_RETURN_FULL;
	uartParams.readTimeout = RN2483_READ_TIMEOUT;
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

//must be called from within a task - this function will block!
int rn2483_begin(){

	//	reset the rn2483
    GPIO_write(Board_LORA_RESET_N, 0);

	Task_sleep(500);

#ifndef CONFIG_MODE
	rn2483_uart_open(&lora_uart_dev);
	lora_serialdev = (SerialDevice *)&lora_uart_dev;
#endif

	GPIO_write(Board_LORA_RESET_N, 1);
	char rn2483_rxBuffer[RN2483_RXBUFFER_SIZE];

//	UART_write(rn2483_uart, rn_reset, sizeof(rn_reset));
//	UART_read(rn2483_uart, rn2483_rxBuffer, sizeof(rn2483_rxBuffer));

	Task_sleep(500);
	UART_read(rn2483_uart, rn2483_rxBuffer, sizeof(rn2483_rxBuffer)); //clean the reset message
	UART_read(rn2483_uart, rn2483_rxBuffer, sizeof(rn2483_rxBuffer));
	memset(&rn2483_rxBuffer, 0, sizeof(rn2483_rxBuffer));


//	/*check parameters*/
//	Task_sleep(500);
//	UART_write(rn2483_uart, rn_get_devaddr, sizeof(rn_get_devaddr));
//	UART_read(rn2483_uart, rn2483_rxBuffer, sizeof(rn2483_rxBuffer));
//	Task_sleep(500);

	UART_write(rn2483_uart, rn_get_deveui, sizeof(rn_get_deveui));
	UART_read(rn2483_uart, rn2483_rxBuffer, sizeof(rn2483_rxBuffer));
	Task_sleep(500);

	serial_printf(cli_stdout, "rn2483 deveui: %s.\r\n", rn2483_rxBuffer);

	UART_write(rn2483_uart, rn_join_otaa, sizeof(rn_join_otaa));
	Task_sleep(10000); // Time needed to complete OTAA.
	memset(&rn2483_rxBuffer, 0, sizeof(rn2483_rxBuffer));
	UART_read(rn2483_uart, rn2483_rxBuffer, sizeof(rn2483_rxBuffer));

	if(!strcmp("ok\r\naccepted\r\n", rn2483_rxBuffer)){
		serial_printf(cli_stdout, "rn2483 OTAA success: %s\r\n", rn2483_rxBuffer);
		rn2483_initialised = 1;
		return 1; //modem can now communicate with us
	}else{
		serial_printf(cli_stdout, "rn2483 error: OTAA failed. Message: %s\r\n", rn2483_rxBuffer);
		return 0;
	}

}


void rn2483_end(){

	rn2483_initialised = 0;
	UART_close(rn2483_uart);
	rn2483_uart = NULL;

	//put module to sleep
	//GPIO_write(Board_ROCKBLOCK_SLEEP, ROCKBLOCK_SLEEP);
}

#ifdef CONFIG_MODE
/*Used to configure the module with network data etc. */
int rn2483_config()
{
	int comm_result = 0;

	//	reset the rn2483
    GPIO_write(Board_LORA_RESET_N, 0);
    GPIO_write(Board_LORA_RESET_N, 1);
    	char rn2483_rxBuffer[RN2483_RXBUFFER_SIZE];

	rn2483_uart_open(&lora_uart_dev);

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

	// NOT needed for OTAA:
//	UART_write(rn2483_uart, rn_set_deveui, sizeof(rn_set_deveui));
//	UART_read(rn2483_uart, rn2483_rxBuffer, sizeof(rn2483_rxBuffer));
//	Task_sleep(500);
//	comm_result += !(rn2483_rxBuffer[0] == 'o' && rn2483_rxBuffer[1] == 'k');
//	rn2483_rxBuffer[0] = 0;

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
	char rn2483_rxBuffer[RN2483_RXBUFFER_SIZE];
	UART_read(rn2483_uart, rn2483_rxBuffer, sizeof(rn2483_rxBuffer)); //clear the buffer: after 1st "ok", it receives a \n before "ok".
												// this is a pretty bad hack and we should look where the character comes from .

	memset(&rn2483_rxBuffer, 0, sizeof(rn2483_rxBuffer));
	char txBuffer[tx_size+18]; //18 for the aditionnal lora commands
	memset(&txBuffer, 0, sizeof(txBuffer)); //important! always clean buffer before tx


	if(!rn2483_initialised) return 0;

	strcat(txBuffer, rn_txstart);
	strcat(txBuffer, tx_buffer);
	strcat(txBuffer, rn_txstop);

	int tx_ret = UART_write(rn2483_uart, txBuffer, strlen(txBuffer));
	int rx_ret = UART_read(rn2483_uart, rn2483_rxBuffer, 4); // 4 to read "ok\r\n"
	Task_sleep(1000);

	if(!strcmp("ok\r\n", rn2483_rxBuffer))
	{
		int rx_ret = UART_read(rn2483_uart, rn2483_rxBuffer, 9); //7 to read "mac rx "

		serial_printf(cli_stdout, "LoRa TX: %d \n", tx_ret);
		GPIO_toggle(Board_LED_GREEN);

		int comp=strcmp("mac_rx", rn2483_rxBuffer);

//		if(!strcmp("mac_rx", rn2483_rxBuffer))
		if(1)
		{
			// we received downlink message
			serial_printf(cli_stdout, "TX successful. RX: %s\r\n", rn2483_rxBuffer);

			int i, port, digit;
			int c;
			uint8_t mav_byte;
			port = 0;
			// read port number:
//			while((UART_read(rn2483_uart, &c, 1) == 1)){
//				digit = a2d(c);
//				if(digit < 0)
//					break;
//				else
//					port = port*10 + digit;
//			}

			COMM_FRAME frame;
			frame.direction = CHANNEL_IN;
			frame.channel = CHANNEL_LORA;
			mavlink_status_t status;

			while((c = serial_getc(lora_serialdev)) >= 0) {
				serial_printf(cli_stdout, "%d", c);
				mav_byte = ((uint8_t)a2d(c))<<4;
				if((c = serial_getc(lora_serialdev)) >= 0)
				{
					serial_printf(cli_stdout, "%d", c);
					mav_byte += ((uint8_t)a2d(c));}
				else
					break;

				if(mavlink_parse_char(CHANNEL_LORA, mav_byte, &(frame.mavlink_message), &status)){
					serial_printf(cli_stdout, "mavlink msg received over lora: ID=%d\r\n", frame.mavlink_message.msgid);
					Mailbox_post(comm_mailbox, &frame, BIOS_NO_WAIT);
				}
			}
			i++;
		}
		else if(!strcmp("mac_tx_ok", rn2483_rxBuffer))
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
		return 0;
	}


}


