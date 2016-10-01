#include "../../../Board.h"
#include "ublox_6.h"

static char ublox_rxBuffer[UBLOX_6_NMEABUFFER_SIZE];

static UART_Handle ublox_uart;

void ublox_6_open(){
	static UART_Params uartParams;

	/* Create a UART with data processing off. */
	UART_Params_init(&uartParams);
	uartParams.writeDataMode = UART_DATA_BINARY;
	uartParams.readDataMode = UART_DATA_BINARY;
	uartParams.readReturnMode = UART_RETURN_FULL; //one NMEA frame per read
	uartParams.readEcho = UART_ECHO_OFF;
	uartParams.baudRate = 115200;
	uartParams.readMode = UART_MODE_BLOCKING;
	//uartParams.readTimeout = 10;
	uartParams.dataLength = UART_LEN_8;

	//Correct port for the mainboard
	ublox_uart = UART_open(Board_UART1_GPS, &uartParams);

	if (ublox_uart == NULL)
		System_abort("Error opening the GPS UART");
}

void ublox_6_close(){

	UART_close(ublox_uart);

}



char* ublox_6_read(){

//	int i=0;
	UART_read(ublox_uart, ublox_rxBuffer, sizeof(ublox_rxBuffer));

//	for(i=0;ublox_rxBuffer[i]!='\0';i++){
//		System_printf("%c \n", ublox_rxBuffer[i]);
	    	/* SysMin will only print to the console when you call flush or exit */
//	    System_flush();
//	}
	return ublox_rxBuffer;
}

uint8_t ublox6_read_bytes(uint8_t* buf, int count)
{
	return UART_read(ublox_uart, buf, count);
}

int ublox_6_write(uint8_t* ublox_txBuffer, int count){
	return UART_write(ublox_uart, ublox_txBuffer, count);
}


/*********** neo-6 config code. adapted from https://github.com/cuspaceflight/Woodchuck/blob/165ecc12db24852cf0c3019f28f655765c0b923c/firmware/gps.c **************/

// Header sync bytes - no touching!
#define UBX_SYNC_1  (0xB5)
#define UBX_SYNC_2  (0x62)

// Class and ID bytes - no touching!
#define UBX_NAV_CLASS       (0x01)
#define UBX_ACK_CLASS       (0x05)
#define UBX_CFG_CLASS       (0x06)

#define UBX_NAV_POSLLH_ID   (0x02)
#define UBX_NAV_SOL_ID      (0x06)
#define UBX_NAV_TIMEUTC_ID  (0x21)
#define UBX_ACK_ACK_ID      (0x01)
#define UBX_CFG_PRT_ID      (0x00)
#define UBX_CFG_MSG_ID      (0x01)
#define UBX_CFG_NAV5_ID     (0x24)



bool ublox6_verify_checksum(uint8_t* data, uint8_t len);
void ublox6_ubx_checksum(uint8_t* data, uint8_t len, uint8_t* cka, uint8_t* ckb);

/**
 * Set the GPS to <1g airborne navigation mode, using a CFG-NAV5 message.
 */
void gps_set_mode(void)
{
    /* Parameters:
     * mask:        0x0001 (only apply dynModel setting)
     * dynModel:    0x06 (airborne <1g)
     * blank x33:   0x00 (not needed due to mask)
     *
     * Checksum bytes are pre-computed
     */

    uint8_t buf[10];

    uint8_t request[44] = {UBX_SYNC_1, UBX_SYNC_2, UBX_CFG_CLASS,
        UBX_CFG_NAV5_ID, 0x00, 36, 0x00, 0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x55, 0x8f};
    ublox_6_write(request, 44);

    // Read the response from the GPS
    ublox6_read_bytes(buf, 10);

    // Verify sync and header bytes
    if( buf[0] != UBX_SYNC_1 || buf[1] != UBX_SYNC_2 )
        GPIO_write(Board_LED_RED, Board_LED_ON);
    if( buf[2] != UBX_ACK_CLASS)
        GPIO_write(Board_LED_RED, Board_LED_ON);

    // Check message checksum
    if( !ublox6_verify_checksum(&buf[2], 6) )
    		GPIO_write(Board_LED_RED, Board_LED_ON);


    // Check if we received an ACK or NACK
    if(buf[3] != UBX_ACK_ACK_ID){ // If not an ACK
        GPIO_write(Board_LED_RED, Board_LED_ON);
    }
}

/**
 * Verify that the uBlox 6 GPS receiver is set to the <1g airborne
 * navigaion mode, using a CFG-NAV5 message.
 */
uint8_t gps_check_nav(void)
{
    uint8_t request[8] = {UBX_SYNC_1, UBX_SYNC_2, UBX_CFG_CLASS, UBX_CFG_NAV5_ID, 0x00, 0x00,
        0x2A, 0x84};
    ublox_6_write(request, 8);

    // Get the message back from the GPS
    uint8_t buf[44];
    ublox6_read_bytes(buf, 44);

    // Verify sync and header bytes
    if( buf[0] != UBX_SYNC_1 || buf[1] != UBX_SYNC_2 )
        GPIO_write(Board_LED_RED, Board_LED_ON);
    if( buf[2] != UBX_CFG_CLASS || buf[3] != UBX_CFG_NAV5_ID )
        GPIO_write(Board_LED_RED, Board_LED_ON);

    // Check 40 bytes of message checksum
    if( !ublox6_verify_checksum(&buf[2], 40) )
        GPIO_write(Board_LED_RED, Board_LED_ON);

    // Clock in and verify the ACK/NACK
    uint8_t ack[10];
    ublox6_read_bytes(ack,10);

    // If we got a NACK, then return 0xFF
    if( ack[3] == 0x00 ) return 0xFF;

    // Return the navigation mode and let the caller analyse it
    return buf[8];
}

///**
// * Set the baudrate of the GPS using a CFG-PRT message.
// * We also update our own baudrate to match.
// */
//void _gps_set_baud(uint32_t baudrate)
//{
//    // Backup the old baudrate incase something goes wrong
//    uint32_t oldBRR = USART_BRR(USART);
//
//    /* Parameters:
//     * portID:      0x01 (UART)
//     * reserved:    0x00
//     * txReady:     0x0000 (disable txReady pin)
//     * mode:        0x000008C0 (1 stop bit, no parity, 8 data bits)
//     * baudrate:    <baudrate> (little endian)
//     * inProtoMask: 0x0001 (UBX input only)
//     * outProtoMask:0x0001 (UBX output only)
//     * flags:       0x0000 (disable extended timeout)
//     * reserved x2: 0x0000
//     */
//    uint8_t cka = 0;
//    uint8_t ckb = 0;
//    uint8_t request[28] = {UBX_SYNC_1, UBX_SYNC_2, UBX_CFG_CLASS,
//        UBX_CFG_PRT_ID, 0x00, 20, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0xC0,
//        baudrate & 0xff, (baudrate >> 8) & 0xff, (baudrate >> 16) & 0xff,
//        (baudrate >> 24) & 0xff, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
//        0x00, cka, ckb};
//
//    // Compute the checksum and fill it out
//    _gps_ubx_checksum(&request[2], 24, &cka, &ckb);
//    request[26] = cka;
//    request[27] = ckb;
//
//    // Transmit the request
//    _gps_send_msg(request, 27);
//
//    // Change our baud rate to match the new speed
//    usart_set_baudrate(USART, baudrate);
//
//    // Read the response from the GPS
//    uint8_t buf[9];
//    uint8_t i = 0;
//    for(i = 0; i < 9; i++)
//        buf[i] = _gps_get_byte();
//
//    // Verify sync and header bytes
//    if( buf[0] != UBX_SYNC_1 || buf[1] != UBX_SYNC_2 )
//        led_set(LED_RED, 1);
//    if( buf[2] != UBX_ACK_CLASS)
//        led_set(LED_RED, 1);
//
//    // Check message checksum
//    if( !_gps_verify_checksum(&buf[2], 5) ) led_set(LED_RED, 1);
//
//    // Check if we received an ACK or NACK
//    if(buf[3] != UBX_ACK_ACK_ID){ // If not an ACK
//        led_set(LED_RED, 1);
//        // Restore the old baud rate.
//        USART_BRR(USART) = oldBRR;
//    }
//}

/**
 * Verify the checksum for the given data and length.
 */
bool ublox6_verify_checksum(uint8_t* data, uint8_t len)
{
    uint8_t a, b;
    ublox6_ubx_checksum(data, len, &a, &b);
    if( a != *(data + len) || b != *(data + len + 1))
        return false;
    else
        return true;
}

/**
 * Calculate a UBX checksum using 8-bit Fletcher (RFC1145)
 */
void ublox6_ubx_checksum(uint8_t* data, uint8_t len, uint8_t* cka, uint8_t* ckb)
{
    *cka = 0;
    *ckb = 0;
    uint8_t i = 0;
    for( i = 0; i < len; i++ )
    {
        *cka += *data;
        *ckb += *cka;
        data++;
    }
}
