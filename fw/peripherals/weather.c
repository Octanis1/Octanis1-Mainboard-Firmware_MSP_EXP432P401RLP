/*
 *  File: weather.c
 *  Description: High level model for accessing calibrated temperature, pressure and moisture
 *  Author:
 */
#include "../../Board.h"

#include "weather.h"
#include "hal/AS3935.h"
#include "hal/bme280i2c.h"
#include "hal/bmp180i2c.h"
#include "hal/SHT2x.h"
#include "hal/windsensor.h"
#include "hal/mcp3425.h"
#include "hal/i2c_helper.h"
#include "../lib/printf.h"
#include "geiger.h"
#include "driverlib.h"
#include <string.h>

//Logging includes:
#include "../core/log.h"
#include "../lib/cmp/cmp.h"
#include "../lib/cmp_mem_access/cmp_mem_access.h"
#include "../hardware_test/mailbox_test.h"
#include "flash.h"
#include "hal/spi_helper.h"

typedef struct Types_FreqHz {
    Bits32 hi;
    // most significant 32-bits of frequency
    Bits32 lo;
    // least significant 32-bits of frequency
} Types_FreqHz;

static struct _weather_data {
	int int_temp; //in 0.01 degree Centigrade
	unsigned int int_press; //in Pa
	unsigned int int_humid; //value of 42313 represents 42313 / 1024 = 41.321 %rH
	int ext_temp_bmp180; //in 0.1 degree Centigrade
	float ext_temp_sht21;
	int ext_temp; // average of both values in 0.01 degree Centigrade
	unsigned int ext_press; //pressure in steps of 1.0 Pa
	float ext_humid;
} weather_data;

//in 0.01 degree Centigrade
int weather_get_int_temp(){
	return weather_data.int_temp;
}

//in Pa
unsigned int weather_get_int_press(){
	return weather_data.int_press;
}

//value of 42313 represents 42313 / 1024 = 41.321 %rH
unsigned int weather_get_int_humid(){
	return weather_data.int_humid;
}

//in 0.01 degree Centigrade
int weather_get_ext_temp(){
	return weather_data.ext_temp;
}

//in Pa
unsigned int weather_get_ext_press(){
	return weather_data.ext_press;
}

//value of 42313 represents 42313 / 1024 = 41.321 %rH
unsigned int weather_get_ext_humid(){
	return (int)(1024*weather_data.ext_humid);
}

/* function calculating averages etc. of measured sensor data */
void weather_aggregate_data()
{
	// average SHT21 and BMP180 external temperature data and scale the result to 0.01 degree Celsius
	weather_data.ext_temp = (int)((10*weather_data.ext_temp_sht21+(float)weather_data.ext_temp_bmp180)*5);
}

uint8_t weather_check_external_connected()
{
	uint8_t j, is_connected = 0;
	P1->OUT |= BIT5; //pre-condition the pin to HIGH
	P1->DIR &= ~BIT5; //make UV sensor enable pin an input
	for(j=0; j++; j<20); //let pin fall to LOW if external board is connected.
	is_connected = (((P1->IN) & BIT5) == 0); //if the pin is pulled down, the external weather monitor is connected
	P1->DIR |= BIT5;
	P1->OUT &= ~BIT5; //pre-condition the pin to HIGH
	return is_connected;
}


void log_weather(struct _weather_data *d)
{
/*    struct logger *l = logger_get("weather");
    cmp_write_array(l->ctx, 8);
    cmp_write_integer(l->ctx, d->int_temp);
    cmp_write_uinteger(l->ctx, d->int_press);
    // ...
    logger_finish(l);
    */
}

void weather_task(){
	static uint8_t external_board_connected = 0;
	external_board_connected = weather_check_external_connected();

	i2c_helper_init_handle();
	windsensor_init();
	geiger_turn_on_off(ON);

	// Initialize external sensors
	if(external_board_connected)
	{
		bmp180_init(); //TODO: change to BMP280
		//	mcp_init();
		lightning_reset();
		lightning_calibrate();
	}

	// Initialize on-board sensors
	bme280_init();



	/************* flash test START ****************/
	spi_helper_init_handle();

	static uint8_t buf[250];
	flash_id_read(buf);
	const uint8_t flash_id[] = {0x01,0x20,0x18}; // S25FL127S ID
	if (memcmp(buf, flash_id, sizeof(flash_id)) == 0) {
		// flash answers with correct ID
		cli_printf("Flash ID OK\n");
		uint32_t addr = 0x00007ff0 ;
		const char write_buf[] = "hello world! hello world! hello world! hello world! hello world! hello world! hello world!hello world!hello world!hello world!";
		size_t write_len = strlen(write_buf) + 1;
		int ret;

		flash_write_enable();
		ret = flash_block_erase(addr);
		if (ret != 0) {
					cli_printf("Flash erase failed\n");
				}
		ret = flash_write(addr, write_buf, write_len);
		if (ret != 0) {
			cli_printf("Flash write failed\n");
		}
		flash_write_disable();



		ret = flash_read(addr, buf, sizeof(buf));
		if (memcmp(buf, write_buf, write_len) == 0) {
			cli_printf("Flash write OK\n");
			cli_printf("read: %s\n", buf);
		} else {
			cli_printf("Flash read != write, FAIL\n");
		}
	} else {
		cli_printf("Flash ID read FAIL\n");
	}

	/************* flash test END ****************/


	while(1){
		Task_sleep(3000);
		if(external_board_connected)
		{
			bmp180_data_readout(&(weather_data.ext_temp_bmp180),&(weather_data.ext_press));
			//		float uv = mcp_get_data();

			weather_data.ext_temp_sht21 = sht2x_get_temp(); //TODO: fix the fact that program stops here if sensor is not connected.
			weather_data.ext_humid = sht2x_get_humidity();

		}

		bme280_data_readout(&(weather_data.int_temp),&(weather_data.int_press),&(weather_data.int_humid));
		//note: bme280 can give pressure, humidity and temperature

		// Logging:

		uint32_t time = Seconds_get();
//		Types_FreqHz freq1;
//		Timestamp_getFreq(&freq1);




		//	windsensor_getvalue();

		weather_aggregate_data();
		cli_printf("W ok. T= %u, He=%u \n", weather_data.int_temp, weather_get_ext_humid());


        log_weather(&weather_data);
	}
}
