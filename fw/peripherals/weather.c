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
#include "../core/log_entries.h"
#include "../lib/cmp/cmp.h"
#include "../lib/cmp_mem_access/cmp_mem_access.h"
#include "../hardware_test/mailbox_test.h"
#include "flash.h"
#include "hal/spi_helper.h"

// NOTE: set to 0 if it should not be logged
#define LOG_GPS_TIMESTEP     5
#define LOG_IMU_TIMESTEP     1
#define LOG_WEATHER_TIMESTEP 5
#define LOG_BACKUP_TIMESTEP  60

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

void weather_call_bme_from_other_task()
{
	bme280_data_readout(&(weather_data.int_temp),&(weather_data.int_press),&(weather_data.int_humid));
}


void weather_task(){
	cli_init();

	Task_sleep(500);

	static uint8_t external_board_connected = 0;
	external_board_connected = weather_check_external_connected();

	i2c_helper_init_handle();
	windsensor_init();
//	mcp_init();
	geiger_turn_on_off(GEIGER_ON);

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


#ifdef FLASH_ENABLED
	/************* flash test START ****************/
	spi_helper_init_handle();

    // force enable logging
    bool logging_enabled = true;


	static uint8_t buf[250];
	flash_id_read(buf);
	const uint8_t flash_id[] = {0x01,0x20,0x18}; // S25FL127S ID
	if (memcmp(buf, flash_id, sizeof(flash_id)) == 0) {
		// flash answers with correct ID
		serial_printf(cli_stdout, "Flash ID OK\n");
	} else {
		serial_printf(cli_stdout, "Flash ID ERROR\n");
	}

    if (logging_enabled) {
        if (!log_init()) {
            serial_printf(cli_stdout, "log_init failed\n");
            log_reset();
        }
    }
    //serial_printf(cli_stdout, "log position 0x%x\n", log_write_pos());
	/************* flash test END ****************/
#endif


    while(1){

#ifdef FLASH_ENABLED

        log_counter++;


        if (logging_enabled) {
            if (LOG_GPS_TIMESTEP > 0 && log_counter % LOG_GPS_TIMESTEP == 0) {
                log_write_gps();
                // serial_printf(cli_stdout, "log gps, %x\n", log_write_pos());
            }
            if (LOG_IMU_TIMESTEP > 0 && log_counter % LOG_IMU_TIMESTEP == 0) {
                log_write_imu();
                // serial_printf(cli_stdout, "log imu, %x\n", log_write_pos());
            }
            if (LOG_WEATHER_TIMESTEP > 0 && log_counter % LOG_WEATHER_TIMESTEP == 0) {
                log_write_weather();
                // serial_printf(cli_stdout, "log weather, %x\n", log_write_pos());
            }
            if (LOG_BACKUP_TIMESTEP > 0 && log_counter % LOG_BACKUP_TIMESTEP == 0) {
                log_position_backup();
                // serial_printf(cli_stdout, "log backup, %x\n", log_write_pos());
            }
        }

#endif
		if(external_board_connected)
		{
			bmp180_data_readout(&(weather_data.ext_temp_bmp180),&(weather_data.ext_press));
			float uv = mcp_get_data();
			int uvint = (int)(1000.0*uv);

			weather_data.ext_temp_sht21 = sht2x_get_temp(); //TODO: fix the fact that program stops here if sensor is not connected.
			weather_data.ext_humid = sht2x_get_humidity();

		}

//		bme280_data_readout(&(weather_data.int_temp),&(weather_data.int_press),&(weather_data.int_humid));

		//	windsensor_getvalue();

		weather_aggregate_data();

//		serial_printf(cli_stdout, "W ok. T= %u, He=%u \n", weather_data.int_temp, weather_get_ext_humid());

//		serial_printf(cli_stdout, "W ok");
		Task_sleep(10);


#ifdef FLASH_ENABLED
        log_weather(&weather_data);
#endif
	}

}
