/* Mock function for logging testing*/

#include "log_mock_fun.h"

float gps_get_lat()
{
    return 12.4;
}

float gps_get_lon()
{
    return 34.5;
}

uint8_t gps_get_fix_quality()
{
    return 4;
}

unsigned int weather_get_int_press()
{
    return 22;
}

int weather_get_int_temp()
{
    return -4;
}

uint8_t imu_get_calib_status()
{
    return 7;
}

uint16_t imu_get_heading()
{
    return 32;
}

uint16_t imu_get_roll()
{
    return 134;
}

uint16_t imu_get_pitch()
{
    return 104;
}

unsigned int weather_get_int_humid()
{
    return 88;
}

unsigned int weather_get_ext_press()
{
    return 99;
}

int weather_get_ex_temp()
{
    return -42;
}

unsigned int weahter_get_ext_humid()
{
    return 155;
}

uint8_t Mailbox_pend(Queue_Handle mailbox, void *l, uint8_t wait)
{
    return 1;
}

uint8_t Mailbox_post(Queue_Handle mailbox, void *l, uint8_t wait)
{
    return 2;
}

uint32_t Timestamp_get32()
{
    return 45678;
}
