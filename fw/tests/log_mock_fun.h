#include <stdint.h>

typedef struct Queue_Handle {
    uint8_t smh;
}Queue_Handle;

uint32_t Timestamp_get32();

float gps_get_lat();
float gps_get_lon();
uint8_t gps_get_fix_quality();

uint8_t imu_get_calib_status();
uint16_t imu_get_heading();
uint16_t imu_get_roll();
uint16_t imu_get_pitch();

unsigned int weather_get_int_press();
int weather_get_int_temp(); 
unsigned int weather_get_int_humid();
unsigned int weather_get_ext_press();
int weather_get_ex_temp();
unsigned int weahter_get_ext_humid();

uint8_t Mailbox_pend(Queue_Handle mailbox, void *l, uint8_t wait);
uint8_t Mailbox_post(Queue_Handle mailbox, void *l, uint8_t wait);

