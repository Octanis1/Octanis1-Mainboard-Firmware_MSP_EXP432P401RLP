#ifndef LOG_ENTRIES_H
#define LOG_ENTRIES_H

#include "../peripherals/navigation.h"

void log_write_gps(void);
void log_write_imu(void);
void log_write_weather(void);
void log_write_navigation_status(void);
void log_write_mavlink_item_list(bool overwrite, uint8_t *pos_counter);
void log_serialize_mavlink_item(cmp_ctx_t *ctx, mavlink_mission_item_t item);

bool log_read_mavlink_item_list(mission_item_list_t * item_list, uint32_t * time, char *name, uint8_t *pos_counter);
bool log_read_mavlink_item(mavlink_mission_item_t * item);




#endif /* LOG_ENTRIES_H */
