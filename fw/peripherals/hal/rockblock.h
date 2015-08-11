/*
 *  File: rockblock.h
 *  Description: Model for Iridium Satellite Modem "Rockblock"
 *  Author: Sam
 */

int rockblock_open();

int rockblock_begin();

int rockblock_get_sleep_status();
int rockblock_get_net_availability();


void rockblock_close();

