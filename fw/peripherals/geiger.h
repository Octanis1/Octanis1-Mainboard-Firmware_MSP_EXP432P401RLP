#ifndef __GEIGER_H
#define __GEIGER_H

#define SAMPLING_TIME 60
#define TOO_SOON -1
#define ON 1
#define OFF 0

/* Enable/disable interrupt on geiger pin
 * Also turn ON/OFF the geiger counter
 * (consume approx 50 mA when on) */
void geiger_init(uint8_t on_off);

/* Keep track of the count this minute and last minute */
void geiger_count();

/* Return the number of particle detected last minute
 * return TOO_SOON if called before he could do the 1st
 * SAMPLING_TIME count */
uint16_t get_last_minute_count();

#endif
