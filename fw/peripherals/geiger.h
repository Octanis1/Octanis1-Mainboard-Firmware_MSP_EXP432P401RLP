#ifndef __GEIGER_H
#define __GEIGER_H

/* Enable interrupt on geiger pin */
void geiger_init();

/* Keep track of the count this minute and last minute */
void geiger_count();

/* Return the number of particle detected last minute */
uint16_t get_last_minute_count();

#endif
