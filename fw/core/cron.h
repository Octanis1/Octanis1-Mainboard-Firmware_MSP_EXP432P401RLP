/*
 * cron.h
 *
 *  Created on: 13 Aug 2015
 *      Author: Sam
 */

#ifndef FW_CORE_CRON_H_
#define FW_CORE_CRON_H_

//these functions are called by SYS/BIOS staticly created Clock objects and
// can be used for various purposes

Void cron_quick_clock(UArg arg);

Void cron_hourly_clock(UArg arg);

#endif /* FW_CORE_CRON_H_ */
