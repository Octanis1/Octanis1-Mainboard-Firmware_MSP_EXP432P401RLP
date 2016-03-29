/*
 *  File: vc0706.h
 *  Description: UART Camera driver
 *  Author: Sam Sulaimanov
 */


#ifndef vc0706_H_
#define vc0706_H_

int vc0706_begin(); //returns 1 if modem responds with OK
void vc0706_end();
void vc0706_gprs_upload_jpeg();

#endif
