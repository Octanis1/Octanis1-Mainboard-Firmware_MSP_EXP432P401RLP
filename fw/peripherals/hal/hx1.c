///*
// *  File: radio.c
// *  Description: Model for vhf radio HX1 to do FSK
// *  Author:
// */
//#include "../../../Board.h"
//
//#include "hx1.h"
//
//#define MILLIS_PER_BIT 22//50 baud
//#define MILLIS_PER_STOPBIT 30 //1.5
//
////space at 830Hz (0), mark at 1000hz (1)
//#define MARK_PERIOD 1000
//#define MARK_DUTY 500
//#define SPACE_PERIOD  1204
//#define SPACE_DUTY  602
//
//void hx1_off(){
//	GPIO_write(Board_HX1_EN, 0);
//}
//
//void hx1_on(){
//	GPIO_write(Board_HX1_EN, 1);
//}
//
//int pwm_bit(int wait_after_bit, int period, int duty){
//	PWM_Handle pwm_handle;
//	PWM_Params pwm_params;
//
//
//	PWM_Params_init(&pwm_params);
//	pwm_params.period = period;             // Period in microseconds
//	pwm_params.dutyMode = PWM_DUTY_TIME; // duty is an integer scaled to the period,
//										  // 0 = 0% and 65535 = 100%
//	pwm_handle = PWM_open(Board_PWM0, &pwm_params);
//	if (pwm_handle == NULL) {
//		return 0;
//	}
//
//	PWM_setDuty(pwm_handle, duty);
//
//	//wait for the bit to "go through"
//	Task_sleep(wait_after_bit);
//	PWM_close(pwm_handle);
//	return 1;
//}
//
//int mark(){ //1
//	return pwm_bit(MILLIS_PER_BIT, 470, 235); //mark with 2125Hz and 170hz shift
//}
//
//int space(){ //0
//	return pwm_bit(MILLIS_PER_BIT, 435, 217); //mark+170
//}
//
//
//
//
//void hx1_txbyte(unsigned char c){
//	//start bit
//
//	int i;
//	int b[8];
//
//	for (i=0;i<8;i++){
//		if (c & 1){
//			//m
//			b[i]=1;
//		}else{
//			//s
//			b[i]=0;
//		}
//		c = c >> 1;
//	}
//
//	//hack
//
//	space();
//
//	for (i=7;i>=0;i--){
//		if(b[i]){
//			mark();
//		}else{
//			space();
//		}
//	}
//
//
//	//stop bit
//	mark();
//	mark();
//}
//
//void hx1_txstring(char * string){
//
//	int i;
//	for (i=0; i<sizeof(string); i++)
//	{
//		hx1_txbyte(string[i]);
//
//	}
//}
//
////starts the timer
//int hx1_begin(){
//
//
//	hx1_txstring((char *)"test");
//
//	//hx1_txbyte('a');
//
//	//send asci 8bit 'a'
//		//01100001
//
//		//or 5bit baudot A
//		// 00011
//
///*
//		space();
//
//		space();
//		mark();
//		mark();
//		space();
//		space();
//		space();
//		space();
//		mark();
//
//
//		mark();
//		mark();
//
//*/
//
///*//s
//	space();
//
//
//	space();
//	mark();
//	mark();
//	mark();
//	space();
//	space();
//	mark();
//	mark();
//
//	mark();
//	mark();
//*/
////M
//
///*
//	space();
//
//	space();
//	mark();
//	space();
//	space();
//	mark();
//	mark();
//	space();
//	mark();
//
//	mark();
//	mark();
//*/
//	//hx1_txbyte('M');
//
//	return 1;
//}
//
//
////stops the timer
//void hx1_end(){
//	hx1_off();
//}
//
