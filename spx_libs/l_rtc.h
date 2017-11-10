/*
 * l_rtc32.h
 *
 *  Created on: 23 de oct. de 2017
 *      Author: pablo
 */

#ifndef SRC_SPX_LIBS_L_RTC_H_
#define SRC_SPX_LIBS_L_RTC_H_

#include "FRTOS-IO.h"

//#define RTC_CYCLES_100ms 100
#define RTC_CYCLES_10ms 0x140

#define RTC_Busy()               ( RTC.STATUS & RTC_SYNCBUSY_bm )

void configure_RTC(void);
void RTC_Initialize( uint16_t period,uint16_t count,uint16_t compareValue,RTC_PRESCALER_t prescaler );
void RTC_SetOverflowIntLevel( RTC_OVFINTLVL_t intLevel );

#endif /* SRC_SPX_LIBS_L_RTC_H_ */
