/*
 * spx.h
 *
 *  Created on: 20 de oct. de 2016
 *      Author: pablo
 */

#ifndef SRC_SPX_TICKLESS_H_
#define SRC_SPX_TICKLESS_H_

//------------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------------
#include <avr/io.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <compat/deprecated.h>
#include <avr/pgmspace.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <avr/sleep.h>
#include <ctype.h>
#include <l_rtc.h>
#include "avr_compiler.h"
#include "clksys_driver.h"

#include "TC_driver.h"
#include "pmic_driver.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "list.h"
#include "croutine.h"
#include "semphr.h"
#include "timers.h"
#include "limits.h"

#include "FRTOS-IO.h"
#include "spx_libs/l_iopines.h"
#include "spx_libs/l_iopines.h"

//------------------------------------------------------------------------------------
// DEFINES
//------------------------------------------------------------------------------------
#define SP6K_REV "0.0.10"
#define SP6K_DATE "@ 20171110"

#define SP6K_MODELO "spx_tickless HW:xmega256A3 R1.0"
#define SP6K_VERSION "FW:FRTOS8"

//#define SYSMAINCLK_2MHZ
#define SYSMAINCLK_8MHZ
//#define SYSMAINCLK_32MHZ

#define CHAR128	128
#define CHAR256	256

#define tkCMD_STACK_SIZE				512
#define tkCTL_STACK_SIZE				512
#define tkDIGITAL_STACK_SIZE			512
#define tkGPRS_RX_STACK_SIZE			512

#define tkCMD_TASK_PRIORITY	 			( tskIDLE_PRIORITY + 1 )
#define tkCTL_TASK_PRIORITY	 			( tskIDLE_PRIORITY + 1 )
#define tkDIGITAL_TASK_PRIORITY	 		( tskIDLE_PRIORITY + 1 )
#define tkGPRS_RX_TASK_PRIORITY	 		( tskIDLE_PRIORITY + 1 )

//------------------------------------------------------------------------------------
// VARIABLES
//------------------------------------------------------------------------------------
char d_printfBuff[CHAR128];
char debug_printfBuff[256];

TaskHandle_t xHandle_tkCmd, xHandle_tkCtl;

bool sleepFlag;

//------------------------------------------------------------------------------------
// PROTOTIPOS
//------------------------------------------------------------------------------------
void tkCmd(void * pvParameters);
void tkCtl(void * pvParameters);

void configureTimer(void);
void initMCU(void);
int8_t pvConfigBTbaud(char *s_baud);

#define WDT_IsSyncBusy() ( WDT.STATUS & WDT_SYNCBUSY_bm )
void WDT_EnableAndSetTimeout( WDT_PER_t period );
void WDT_DisableWindowMode( void );
#define WDT_Reset()	asm("wdr") //( watchdog_reset( ) )

//------------------------------------------------------------------------------------

#endif /* SRC_SPX_TICKLESS_H_ */
