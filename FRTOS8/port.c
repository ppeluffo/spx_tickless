/* This file has been prepared for Doxygen automatic documentation generation.*/
/*
 * Copyright (C) 2012 Yuriy Kulikov
 *      Universitaet Erlangen-Nuernberg
 *      LS Informationstechnik (Kommunikationselektronik)
 *      Support email: Yuriy.Kulikov.87@googlemail.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* Compiler definitions include file. */
#include "avr_compiler.h"
#include "FreeRTOSConfig.h"
/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"

#include "TC_driver.h"
#include "pmic_driver.h"
#include <avr/sleep.h>

#define FOSC_RTC		32768

static volatile bool	countingCompleted;
static void prvSleepExactTime( portTickType xExpectedIdleTime );

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the AVR XMEGA port.
 *----------------------------------------------------------*/

/* Start tasks with interrupts enables. */
#define portFLAGS_INT_ENABLED                    ( ( portSTACK_TYPE ) 0x80 )

/*-----------------------------------------------------------*/

/* We require the address of the pxCurrentTCB variable, but don't want to know
 any details of its type. */
typedef void tskTCB;
extern volatile tskTCB * volatile pxCurrentTCB;

/*-----------------------------------------------------------*/

/* 
 * Macro to save all the general purpose registers, the save the stack pointer
 * into the TCB.  
 * 
 * The first thing we do is save the flags then disable interrupts.  This is to 
 * guard our stack against having a context switch interrupt after we have already 
 * pushed the registers onto the stack - causing the 32 registers to be on the 
 * stack twice. 
 * 
 * r1 is set to zero as the compiler expects it to be thus, however some
 * of the math routines make use of R1. 
 * 
 * The interrupts will have been disabled during the call to portSAVE_CONTEXT()
 * so we need not worry about reading/writing to the stack pointer. 
 */

#define portSAVE_CONTEXT()                                   \
    asm volatile (  "push    r0                     \n\t"    \
                    "in      r0, __SREG__           \n\t"    \
                    "cli                            \n\t"    \
                    "push    r0                     \n\t"    \
                    "push    r1                     \n\t"    \
                    "clr    r1                      \n\t"    \
                    "push    r2                     \n\t"    \
                    "push    r3                     \n\t"    \
                    "push    r4                     \n\t"    \
                    "push    r5                     \n\t"    \
                    "push    r6                     \n\t"    \
                    "push    r7                     \n\t"    \
                    "push    r8                     \n\t"    \
                    "push    r9                     \n\t"    \
                    "push    r10                    \n\t"    \
                    "push    r11                    \n\t"    \
                    "push    r12                    \n\t"    \
                    "push    r13                    \n\t"    \
                    "push    r14                    \n\t"    \
                    "push    r15                    \n\t"    \
                    "push    r16                    \n\t"    \
                    "push    r17                    \n\t"    \
                    "push    r18                    \n\t"    \
                    "push    r19                    \n\t"    \
                    "push    r20                    \n\t"    \
                    "push    r21                    \n\t"    \
                    "push    r22                    \n\t"    \
                    "push    r23                    \n\t"    \
                    "push    r24                    \n\t"    \
                    "push    r25                    \n\t"    \
                    "push    r26                    \n\t"    \
                    "push    r27                    \n\t"    \
                    "push    r28                    \n\t"    \
                    "push    r29                    \n\t"    \
                    "push    r30                    \n\t"    \
                    "push    r31                    \n\t"    \
                    "lds    r26, pxCurrentTCB       \n\t"    \
                    "lds    r27, pxCurrentTCB + 1   \n\t"    \
                    "in        r0, 0x3d             \n\t"    \
                    "st        x+, r0               \n\t"    \
                    "in        r0, 0x3e             \n\t"    \
                    "st        x+, r0               \n\t"    \
                );

/* 
 * Opposite to portSAVE_CONTEXT().  Interrupts will have been disabled during
 * the context save so we can write to the stack pointer. 
 */
#define portRESTORE_CONTEXT()                                \
    asm volatile (  "lds    r26, pxCurrentTCB        \n\t"    \
                    "lds    r27, pxCurrentTCB + 1    \n\t"    \
                    "ld     r28, x+                  \n\t"    \
                    "out    __SP_L__, r28            \n\t"    \
                    "ld     r29, x+                  \n\t"    \
                    "out    __SP_H__, r29            \n\t"    \
                    "pop    r31                      \n\t"    \
                    "pop    r30                      \n\t"    \
                    "pop    r29                      \n\t"    \
                    "pop    r28                      \n\t"    \
                    "pop    r27                      \n\t"    \
                    "pop    r26                      \n\t"    \
                    "pop    r25                      \n\t"    \
                    "pop    r24                      \n\t"    \
                    "pop    r23                      \n\t"    \
                    "pop    r22                      \n\t"    \
                    "pop    r21                      \n\t"    \
                    "pop    r20                      \n\t"    \
                    "pop    r19                      \n\t"    \
                    "pop    r18                      \n\t"    \
                    "pop    r17                      \n\t"    \
                    "pop    r16                      \n\t"    \
                    "pop    r15                      \n\t"    \
                    "pop    r14                      \n\t"    \
                    "pop    r13                      \n\t"    \
                    "pop    r12                      \n\t"    \
                    "pop    r11                      \n\t"    \
                    "pop    r10                      \n\t"    \
                    "pop    r9                       \n\t"    \
                    "pop    r8                       \n\t"    \
                    "pop    r7                       \n\t"    \
                    "pop    r6                       \n\t"    \
                    "pop    r5                       \n\t"    \
                    "pop    r4                       \n\t"    \
                    "pop    r3                       \n\t"    \
                    "pop    r2                       \n\t"    \
                    "pop    r1                       \n\t"    \
                    "pop    r0                       \n\t"    \
                    "out    __SREG__, r0             \n\t"    \
                    "pop    r0                       \n\t"    \
                );

/*-----------------------------------------------------------*/
/*
 * Perform hardware setup to enable ticks from timer 0, compare match A.
 */
static void prvSetupTimerInterrupt(void);
/*-----------------------------------------------------------*/

/* 
 * See header file for description. 
 */portSTACK_TYPE *pxPortInitialiseStack(portSTACK_TYPE *pxTopOfStack,
        pdTASK_CODE pxCode, void *pvParameters) {
    /*The addresses are 16 or 24 bit depending on the xmega memory, so use 32 bit variable but put only a
     * part to stack.*/
    uint16_t usAddress;
    /* Place a few bytes of known values on the bottom of the stack.
     This is just useful for debugging. */
    *pxTopOfStack = 0x11;
    pxTopOfStack--;
    *pxTopOfStack = 0x22;
    pxTopOfStack--;
    *pxTopOfStack = 0x33;
    pxTopOfStack--;
    /* Simulate how the stack would look after a call to vPortYield() generated by
     the compiler. */
    /*lint -e950 -e611 -e923 Lint doesn't like this much - but nothing I can do about it. */
    /* The start of the task code will be popped off the stack last, so place
     it on first. */
    /*    Original code
     * For 16-bit program counter (128K program memory or less)
     usAddress = ( unsigned short ) pxCode;
     *pxTopOfStack = ( portSTACK_TYPE ) ( usAddress & ( unsigned short ) 0x00ff );
     pxTopOfStack--;
     usAddress >>= 8;
     *pxTopOfStack = ( portSTACK_TYPE ) ( usAddress & ( unsigned short ) 0x00ff );
     pxTopOfStack--;*/
    /* end of original code block */

    /* The way it should be done for xmega with probably  more than 128K program memory.
     * Warning is OK here - type incompatibility does not matter - usAddress is only
     * used as temporary storage */
    usAddress = (uint16_t)pxCode;

    *pxTopOfStack = (portSTACK_TYPE ) (usAddress & (uint16_t) 0x00ff);
    pxTopOfStack--;
    usAddress >>= 8;

    *pxTopOfStack = (portSTACK_TYPE ) (usAddress & (uint16_t) 0x00ff);
    pxTopOfStack--;

// Los ATXmega256A3 tienen 265K flash ( 24 bits )
//#if defined(__AVR_3_BYTE_PC__) && __AVR_3_BYTE_PC__
    *pxTopOfStack = (portSTACK_TYPE ) 0;
    pxTopOfStack--;
//#endif

    /* Next simulate the stack as if after a call to portSAVE_CONTEXT().
     portSAVE_CONTEXT places the flags on the stack immediately after r0
     to ensure the interrupts get disabled as soon as possible, and so ensuring
     the stack use is minimal should a context switch interrupt occur. */
    *pxTopOfStack = (portSTACK_TYPE ) 0x00; /* R0 */
    pxTopOfStack--;
    *pxTopOfStack = portFLAGS_INT_ENABLED;
    pxTopOfStack--;

    /* Now the remaining registers.   The compiler expects R1 to be 0. */
    *pxTopOfStack = (portSTACK_TYPE ) 0x00; /* R1 */
    pxTopOfStack--;
    *pxTopOfStack = (portSTACK_TYPE ) 0x02; /* R2 */
    pxTopOfStack--;
    *pxTopOfStack = (portSTACK_TYPE ) 0x03; /* R3 */
    pxTopOfStack--;
    *pxTopOfStack = (portSTACK_TYPE ) 0x04; /* R4 */
    pxTopOfStack--;
    *pxTopOfStack = (portSTACK_TYPE ) 0x05; /* R5 */
    pxTopOfStack--;
    *pxTopOfStack = (portSTACK_TYPE ) 0x06; /* R6 */
    pxTopOfStack--;
    *pxTopOfStack = (portSTACK_TYPE ) 0x07; /* R7 */
    pxTopOfStack--;
    *pxTopOfStack = (portSTACK_TYPE ) 0x08; /* R8 */
    pxTopOfStack--;
    *pxTopOfStack = (portSTACK_TYPE ) 0x09; /* R9 */
    pxTopOfStack--;
    *pxTopOfStack = (portSTACK_TYPE ) 0x10; /* R10 */
    pxTopOfStack--;
    *pxTopOfStack = (portSTACK_TYPE ) 0x11; /* R11 */
    pxTopOfStack--;
    *pxTopOfStack = (portSTACK_TYPE ) 0x12; /* R12 */
    pxTopOfStack--;
    *pxTopOfStack = (portSTACK_TYPE ) 0x13; /* R13 */
    pxTopOfStack--;
    *pxTopOfStack = (portSTACK_TYPE ) 0x14; /* R14 */
    pxTopOfStack--;
    *pxTopOfStack = (portSTACK_TYPE ) 0x15; /* R15 */
    pxTopOfStack--;
    *pxTopOfStack = (portSTACK_TYPE ) 0x16; /* R16 */
    pxTopOfStack--;
    *pxTopOfStack = (portSTACK_TYPE ) 0x17; /* R17 */
    pxTopOfStack--;
    *pxTopOfStack = (portSTACK_TYPE ) 0x18; /* R18 */
    pxTopOfStack--;
    *pxTopOfStack = (portSTACK_TYPE ) 0x19; /* R19 */
    pxTopOfStack--;
    *pxTopOfStack = (portSTACK_TYPE ) 0x20; /* R20 */
    pxTopOfStack--;
    *pxTopOfStack = (portSTACK_TYPE ) 0x21; /* R21 */
    pxTopOfStack--;
    *pxTopOfStack = (portSTACK_TYPE ) 0x22; /* R22 */
    pxTopOfStack--;
    *pxTopOfStack = (portSTACK_TYPE ) 0x23; /* R23 */
    pxTopOfStack--;

    /* Place the parameter on the stack in the expected location. */
    usAddress = (unsigned short) pvParameters;
    *pxTopOfStack = (portSTACK_TYPE ) (usAddress & (unsigned short) 0x00ff);
    pxTopOfStack--;

    usAddress >>= 8;
    *pxTopOfStack = (portSTACK_TYPE ) (usAddress & (unsigned short) 0x00ff);
    pxTopOfStack--;

    *pxTopOfStack = (portSTACK_TYPE ) 0x26; /* R26 X */
    pxTopOfStack--;
    *pxTopOfStack = (portSTACK_TYPE ) 0x27; /* R27 */
    pxTopOfStack--;
    *pxTopOfStack = (portSTACK_TYPE ) 0x28; /* R28 Y */
    pxTopOfStack--;
    *pxTopOfStack = (portSTACK_TYPE ) 0x29; /* R29 */
    pxTopOfStack--;
    *pxTopOfStack = (portSTACK_TYPE ) 0x30; /* R30 Z */
    pxTopOfStack--;
    *pxTopOfStack = (portSTACK_TYPE ) 0x31; /* R31 */
    pxTopOfStack--;

    return pxTopOfStack;
}
/*-----------------------------------------------------------*/
BaseType_t xPortStartScheduler(void) {

    /* Setup the hardware to generate the tick. */
    prvSetupTimerInterrupt();

    /* Restore the context of the first task that is going to run. */
    portRESTORE_CONTEXT();

    /* Simulate a function call end as generated by the compiler.  We will now
     jump to the start of the task the context of which we have just restored. */
    asm volatile ( "ret" );

    /* Should not get here. */
    return pdTRUE;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler(void) {
    /* It is unlikely that the AVR port will get stopped.  If required simply
     disable the tick interrupt here. */
}
/*-----------------------------------------------------------*/

/*
 * Manual context switch.  The first thing we do is save the registers so we
 * can use a naked attribute.
 */
void vPortYield(void) __attribute__ ( ( naked ) );
void vPortYield(void) {
    portSAVE_CONTEXT();
    vTaskSwitchContext();
    portRESTORE_CONTEXT();

    asm volatile ( "ret" );
}
/*-----------------------------------------------------------*/

/*
 * Context switch function used by the tick.  This must be identical to
 * vPortYield() from the call to vTaskSwitchContext() onwards.  The only
 * difference from vPortYield() is the tick count is incremented as the
 * call comes from the tick ISR.
 */
void vPortYieldFromTick( void ) __attribute__ ( ( naked ) );
void vPortYieldFromTick( void )
{
	portSAVE_CONTEXT();
	if( xTaskIncrementTick() != pdFALSE )
	{
		vTaskSwitchContext();
	}
	portRESTORE_CONTEXT();

	asm volatile ( "ret" );
}

/*-----------------------------------------------------------*/
/*
 * Setup timer 0 compare match A to generate a tick interrupt.
 * La duracion de los ticks la indico en configTICK_RATE_HZ.
 *
 */

static void prvSetupTimerInterrupt(void) {
    //Use TCC0 as a tick counter. If this is to be changed, change ISR as well
    TC0_t * tickTimer = &TCC0;
    //select the clock source and pre-scale by 64
    TC0_ConfigClockSource(tickTimer, TC_CLKSEL_DIV64_gc);
    //set period of counter
    tickTimer->PER = configCPU_CLOCK_HZ / configTICK_RATE_HZ / 64 - 1;

    //enable interrupt and set low level
    TC0_SetOverflowIntLevel(tickTimer, TC_OVFINTLVL_LO_gc);
    //enable low level interrupts
    PMIC_EnableLowLevel();
}
/*-----------------------------------------------------------*/

#if configUSE_PREEMPTION == 1

/*
 * Tick ISR for preemptive scheduler.  We can use a naked attribute as
 * the context is saved at the start of vPortYieldFromTick().  The tick
 * count is incremented after the context is saved.
 */
ISR( TCC0_OVF_vect, ISR_NAKED);
ISR (TCC0_OVF_vect, ISR_NAKED) {
//ISR (RTC_OVF_vect, ISR_NAKED);
//ISR (RTC_OVF_vect, ISR_NAKED) {
    /*
     * Context switch function used by the tick.  This must be identical to
     * vPortYield() from the call to vTaskSwitchContext() onwards.  The only
     * difference from vPortYield() is the tick count is incremented as the
     * call comes from the tick ISR.
     */

	PORTA.OUTTGL = 0x04;	// Hago un toggle del pin A3 para medir la frecuencia del tick con osciloscopio.
	vPortYieldFromTick();
	asm volatile ( "reti" );
}

#else

/*
 * Tick ISR for the cooperative scheduler.  All this does is increment the
 * tick count.  We don't need to switch context, this can only be done by
 * manual calls to taskYIELD();
 */
ISR( TCC0_OVF_vect, ISR_NAKED);
ISR (TCC0_OVF_vect, ISR_NAKED) {
//ISR (RTC_OVF_vect, ISR_NAKED);
//ISR (RTC_OVF_vect, ISR_NAKED) {

}
    vTaskIncrementTick();
}
#endif
//-------------------------------------------------------------------------------------
//  Tickless sleeping mode.

#if configUSE_TICKLESS_IDLE == 2
//-------------------------------------------------------------------------------------
static void prvSleep()
{
	/* Re-enabling interrupts to awake and go to sleep*/
	sleep_enable();
	sei();
	//sleep_cpu();
	sleep_mode();

	/* Sleeps here until awaken, then continues */
	sleep_disable();
	cli();
}
//-------------------------------------------------------------------------------------
// Define the function that is called by portSUPPRESS_TICKS_AND_SLEEP().
void vApplicationSleep( TickType_t xExpectedIdleTime )
{
	// Utilizo el RTC16 bits para contar el tiempo de sleep.

eSleepModeStatus eSleepStatus;

	// Paso1: Leer el 'tiempo actual'
	// Como el contador del RTC16 lo inicializo en 0, este es el tiempo inicial.
	// ulLowPowerTimeBeforeSleep = ulGetExternalTime();

	// Paso2: Detener la generacion de ticks.
	portENTER_CRITICAL();
    //prvStopTickInterruptTimer();

    // Enter a critical section that will not effect interrupts bringing the MCU
    // out of sleep mode.
    //disable_interrupts();

    // Paso3: Ensure it is still ok to enter the sleep mode.
    eSleepStatus = eTaskConfirmSleepModeStatus();

    if( eSleepStatus == eNoTasksWaitingTimeout ) {
        /* It is not necessary to configure an interrupt to bring the
        microcontroller out of its low power state at a fixed time in the
        future. */
    	set_sleep_mode(SLEEP_MODE_PWR_SAVE);
        prvSleep();
        portEXIT_CRITICAL();

    } else  if( eSleepStatus == eAbortSleep ) {
        /* A task has been moved out of the Blocked state since this macro was
        executed, or a context swith is being held pending.  Do not enter a
        sleep state.  Restart the tick and exit the critical section. */
    	portEXIT_CRITICAL();
        //prvStartTickInterruptTimer();
        //enable_interrupts();
    } else {
    	/* Configure an interrupt to bring the microcontroller out of its low
    	   power state at the time the kernel next needs to execute.  The
    	   interrupt must be generated from a source that remains operational
    	   when the microcontroller is in a low power state. */

    	portEXIT_CRITICAL();
    	prvSleepExactTime(xExpectedIdleTime);

    }

}
//-------------------------------------------------------------------------------------
static void prvSleepExactTime( portTickType xExpectedIdleTime )
{

portTickType tick_to_sleep, completedTicks;
uint32_t rtc_counter;

	// En esta funcion debo:
	// 1- Configurar el RTC16 para interrumpir luego de xExpectedIdleTime
	// 2- Sleep
	// 3- Al despertar, ajustar el tick a los ticks dormidos reales.

	// El RTC lo debo programar para dormir TICKS ( 10ms ) o sea que este va a ser el error
	// al despertar ( +/- 1 tick).
	// Por otro lado, el kernel determina cuanto puede dormir y este es el valor que pasa en xExpectedIdleTime.
	// Nosotros no queremos dormir mas de 1s por lo tanto calculo el valor del RTC y si supera 1s, lo limito.

	// Paso 1:
	// En MAIN ya configure el RTC con el osc externo a 32Khz sin prescaler.
	// El periodo son 0.03 ms por lo que para contar 10ms debo contar hasta 327 y para
	// contar 1s debo contar hasta 32768.
	// El contador es de 16bits o sea cuenta hasta 65536
	// Lo configuro para contar hasta xExpectedIdleTime.
	// Nunca cuento mas de 1500 ms


	tick_to_sleep = xExpectedIdleTime;
	if ( tick_to_sleep > 1500 ) {
		tick_to_sleep = 1500;
	}
	rtc_counter = tick_to_sleep * FOSC_RTC / configTICK_RATE_HZ - 1;

	RTC.PER = (uint16_t) (rtc_counter);
	RTC.CNT = 0;
	RTC.COMP = 0;
	RTC.CTRL = ( RTC.CTRL & ~RTC_PRESCALER_gm ) | RTC_PRESCALER_DIV1_gc;
	// Enable overflow interrupt.
	RTC.INTCTRL = 0x01;
	// Espero que se sincronizen todos los registros
	do {} while ( ( RTC.STATUS & RTC_SYNCBUSY_bm ) );

	// Paso2: habilito las interrupciones y sleep.
	portENTER_CRITICAL();
	countingCompleted = false;
	prvSleep();

	// Paso3: Al despertar actualizo los ticks.
	// Con countingComplete determino si complete el periodo de sleep o alguna
	// interrupcion me desperto antes.
	// Detengo el timer.
	RTC.CTRL = 0x00;
	portEXIT_CRITICAL();

	if (countingCompleted) {
		// Durmio todo el periodo.
		completedTicks = xExpectedIdleTime;
	} else {
		// Algo lo desperto. Durmio de menos.
		// Como la ISR no borro el contador, vemos cuanto durmio y lo convierto a ticks.
		// Agrego 1 para redondear.
		completedTicks = ( RTC.CNT * configTICK_RATE_HZ / FOSC_RTC ) +1;
	}

	vTaskStepTick(completedTicks);

}
//-------------------------------------------------------------------------------------
ISR (RTC_OVF_vect) {
	countingCompleted = true;
}
//-------------------------------------------------------------------------------------
#endif /* configUSE_TICKLESS_IDLE == 1 */




