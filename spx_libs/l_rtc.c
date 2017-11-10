/*
 * l_rtc32.c
 *
 *  Created on: 23 de oct. de 2017
 *      Author: pablo
 */

#include <l_rtc.h>

//----------------------------------------------------------------------------------------
void RTC_Initialize( uint16_t period,uint16_t count,uint16_t compareValue,RTC_PRESCALER_t prescaler )
{
	RTC.PER = period - 1;
	RTC.CNT = count;
	RTC.COMP = compareValue;
	RTC.CTRL = ( RTC.CTRL & ~RTC_PRESCALER_gm ) | prescaler;
}
//----------------------------------------------------------------------------------------
void RTC_SetOverflowIntLevel( RTC_OVFINTLVL_t intLevel )
{
	RTC.INTCTRL = ( RTC.INTCTRL & ~RTC_OVFINTLVL_gm ) | intLevel;
}
//----------------------------------------------------------------------------------------
void configure_RTC(void)
{
	// Prendo el oscilador externo
	//OSC.CTRL |= OSC_XOSCEN_bm;
	// Espero que se estabilize
	//do {} while ( (OSC.STATUS & OSC_XOSCRDY_bm ) == 0);

	// Prendo el 32Khz interno calibrado ya que no tengo los condensadores
	// en la placa por lo que el TOSC no funciona.
//	OSC.CTRL |= OSC_RC32KEN_bm;
//	do {} while ( ( OSC.STATUS & OSC_RC32KRDY_bm ) == 0);

	// Set internal external OSC as clock source for RTC. */
	//CLK.RTCCTRL = CLK_RTCSRC_ULP_gc | CLK_RTCEN_bm;

	// Pongo como fuente el xtal externo de 32768 contando en 1khz.
	CLK.RTCCTRL = CLK_RTCSRC_TOSC32_gc | CLK_RTCEN_bm;
	do {} while ( RTC_Busy() );

	/* Configure RTC period to 1 second. */
	RTC_Initialize( RTC_CYCLES_10ms, 0, 0, RTC_PRESCALER_DIV1_gc );

	/* Enable overflow interrupt. */
	RTC.INTCTRL = 0x01;
	//RTC_SetOverflowIntLevel(RTC_COMPINTLVL_LO_gc);

}
//----------------------------------------------------------------------------------------
//ISR(RTC_OVF_vect)
//{
	// Utilizo el pin A0 ( AN3 ) para generar la onda de la interrupcion
//	IO_toggle_AN_3V3_CTL();
//}
//----------------------------------------------------------------------------------------
