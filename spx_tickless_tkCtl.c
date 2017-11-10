/*
 * spx_tkCtl.c
 *
 *  Created on: 4 de oct. de 2017
 *      Author: pablo
 */

#include "spx_tickless.h"

void tkCtl(void * pvParameters)
{

( void ) pvParameters;

	for( ;; )
	{

		// Prendo y apago el led PA0 cada 3s.

		IO_set_LED_PA0();
		vTaskDelay( ( TickType_t)( 10 / portTICK_RATE_MS ) );
		IO_clr_LED_PA0();
		vTaskDelay( ( TickType_t)( 2990 / portTICK_RATE_MS ) );

	}
}
//------------------------------------------------------------------------------------

