/*
 * xmega01_utils.c
 *
 *  Created on: 1 de nov. de 2016
 *      Author: pablo
 */

#include "spx_tickless.h"

//-----------------------------------------------------------
void initMCU(void)
{
	// Inicializa los pines del micro

	IO_config_MEM_VCC();
	IO_clr_MEM_VCC();

	IO_config_BT_PWR_CTL();
	IO_clr_BT_PWR_CTL();

	IO_config_SENS_12V_CTL();	// Control de fuente de 12V
	IO_clr_SENS_12V_CTL();

	IO_config_AN_3V3_CTL();
	IO_clr_AN_3V3_CTL();

	IO_config_SPIMEM_CS();		// El pin CS de la FLASH SPI es output
	IO_set_SPIMEM_CS();			// Inicialmente HIGH.

//	IO_config_SPIMEM_MOSI();	// MOSI output
	// PORTC.PIN5CTRL = PORT_OPC_TOTEM_gc;
	//PORTC.PIN5CTRL =  PORT_OPC_BUSKEEPER_gc;
	//PORTC.PIN5CTRL =  PORT_OPC_PULLDOWN_gc;
	//PORTC.PIN5CTRL = PORT_OPC_PULLUP_gc;
	//PORTC.PIN5CTRL = PORT_OPC_WIREDOR_gc;
	//PORTC.PIN5CTRL = PORT_OPC_WIREDAND_gc; Consume poco pero no leo
	//PORTC.PIN5CTRL = PORT_OPC_WIREDORPULL_gc;
//	PORTC.PIN5CTRL = PORT_OPC_WIREDANDPULL_gc;

	IO_config_SPIMEM_SCK();		// SCK output

	IO_config_CLRD();
	IO_set_CLRD();

	IO_config_DOP();
	IO_config_DOL();
	IO_config_D1P();
	IO_config_D1L();

	IO_config_LED1();

	IO_config_PWR_SLEEP();
	IO_set_PWR_SLEEP();

	IO_config_OUT_12V_CTL();	 // OUTPUTS 8814 CONTROL
	IO_config_PHB();
	IO_config_ENB();
	IO_config_ENA();
	IO_config_PHA();
	IO_config_SLP();
	IO_config_RES();

	// GPRS
	IO_config_GPRS_SW();
	IO_clr_GPRS_SW();

	IO_config_GPRS_PWR();
	IO_clr_GPRS_PWR();

	IO_config_GPRS_RTS();
	IO_clr_GPRS_RTS();
//	PORTD.PIN6CTRL = PORT_OPC_WIREDANDPULL_gc;

	IO_config_GPRS_CTS();
	IO_config_GPRS_DCD();
	IO_config_GPRS_RI();

	IO_config_GPRS_RX();
	IO_config_GPRS_TX();
	IO_set_GPRS_TX();

	IO_config_SLEEP_CTL();

	IO_config_LED_PA0();
	IO_config_TERMCTL_PIN();
	IO_config_TICK_MONITOR();


}
//-----------------------------------------------------------
