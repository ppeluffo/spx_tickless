/*
 * l_iopines.c
 *
 *  Created on: 7 de jul. de 2017
 *      Author: pablo
 */

#include "../spx_libs/l_iopines.h"

//------------------------------------------------------------------------------------
// ENTRADAS DIGITALES DE CONTEO DE PULSOS Y NIVELES
//------------------------------------------------------------------------------------
/*
void IO_config_DOP(void)
{
	// D0P: Entrada digital
	PORT_SetPinAsInput( &D0P_PORT, D0P_BITPOS);
}
//------------------------------------------------------------------------------------
void IO_config_DOL(void)
{
	// D0L: Entrada digital
	PORT_SetPinAsInput( &D0L_PORT, D0L_BITPOS);
}
//------------------------------------------------------------------------------------
void IO_config_D1P(void)
{
	// D1P: Entrada digital
	PORT_SetPinAsInput( &D1P_PORT, D1P_BITPOS);
}
//------------------------------------------------------------------------------------
void IO_config_D1L(void)
{
	// D1L: Entrada digital
	PORT_SetPinAsInput( &D1L_PORT, D1L_BITPOS);
}
//------------------------------------------------------------------------------------
void IO_config_CLRD(void)
{
	// CLRD: Salida digital que borra el latch
	PORT_SetPinAsOutput( &CLR_D_PORT, CLR_D_BITPOS);
	// En reposo debe estar en 1
	//PORT_SetOutputBit( &CLR_D_PORT, CLR_D_BITPOS);
}
//------------------------------------------------------------------------------------
void IO_set_CLRD(void)
{
	// Pongo en 1
	PORT_SetOutputBit( &CLR_D_PORT, CLR_D_BITPOS);
}
//------------------------------------------------------------------------------------
void IO_clr_CLRD(void)
{
	// Pongo en 0
	PORT_ClearOutputBit( &CLR_D_PORT, CLR_D_BITPOS);
}
*/
//------------------------------------------------------------------------------------
uint8_t IO_read_D0L(void)
{
	return( PORT_GetBitValue(&D0L_PORT, D0L_BITPOS));
}
//------------------------------------------------------------------------------------
uint8_t IO_read_D0P(void)
{
	return( PORT_GetBitValue(&D0P_PORT, D0P_BITPOS));
}
//------------------------------------------------------------------------------------
uint8_t IO_read_D1L(void)
{
	return( PORT_GetBitValue(&D1L_PORT, D1L_BITPOS));
}
//------------------------------------------------------------------------------------
uint8_t IO_read_D1P(void)
{
	return( PORT_GetBitValue(&D1P_PORT, D1P_BITPOS));
}
//------------------------------------------------------------------------------------
uint8_t IO_read_SLEEP_CTL(void)
{
	return( PORT_GetBitValue(&SLEEP_CTL_PORT, SLEEP_CTL_BITPOS));
}
//------------------------------------------------------------------------------------
uint8_t IO_read_TERMCTL_PIN(void)
{
	return( PORT_GetBitValue(&TERMCTL_PIN_PORT, TERMCTL_PIN_BITPOS));
}
//------------------------------------------------------------------------------------
// 12V CONTROL PARA SENSORES
//------------------------------------------------------------------------------------
/*
void IO_config_SENS_12V_CTL(void)
{
	// salida
	PORT_SetPinAsOutput( &SENS_12V_CTL_PORT, SENS_12V_CTL_BITPOS);

}
//------------------------------------------------------------------------------------
void IO_set_SENS_12V_CTL(void)
{
	// Pongo en 1
	PORT_SetOutputBit( &SENS_12V_CTL_PORT, SENS_12V_CTL_BITPOS);
}
//------------------------------------------------------------------------------------
void IO_clr_SENS_12V_CTL(void)
{
	// Pongo en 0
	PORT_ClearOutputBit( &SENS_12V_CTL_PORT, SENS_12V_CTL_BITPOS);
}
*/
//------------------------------------------------------------------------------------
// 3V3AN PARA INA ( Como el INA es I2C debe actuar junto a MEMVCC )
//------------------------------------------------------------------------------------
/*
void IO_config_AN_3V3_CTL(void)
{
	// salida
	PORT_SetPinAsOutput( &AN_3V3_CTL_PORT, AN_3V3_CTL_BITPOS);

}
//------------------------------------------------------------------------------------
void IO_set_AN_3V3_CTL(void)
{
	// Pongo en 1
	PORT_SetOutputBit( &AN_3V3_CTL_PORT, AN_3V3_CTL_BITPOS);
}
//------------------------------------------------------------------------------------
void IO_clr_AN_3V3_CTL(void)
{
	// Pongo en 0
	PORT_ClearOutputBit( &AN_3V3_CTL_PORT, AN_3V3_CTL_BITPOS);
}
*/
//------------------------------------------------------------------------------------
// BLUETOOTH
//------------------------------------------------------------------------------------
/*
void IO_config_BT_PWR_CTL(void)
{
	// Configuro todos los pines que estan afectados al BT

	// PWR: salida.
	PORT_SetPinAsOutput( &BT_PWR_PORT, BT_PWR_BITPOS);
	//

}
//------------------------------------------------------------------------------------
void IO_set_BT_PWR_CTL(void)
{
	// Pongo el bit en 1
	PORT_SetOutputBit( &BT_PWR_PORT, BT_PWR_BITPOS);
}
//------------------------------------------------------------------------------------
void IO_clr_BT_PWR_CTL(void)
{
	// Pongo el bit en 0
	PORT_ClearOutputBit( &BT_PWR_PORT, BT_PWR_BITPOS);
}
*/
//------------------------------------------------------------------------------------
// VCC de EEPROM / FLASH / RTC ( Como son I2C debe actuar junto a 3V3AN )
//------------------------------------------------------------------------------------
/*
void IO_config_MEM_VCC(void)
{
	PORT_SetPinAsOutput( &MEM_VCC_PORT, MEM_VCC_BITPOS);
}
//------------------------------------------------------------------------------------
void IO_set_MEM_VCC(void)
{
	// Pongo el bit en 1
	PORT_SetOutputBit( &MEM_VCC_PORT, MEM_VCC_BITPOS);
}
//------------------------------------------------------------------------------------
void IO_clr_MEM_VCC(void)
{
	// Pongo el bit en 0
	PORT_ClearOutputBit( &MEM_VCC_PORT, MEM_VCC_BITPOS);
}
*/
//------------------------------------------------------------------------------------
// FLASH SPI CS
//------------------------------------------------------------------------------------
/*
void IO_config_SPIMEM_CS(void)
{
	// salida
	PORT_SetPinAsOutput( &SPIMEM_CS_PORT, SPIMEM_CS_BITPOS);

}
//------------------------------------------------------------------------------------
void IO_set_SPIMEM_CS(void)
{
	// Pongo en 1
	PORT_SetOutputBit( &SPIMEM_CS_PORT, SPIMEM_CS_BITPOS);
}
//------------------------------------------------------------------------------------
void IO_clr_SPIMEM_CS(void)
{
	// Pongo en 0
	PORT_ClearOutputBit( &SPIMEM_CS_PORT, SPIMEM_CS_BITPOS);
}
*/
//------------------------------------------------------------------------------------
/*
 *  This function configures interrupt 1 to be associated with a set of pins and
 *  sets the desired interrupt level.
 *
 *  port       The port to configure.
 *  intLevel   The desired interrupt level for port interrupt 1.
 *  pinMask    A mask that selects the pins to associate with port interrupt 1.
 */
void PORT_ConfigureInterrupt0( PORT_t * port,PORT_INT0LVL_t intLevel,uint8_t pinMask )
{
	port->INTCTRL = ( port->INTCTRL & ~PORT_INT0LVL_gm ) | intLevel;
	port->INT0MASK = pinMask;
}
//------------------------------------------------------------------------------------
void PORT_ConfigureInterrupt1( PORT_t * port, PORT_INT1LVL_t intLevel,uint8_t pinMask )
{
	port->INTCTRL = ( port->INTCTRL & ~PORT_INT1LVL_gm ) | intLevel;
	port->INT1MASK = pinMask;
}

