/*
 * ee_sp5K.c
 *
 *  Created on: 26/10/2015
 *      Author: pablo
 */

#include "../spx_libs/l_eeprom.h"

//------------------------------------------------------------------------------------
bool EE_read( uint32_t eeAddress, char *data, uint8_t length )
{
	// Lee en la EE, desde la posicion 'address', 'length' bytes
	// y los deja en el buffer apuntado por 'data'

size_t xReturn = 0U;
uint16_t val = 0;
uint8_t xBytes = 0;

		// Lo primero es obtener el semaforo
		FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL);

		// Luego indicamos el periferico i2c en el cual queremos leer
#ifdef M24M02
		// Uso el bit A16 y A17 de la direccion del dato para la
		// direccion del dispositivo
//		// Por ahora lo dejo en 00.
		val = EE_ADDR | (( eeAddress & 0x30000 ) >> 15);
#else
		val = EE_ADDR;
#endif

		FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val);

		// Luego indicamos la direccion desde donde leer: largo ( 2 bytes )
		val = 2;
		FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val);
		// y direccion
		// Hago un cast para dejarla en 16 bits.
		val = (uint16_t)(eeAddress);
		FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);

		// Por ultimo leemos.
		xBytes = length;
		xReturn = FreeRTOS_read(&pdI2C, data, xBytes);
		// Y libero el semaforo.
		FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL);

		if (xReturn != xBytes ) {
			return ( false );
		}

		return(true);

}
//------------------------------------------------------------------------------------
bool EE_write( uint32_t eeAddress, char *data, uint8_t length )
{
	// Escribe en la EE a partir de la posicion 'eeAddress', la cantidad
	// 'length' de bytes apuntados por 'data'
	// Puedo estar escribiendo un pageWrite entonces debo controlar no
	// salime de la pagina.
	//
size_t xReturn = 0U;
uint16_t val = 0;
uint8_t xBytes = 0;
uint16_t n, pageBytesFree;

	// Lo primero es obtener el semaforo
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL);

	// Luego indicamos el periferico i2c ( EEPROM ) en el cual queremos leer
	// Luego indicamos el periferico i2c en el cual queremos leer
#ifdef M24M02
	// Uso el bit A16 y A17 de la direccion del dato para la
	// direccion del dispositivo
	// Por ahora lo dejo en 00.
	val = EE_ADDR | (( eeAddress & 0x30000 ) >> 15);
#else
	val = EE_ADDR;
#endif

	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val);
	// Luego indicamos la direccion a partir de donde escribir: largo ( 2 bytes )
	val = 2;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val);
	// y direccion
	// Hago un cast para dejarla en 16 bits.
	val = (uint16_t)(eeAddress);
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);

	// Por ultimo escribimos xBytes.
	// Controlo no hacer page overflow
	xBytes = length;
	n = length % EE_PAGESIZE;
	pageBytesFree = (n+1)*EE_PAGESIZE - length;
	if ( pageBytesFree < length ) {
		xBytes = pageBytesFree;
	}
	xReturn = FreeRTOS_write(&pdI2C, data, xBytes);
	// Y libero el semaforo.
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL);

	if (xReturn != xBytes ) {
		return ( false );
	}

	return(true);

}
//------------------------------------------------------------------------------------
bool EE_test_write(char *s0, char *s1)
{
	/* Se usa para testear desde el modo comando las funciones de escribir la eeprom.
	 * Desde el modo comando ingresamos 2 parametros que son 2 strings: la direccion y el texto
	 * Para usar EE_write debemos calcular el largo del texto antes de invocarla
	 */

uint8_t length = 0;
char *p;
bool retS = false;

	// Calculamos el largo del texto a escribir en la eeprom.
	p = s1;
	while (*p != 0) {
		p++;
		length++;
	}
//	snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("S=[%s](%d)\r\n\0"),s1, length);
//	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	retS = EE_write( (uint32_t)(atol(s0)), s1, length );
	return(retS);
}
//-----------------------------------------------------------------------------------
bool EE_test_read(char *s0, char *s1, char *s2)
{
	/* Se usa para testear desde el modo comando las funciones de leer la eeprom.
	 * Desde el modo comando ingresamos 2 parametros que son 2 strings: la direccion y el largo
	 */

bool retS = false;

	retS = EE_read( (uint32_t)(atol(s0)), s1, (uint8_t)(atoi(s2)) );
	return(retS);
}
//-----------------------------------------------------------------------------------
