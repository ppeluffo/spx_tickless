/*
 * l_ina3221.c
 *
 *  Created on: 13 de oct. de 2017
 *      Author: pablo
 */

#include "../spx_libs/l_ina3221.h"

//------------------------------------------------------------------------------------
bool INA3221_read( uint8_t regAddress, char *data, uint8_t length )
{
	// C/registro es de 2 bytes de informacion.

size_t xReturn = 0U;
uint8_t val = 0;
uint8_t xBytes = 0;

		// Lo primero es obtener el semaforo
		FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL);

		// Luego indicamos el periferico i2c en el cual queremos leer
		val = INA3221_ADDR;
		FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val);
//		snprintf_P( debug_printfBuff, sizeof(debug_printfBuff),PSTR( "INArd: devAddr=0x%02x\r\n\0"), val );
//		CMD_write( debug_printfBuff, sizeof(debug_printfBuff) );

		// Luego indicamos el registro desde donde leer: largo ( 1 bytes )
		val = 1;
		FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val);
//		snprintf_P( debug_printfBuff, sizeof(debug_printfBuff),PSTR( "INArd: length=0x%02x\r\n\0"), val );
//		CMD_write( debug_printfBuff, sizeof(debug_printfBuff) );

		// y direccion
		val = regAddress;
		FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);
//		snprintf_P( debug_printfBuff, sizeof(debug_printfBuff),PSTR( "INArd: regAddr=0x%02x\r\n\0"), val );
//		CMD_write( debug_printfBuff, sizeof(debug_printfBuff) );

		// Por ultimo leemos.( 2 bytes )
		xBytes = 2;
		xReturn = FreeRTOS_read(&pdI2C, data, xBytes);
//		snprintf_P( debug_printfBuff, sizeof(debug_printfBuff),PSTR( "INArd: size=0x%02x\r\n\0"), xBytes );
//		CMD_write( debug_printfBuff, sizeof(debug_printfBuff) );

		// Y libero el semaforo.
		FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL);

		if (xReturn != xBytes ) {
			return ( false );
		}

		return(true);

}
//------------------------------------------------------------------------------------
bool INA3221_write( uint8_t regAddress, char *data, uint8_t length )
{
	// Escribe en el INA3221 en la posicion regAddress, la cantidad
	// 'length' de bytes apuntados por 'data'
	// En los INA3221 siempre se escriben solo 2 bytes de datos !!!
	//
size_t xReturn = 0U;
uint8_t val = 0;
uint8_t xBytes = 0;

	// Lo primero es obtener el semaforo
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL);

	// Luego indicamos el periferico i2c ( INA3221 ) en el cual queremos leer
	val = INA3221_ADDR;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val);

	// Luego indicamos la direccion ( el registro ) a partir de donde escribir: largo ( 1 bytes )
	val = 1;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val);
	// y direccion ( registro )
	val = regAddress;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);

	// Por ultimo escribimos xBytes.
	//xBytes = length;
	xBytes = 2;	// En los INA siempre son 2 bytes
	xReturn = FreeRTOS_write(&pdI2C, data, xBytes);

	// Y libero el semaforo.
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL);

	if (xReturn != xBytes ) {
		return ( false );
	}

	return(true);

}
//------------------------------------------------------------------------------------
bool INA3221_test_write(char *s0, char *s1)
{
	/* Se usa para testear desde el modo comando las funciones de escribir la ina.
	 * El unico registro que programamos es el de configuracion, 0.
	 */

uint8_t regAddress;
uint16_t value;
char data[3];

	regAddress = atoi(s0);
	value = atoi(s1);
	memset(data,'\0', sizeof(data));
	data[0] = ( value & 0xFF00 ) >> 8;
	data[1] = ( value & 0x00FF );

	snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("Raddr=%d, Val=0x%04x, d0=0x%02x,d1=0x%02x \r\n\0"),regAddress,value,data[0],data[1]);
	CMD_write( debug_printfBuff, sizeof(debug_printfBuff) );


 	INA3221_write( regAddress, data, 2 );
	return(true);
}
//-----------------------------------------------------------------------------------
bool INA3221_test_read(uint8_t regAddress )
{
	/* Se usa para testear desde el modo comando las funciones de leer la eeprom.
	 * Desde el modo comando ingresamos 2 parametros que son 2 strings: la direccion y el largo
	 */

bool retS = false;
char res[3];
uint16_t val1;
float mV;

	memset(res,'\0', sizeof(res));
	retS = INA3221_read(regAddress, res, 2 );

//	snprintf_P( debug_printfBuff, sizeof(debug_printfBuff),PSTR( "INArd: r=0x%02x-> 0x%02x%02x \r\n\0"), regAddress, res[0],res[1]);
//	CMD_write( debug_printfBuff, sizeof(debug_printfBuff) );

	val1 = 0;
	val1 = ( res[0]<< 8 ) + res[1];
	val1 = val1 >> 3;

	mV = ( val1 / 1000.0 * 40 );

	snprintf_P( debug_printfBuff, sizeof(debug_printfBuff),PSTR( "INArd: r=0x%02x-> 0x%02x%02x , val=%05d, mV=%.2f\r\n\0"), regAddress, res[0],res[1], val1, mV);
	CMD_write( debug_printfBuff, sizeof(debug_printfBuff) );

	return(retS);
}
//-----------------------------------------------------------------------------------
