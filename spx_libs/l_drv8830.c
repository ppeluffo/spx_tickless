/*
 * l_drv8830.c
 *
 *  Created on: 18 de oct. de 2017
 *      Author: pablo
 */

#include "../spx_libs/l_drv8830.h"

//------------------------------------------------------------------------------------
bool DRV8830_read( uint8_t deviceId, uint8_t regAddr, char *data )
{
	// En los DRV8830 solo hay un registro para leer, el de fault en la
	// direccion 0x01

size_t xReturn = 0U;
uint8_t val = 0;
uint8_t xBytes = 0;
uint16_t regAddress;

		// Lo primero es obtener el semaforo
		FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL);

		// Luego indicamos el periferico i2c en el cual queremos leer
		val = deviceId;
		FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val);
		snprintf_P( debug_printfBuff, sizeof(debug_printfBuff),PSTR( "DRV8830: devAddr=0x%02x\r\n\0"), val );
		CMD_write( debug_printfBuff, sizeof(debug_printfBuff) );

		// Luego indicamos el registro desde donde leer: largo ( 1 bytes )
		val = 1;
		FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val);

		// y direccion ( 16 bits )
		regAddress = regAddr;
		FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&regAddress);
		snprintf_P( debug_printfBuff, sizeof(debug_printfBuff),PSTR( "DRV8830: regAddr=0x%02x\r\n\0"), regAddress );
		CMD_write( debug_printfBuff, sizeof(debug_printfBuff) );

		// Por ultimo leemos.( 1 byte )
		xBytes = 1;
		xReturn = FreeRTOS_read(&pdI2C, data, xBytes);

		// Y libero el semaforo.
		FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL);

		if (xReturn != xBytes ) {
			return ( false );
		}

		return(true);

}
//------------------------------------------------------------------------------------
bool DRV8830_write(  uint8_t deviceId, uint8_t regAddr, char *data )
{
	// En los DRV8830 solo hay un registro para escribit, el de control en la
	// direccion 0x00

size_t xReturn = 0U;
uint8_t val = 0;
uint8_t xBytes = 0;
uint16_t regAddress;

	// Lo primero es obtener el semaforo
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL);

	// Luego indicamos el periferico i2c en el cual queremos escribir
	val = deviceId;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val);
	snprintf_P( debug_printfBuff, sizeof(debug_printfBuff),PSTR( "DRV8830: devAddr=0x%02x\r\n\0"), val );
	CMD_write( debug_printfBuff, sizeof(debug_printfBuff) );

	// Luego indicamos la direccion ( el registro ) a partir de donde escribir: largo ( 1 bytes )
	val = 1;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val);

	// y direccion ( registro ) ( 16 bits )
	regAddress = regAddr;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&regAddress);
	snprintf_P( debug_printfBuff, sizeof(debug_printfBuff),PSTR( "DRV8830: regAddr=0x%02x\r\n\0"), regAddress );
	CMD_write( debug_printfBuff, sizeof(debug_printfBuff) );

	// Por ultimo escribimos xBytes.
	xBytes = 1;	// En los INA siempre son 2 bytes
	xReturn = FreeRTOS_write(&pdI2C, data, xBytes);

	// Y libero el semaforo.
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL);

	if (xReturn != xBytes ) {
		return ( false );
	}

	return(true);

}
//------------------------------------------------------------------------------------
