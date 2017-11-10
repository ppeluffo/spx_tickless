/*
 * sp5KFRTOS_rtc.c
 *
 *  Created on: 01/11/2013
 *      Author: root
 *
 * Funciones del RTC DS1340-33 modificadas para usarse con FRTOS.
 *
 *
 */
//------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------
// SPV5 LIB
// --------------------------------------------------------------------------------

#include <l_ds1340.h>

static char pv_bcd2dec(char num);
static char pv_dec2bcd(char num);

//------------------------------------------------------------------------------------
// Funciones de uso general
//------------------------------------------------------------------------------------
bool DS1340_read(RtcTimeType_t *rtc)
{
	// Retorna la hora formateada en la estructura RtcTimeType_t
	// No retornamos el valor de EOSC ni los bytes adicionales.

uint8_t data[8];
size_t xReturn = 0U;
uint16_t val = 0;
uint8_t xBytes = 0;

	// Lo primero es obtener el semaforo
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL);
	// Luego indicamos el periferico i2c en el cual queremos leer
	val = DS1340_DEVADDR;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val);
	// Luego indicamos la direccion desde donde leer: largo ( 2 bytes )
	val = 1;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val);
	// y direccion
	val = 0;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);

	// Por ultimo leemos 8 bytes.
	xBytes = sizeof(data);
	xReturn = FreeRTOS_read(&pdI2C, &data, xBytes);
	// Y libero el semaforo.
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL);

	if (xReturn != xBytes ) {
		return ( false );
	}

	// Decodifico los resultados del buffer para ponerlos en la estructura RTC

	rtc->sec = pv_bcd2dec(data[0] & 0x7F);
	rtc->min = pv_bcd2dec(data[1]);
	rtc->hour = pv_bcd2dec(data[2] & 0x3F);
	rtc->day = pv_bcd2dec(data[4] & 0x3F);
	rtc->month = pv_bcd2dec(data[5] & 0x1F);
	rtc->year = pv_bcd2dec(data[6]) + 2000;

	return(true);
}
//------------------------------------------------------------------------------------
bool DS1340_write(RtcTimeType_t *rtc)
{
	// Setea el RTC con la hora pasada en la estructura RtcTimeType

uint8_t data[8];
size_t xReturn = 0U;
uint16_t val = 0;
uint8_t xBytes = 0;

	data[0] = 0;	// EOSC = 0 ( rtc running)
	data[1] = pv_dec2bcd(rtc->min);
	data[2] = pv_dec2bcd(rtc->hour);
	data[3] = 0;
	data[4] = pv_dec2bcd(rtc->day);
	data[5] = pv_dec2bcd(rtc->month);
	data[6] = pv_dec2bcd(rtc->year);

	// Lo primero es obtener el semaforo
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL);
	// Luego indicamos el periferico i2c ( EEPROM ) en el cual queremos leer
	val = DS1340_DEVADDR;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val);
	// Luego indicamos la direccion a partir de donde escribir: largo ( 2 bytes )
	val = 1;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val);
	// y direccion
	val = 0;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);

	// Por ultimo escribimos el buffer data.
	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);
	// Y libero el semaforo.
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL);

	if (xReturn != xBytes ) {
		return ( false );
	}

	return(true);

}
//------------------------------------------------------------------------------------
void DS1340_rtc2str(char *str, RtcTimeType_t *rtc)
{
	// Convierte los datos del RTC a un string con formato DD/MM/YYYY hh:mm:ss

	snprintf( str, 32 ,"%02d/%02d/%04d %02d:%02d:%02d\r\n",rtc->day,rtc->month, rtc->year, rtc->hour,rtc->min, rtc->sec );

}
//------------------------------------------------------------------------------------
bool DS1340_str2rtc(char *str, RtcTimeType_t *rtc)
{
	// Convierto los datos de un string con formato YYMMDDhhmm a RTC

char dateTimeStr[11];
char tmp[3];
bool retS;


	/* YYMMDDhhmm */
	if ( str == NULL )
		return(false);

		memcpy(dateTimeStr, str, 10);
		// year
		tmp[0] = dateTimeStr[0]; tmp[1] = dateTimeStr[1];	tmp[2] = '\0';
		rtc->year = atoi(tmp);
		// month
		tmp[0] = dateTimeStr[2]; tmp[1] = dateTimeStr[3];	tmp[2] = '\0';
		rtc->month = atoi(tmp);
		// day of month
		tmp[0] = dateTimeStr[4]; tmp[1] = dateTimeStr[5];	tmp[2] = '\0';
		rtc->day = atoi(tmp);
		// hour
		tmp[0] = dateTimeStr[6]; tmp[1] = dateTimeStr[7];	tmp[2] = '\0';
		rtc->hour = atoi(tmp);
		// minute
		tmp[0] = dateTimeStr[8]; tmp[1] = dateTimeStr[9];	tmp[2] = '\0';
		rtc->min = atoi(tmp);

		return(retS);

}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
static char pv_dec2bcd(char num)
{
	// Convert Decimal to Binary Coded Decimal (BCD)
	return ((num/10 * 16) + (num % 10));
}
//------------------------------------------------------------------------------------
static char pv_bcd2dec(char num)
{
	// Convert Binary Coded Decimal (BCD) to Decimal
	return ((num/16 * 10) + (num % 16));
}
//------------------------------------------------------------------------------------
