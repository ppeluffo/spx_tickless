/*------------------------------------------------------------------------------------
 * rtc_sp5KFRTOS.h
 * Autor: Pablo Peluffo @ 2015
 * Basado en Proycon AVRLIB de Pascal Stang.
 *
 * Son funciones que impelementan la API de acceso al RTC del sistema SP5K con FRTOS.
 * Para su uso debe estar inicializado el semaforo del bus I2C, que se hace llamando a i2cInit().
 *
 *
*/

// --------------------------------------------------------------------------------
// SPV5 LIB
// --------------------------------------------------------------------------------

#ifndef AVRLIBFRTOS_RTC_SP5KFRTOS_H_
#define AVRLIBFRTOS_RTC_SP5KFRTOS_H_

#include "FRTOS-IO.h"
#include "stdio.h"

// Direccion del bus I2C donde esta el DS1340
#define DS1340_DEVADDR		   	0xD0

typedef struct
{
	// Tamanio: 7 byte.
	// time of day
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	// date
	uint8_t day;
	uint8_t month;
	uint16_t year;

} RtcTimeType_t;


bool DS1340_read(RtcTimeType_t *rtc);
bool DS1340_write(RtcTimeType_t *rtc);

void DS1340_rtc2str(char *str, RtcTimeType_t *rtc);
bool DS1340_str2rtc(char *str, RtcTimeType_t *rtc);


#endif /* AVRLIBFRTOS_RTC_SP5KFRTOS_H_ */
