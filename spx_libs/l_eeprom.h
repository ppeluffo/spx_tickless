/*
 * ee_sp5K.h
 *
 *  Created on: 26/10/2015
 *      Author: pablo
 */

#ifndef SRC_SPX_LIBS_L_EEPROM_H_
#define SRC_SPX_LIBS_L_EEPROM_H_

#include "FRTOS-IO.h"
#include "stdio.h"

//------------------------------------------------------------------------------------
// Identificacion en el bus I2C en el board sp6KX_LOGICA
// La memoria EE M24M02 es de 1024 paginas de 256 bytes o sea de 256Kbytes.
// Se accede con 16 bits ( 2 bytes de direcciones ) por lo que con los bit A16 y A17
// se indican en la direccion de la memoria en el bus I2C
// EEADDRESS = 0xA | 0 A17 A16 r/w
// Definimos el parametro M24M02 para que al acceder a la EEPROM si tenemos esta memoria
// podamos hacer los calculos correctos de la direccion.
// Debemos cambiar las funciones EE_read / EE_write para la eeAddress sea de 32bits.
#define M24M02	true

#define EE_ADDR			0xA0
#define EE_PAGESIZE		64

bool EE_read( uint32_t eeAddress, char *data, uint8_t length );
bool EE_write( uint32_t eeAddress, char *data, uint8_t length );
bool EE_test_write(char *s0, char *s1);
bool EE_test_read(char *s0, char *s1, char *s2);

#endif /* SRC_SPX_LIBS_L_EEPROM_H_ */
