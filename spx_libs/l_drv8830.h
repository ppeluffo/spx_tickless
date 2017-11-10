/*
 * l_drv8830.h
 *
 *  Created on: 18 de oct. de 2017
 *      Author: pablo
 */

#ifndef SRC_SPX_LIBS_L_DRV8830_H_
#define SRC_SPX_LIBS_L_DRV8830_H_

#include "FRTOS-IO.h"
#include "stdio.h"

//------------------------------------------------------------------------------------

#define DRV8830_A_ADDR			0xC0
#define DRV8830_B_ADDR			0xCC

#define DRV8830_CONTROL			0x0000
#define DRV8830_FAULT			0x0001

bool DRV8830_read( uint8_t deviceId, uint8_t regAddr, char *data );
bool DRV8830_write(  uint8_t deviceId, uint8_t regAddr, char *data );

#endif /* SRC_SPX_LIBS_L_DRV8830_H_ */
