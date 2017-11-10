/*
 * l_drv8814.h
 *
 *  Created on: 21 de oct. de 2017
 *      Author: pablo
 */

#ifndef SRC_SPX_LIBS_L_DRV8814_H_
#define SRC_SPX_LIBS_L_DRV8814_H_

#include "FRTOS-IO.h"
#include "stdio.h"
#include "l_iopines.h"

//------------------------------------------------------------------------------------

void DRV8814_pulse(char channel, char phase, uint16_t pulse_width);
bool DRV8814_test_pulse(char *s0, char *s1, char *s2);

#endif /* SRC_SPX_LIBS_L_DRV8814_H_ */
