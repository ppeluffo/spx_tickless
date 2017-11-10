/*
 * l_gprs.h
 *
 *  Created on: 21 de oct. de 2017
 *      Author: pablo
 */

#ifndef SRC_SPX_LIBS_L_GPRS_H_
#define SRC_SPX_LIBS_L_GPRS_H_

#include "FRTOS-IO.h"
#include "stdio.h"
#include "l_iopines.h"

//------------------------------------------------------------------------------------

void GPRS_pwron(void);
void GPRS_pwroff(void);
void GPRS_swron(void);
void GPRS_swroff(void);

#endif /* SRC_SPX_LIBS_L_GPRS_H_ */
