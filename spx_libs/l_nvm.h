/*
 * l_nvm.h
 *
 *  Created on: 29 de set. de 2017
 *      Author: pablo
 */

#ifndef SRC_SPX_LIBS_L_NVM_H_
#define SRC_SPX_LIBS_L_NVM_H_

#include "FRTOS-IO.h"

typedef uint16_t eeprom_addr_t;

static uint8_t signature[11];
static bool signature_ok = false;

enum {
	LOTNUM0=8,  // Lot Number Byte 0, ASCII
	LOTNUM1,    // Lot Number Byte 1, ASCII
	LOTNUM2,    // Lot Number Byte 2, ASCII
	LOTNUM3,    // Lot Number Byte 3, ASCII
	LOTNUM4,    // Lot Number Byte 4, ASCII
	LOTNUM5,    // Lot Number Byte 5, ASCII
	WAFNUM =16, // Wafer Number
	COORDX0=18, // Wafer Coordinate X Byte 0
	COORDX1,    // Wafer Coordinate X Byte 1
	COORDY0,    // Wafer Coordinate Y Byte 0
	COORDY1,    // Wafer Coordinate Y Byte 1
};

/* Offset definitions for Production Signatures */
typedef enum NVM_PROD_SIGNATURES_offsets {
    RCOSC2M_offset = 0x00, ///< RCOSC 2MHz Calibration Value
    RCOSC32K_offset = 0x02, ///< RCOSC 32kHz Calibration Value
    RCOSC32M_offset = 0x03, ///< RCOSC 32MHz Calibration Value
    LOTNUM0_offset = 0x08, ///< Lot Number Byte 0, ASCII
    LOTNUM1_offset = 0x09, ///< Lot Number Byte 1, ASCII
    LOTNUM2_offset = 0x0A, ///< Lot Number Byte 2, ASCII
    LOTNUM3_offset = 0x0B, ///< Lot Number Byte 3, ASCII
    LOTNUM4_offset = 0x0C, ///< Lot Number Byte 4, ASCII
    LOTNUM5_offset = 0x0D, ///< Lot Number Byte 5, ASCII
    WAFNUM_offset = 0x10, ///< Wafer Number
    COORDX0_offset = 0x12, ///< Wafer Coordinate X Byte 0
    COORDX1_offset = 0x13, ///< Wafer Coordinate X Byte 1
    COORDY0_offset = 0x14, ///< Wafer Coordinate Y Byte 0
    COORDY1_offset = 0x15, ///< Wafer Coordinate Y Byte 1
    ADCACAL0_offset = 0x20, ///< ADCA Calibration Byte 0
    ADCACAL1_offset = 0x21, ///< ADCA Calibration Byte 1
    ADCBCAL0_offset = 0x24, ///< ADCB Calibration Byte 0
    ADCBCAL1_offset = 0x25, ///< ADCB Calibration Byte 1
    TEMPSENSE0_offset = 0x2E, ///< Temperature Sensor Calibration Byte 0
    TEMPSENSE1_offset = 0x2F, ///< Temperature Sensor Calibration Byte 0
    DACAOFFCAL_offset = 0x30, ///< DACA Calibration Byte 0
    DACACAINCAL_offset = 0x31, ///< DACA Calibration Byte 1
    DACBOFFCAL_offset = 0x32, ///< DACB Calibration Byte 0
    DACBGAINCAL_offset = 0x33, ///< DACB Calibration Byte 1
} NVM_PROD_SIGNATURES_offsets_t;

void NVM_readID(char *str );
void NVM_EEPROM_write_buffer(eeprom_addr_t address, const void *buf, uint16_t len);
void NVM_EEPROM_write_byte(eeprom_addr_t address, uint8_t value);
uint8_t NVM_EEPROM_ReadByte( eeprom_addr_t address );
void NVM_EEPROM_read_buffer(eeprom_addr_t address, char *buf, uint16_t len);
void NVM_EEPROM_EraseAll( void );
void NVM_EEPROM_test_write(char *s0, char *s1);
void NVM_EEPROM_test_read(char *s0, char *s1, char *s2);

/*  Non-Volatile Memory Execute Command
 *  This macro set the CCP register before setting the CMDEX bit in the
 *  NVM.CTRLA register.
 *
 *  The CMDEX bit must be set within 4 clock cycles after setting the
 *  protection byte in the CCP register.
 */
#define NVM_EXEC()	asm("push r30"      "\n\t"	\
			    "push r31"      "\n\t"	\
    			    "push r16"      "\n\t"	\
    			    "push r18"      "\n\t"	\
			    "ldi r30, 0xCB" "\n\t"	\
			    "ldi r31, 0x01" "\n\t"	\
			    "ldi r16, 0xD8" "\n\t"	\
			    "ldi r18, 0x01" "\n\t"	\
			    "out 0x34, r16" "\n\t"	\
			    "st Z, r18"	    "\n\t"	\
    			    "pop r18"       "\n\t"	\
			    "pop r16"       "\n\t"	\
			    "pop r31"       "\n\t"	\
			    "pop r30"       "\n\t"	\
			    )


#endif /* SRC_SPX_LIBS_L_NVM_H_ */
