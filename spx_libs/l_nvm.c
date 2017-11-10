/*
 * l_nvm.c
 *
 *  Created on: 29 de set. de 2017
 *      Author: pablo
 */

#include "l_nvm.h"

static void pv_NVM_EEPROM_FlushBuffer( void );
static void pv_NVM_EEPROM_WaitForNVM( void );
static uint8_t pv_ReadSignatureByte(uint16_t Address);
static void pv_NVM_GetGUID(void);

//----------------------------------------------------------------------------------------
void NVM_readID( char *str )
{
	// Paso los bytes de identificacion a un string para su display.

	if ( !signature_ok ) {
		pv_NVM_GetGUID();
	}

//	snprintf( str, 32 ,"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",signature[0],signature[1],signature[2],signature[3],signature[4],signature[5],signature[6],signature[7],signature[8],signature[9],signature[10]  );
	snprintf( str, 32 ,"%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\r\n",signature[0],signature[1],signature[2],signature[3],signature[4],signature[5],signature[6],signature[7],signature[8],signature[9],signature[10]  );


}
//------------------------------------------------------------------------------------
void NVM_EEPROM_write_buffer(eeprom_addr_t address, const void *buf, uint16_t len)
{
	// Escribe un buffer en la internal EEprom de a 1 byte. No considero paginacion.
	// Utiliza una funcion que hace erase&write.

	if ( address >= EEPROM_SIZE) return;

	while (len > 0) {
		NVM_EEPROM_write_byte(address++, *(uint8_t*)buf);
        buf = (uint8_t*)buf + 1;
        len--;
    }
}
//------------------------------------------------------------------------------------
void NVM_EEPROM_write_byte(eeprom_addr_t address, uint8_t value)
{
	// Escribe de a 1 byte en la internal EEprom


	if ( address >= EEPROM_SIZE ) return;

    /*  Flush buffer to make sure no unintentional data is written and load
     *  the "Page Load" command into the command register.
     */
    pv_NVM_EEPROM_FlushBuffer();
    NVM.CMD = NVM_CMD_LOAD_EEPROM_BUFFER_gc;

    // Set address to write to
    NVM.ADDR2 = 0x00;
    NVM.ADDR1 = (address >> 8) & 0xFF;
    NVM.ADDR0 = address & 0xFF;

	/* Load data to write, which triggers the loading of EEPROM page buffer. */
	NVM.DATA0 = value;

	/*  Issue EEPROM Atomic Write (Erase&Write) command. Load command, write
	 *  the protection signature and execute command.
	 */
	NVM.CMD = NVM_CMD_ERASE_WRITE_EEPROM_PAGE_gc;
	NVM_EXEC();

}
//------------------------------------------------------------------------------------
uint8_t NVM_EEPROM_ReadByte( eeprom_addr_t address )
{
	/* Wait until NVM is not busy. */
	pv_NVM_EEPROM_WaitForNVM();

	/* Set address to read from. */
	NVM.ADDR0 = address & 0xFF;
	NVM.ADDR1 = (address >> 8) & 0xFF;
	NVM.ADDR2 = 0x00;

	/* Issue EEPROM Read command. */
	NVM.CMD = NVM_CMD_READ_EEPROM_gc;
	NVM_EXEC();

	return NVM.DATA0;
}
//------------------------------------------------------------------------------------
void NVM_EEPROM_read_buffer(eeprom_addr_t address, char *buf, uint16_t len)
{

uint8_t rb;

	if ( address >= EEPROM_SIZE) return;

	while (len--) {
		rb = NVM_EEPROM_ReadByte(address++);
		*buf++ = rb;
    }

}
//------------------------------------------------------------------------------------
void NVM_EEPROM_EraseAll( void )
{
	/* Wait until NVM is not busy. */
	pv_NVM_EEPROM_WaitForNVM();

	/* Issue EEPROM Erase All command. */
	NVM.CMD = NVM_CMD_ERASE_EEPROM_gc;
	NVM_EXEC();
}
//------------------------------------------------------------------------------------
void NVM_EEPROM_test_write(char *s0, char *s1)
{
	/* Se usa para testear desde el modo comando las funciones de escribir la eeprom.
	 * Desde el modo comando ingresamos 2 parametros que son 2 strings: la direccion y el texto
	 * Para usar NVMEE_write debemos calcular el largo del texto antes de invocarla
	 */

uint8_t length = 0;
char *p;

	// Calculamos el largo del texto a escribir en la eeprom.
	p = s1;
	while (*p != 0) {
		p++;
		length++;
	}

//	snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("S=[%s](%d)\r\n\0"),s1, length);
//	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	NVM_EEPROM_write_buffer( (uint16_t)(atoi(s0)), s1, length );

}
//-----------------------------------------------------------------------------------
void NVM_EEPROM_test_read(char *s0, char *s1, char *s2)
{
	/* Se usa para testear desde el modo comando las funciones de leer la eeprom.
	 * Desde el modo comando ingresamos 2 parametros que son 2 strings: la direccion y el largo
	 */

	NVM_EEPROM_read_buffer( (uint16_t)(atoi(s0)), s1, (uint8_t)(atoi(s2)) );
}
//-----------------------------------------------------------------------------------
static void pv_NVM_EEPROM_FlushBuffer( void )
{
	// Flushea el eeprom page buffer.

	/* Wait until NVM is not busy. */
	pv_NVM_EEPROM_WaitForNVM();

	/* Flush EEPROM page buffer if necessary. */
	if ((NVM.STATUS & NVM_EELOAD_bm) != 0) {
		NVM.CMD = NVM_CMD_ERASE_EEPROM_BUFFER_gc;
		NVM_EXEC();
	}
}
//------------------------------------------------------------------------------------
static void pv_NVM_EEPROM_WaitForNVM( void )
{
	/* Wait for any NVM access to finish, including EEPROM.
 	 *
 	 *  This function is blcoking and waits for any NVM access to finish,
 	 *  including EEPROM. Use this function before any EEPROM accesses,
 	 *  if you are not certain that any previous operations are finished yet,
 	 *  like an EEPROM write.
 	 */
	do {
		/* Block execution while waiting for the NVM to be ready. */
	} while ((NVM.STATUS & NVM_NVMBUSY_bm) == NVM_NVMBUSY_bm);
}
//------------------------------------------------------------------------------------
static uint8_t pv_ReadSignatureByte(uint16_t Address) {

	// Funcion que lee la memoria NVR, la calibration ROW de a una posicion ( de 16 bits )

	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	uint8_t Result;
	__asm__ ("lpm %0, Z\n" : "=r" (Result) : "z" (Address));
	//  __asm__ ("lpm \n  mov %0, r0 \n" : "=r" (Result) : "z" (Address) : "r0");
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;
	return Result;
}
//----------------------------------------------------------------------------------------
static void pv_NVM_GetGUID(void) {

	// Lee todos los registros de identificacion y los deja en una variable global

	signature[ 0]=pv_ReadSignatureByte(LOTNUM0);
	signature[ 1]=pv_ReadSignatureByte(LOTNUM1);
	signature[ 2]=pv_ReadSignatureByte(LOTNUM2);
	signature[ 3]=pv_ReadSignatureByte(LOTNUM3);
	signature[ 4]=pv_ReadSignatureByte(LOTNUM4);
	signature[ 5]=pv_ReadSignatureByte(LOTNUM5);
	signature[ 6]=pv_ReadSignatureByte(WAFNUM );
	signature[ 7]=pv_ReadSignatureByte(COORDX0);
	signature[ 8]=pv_ReadSignatureByte(COORDX1);
	signature[ 9]=pv_ReadSignatureByte(COORDY0);
	signature[10]=pv_ReadSignatureByte(COORDY1);

	signature_ok = true;

}
//----------------------------------------------------------------------------------------
