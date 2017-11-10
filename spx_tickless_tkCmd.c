/*
 * sp5K_tkCmd.c
 *
 *  Created on: 27/12/2013
 *      Author: root
 */


#include <l_ds1340.h>
#include "spx_libs/l_eeprom.h"
#include "spx_libs/l_nvm.h"
#include "spx_libs/l_spiflash.h"
#include "spx_libs/l_uarts.h"
#include "spx_libs/l_ina3221.h"
#include "spx_libs/l_drv8830.h"
#include "spx_libs/l_iopines.h"
#include "spx_libs/l_drv8814.h"

#include "FRTOS-IO/FRTOS-CMD.h"
#include "spx_tickless.h"

static char cmd_printfBuff[256];

//----------------------------------------------------------------------------------------
// FUNCIONES DE USO PRIVADO
//----------------------------------------------------------------------------------------
static void pv_snprintfP_OK(void );
static void pv_snprintfP_ERR(void);

//----------------------------------------------------------------------------------------
// FUNCIONES DE CMDMODE
//----------------------------------------------------------------------------------------
static void cmdHelpFunction(void);
static void cmdClearScreen(void);
static void cmdResetFunction(void);
static void cmdWriteFunction(void);
static void cmdReadFunction(void);

/*------------------------------------------------------------------------------------*/
void tkCmd(void * pvParameters)
{

uint8_t c;
uint8_t ticks;
uint8_t cmd_pin;

( void ) pvParameters;

	vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	FRTOS_CMD_init();

	// Registro los comandos y los mapeo al cmd string.
	FRTOS_CMD_register( "cls\0", cmdClearScreen );
	FRTOS_CMD_register( "reset\0", cmdResetFunction);
	FRTOS_CMD_register( "write\0", cmdWriteFunction);
	FRTOS_CMD_register( "read\0", cmdReadFunction);
	FRTOS_CMD_register( "help\0", cmdHelpFunction );

	// Espero la notificacion para arrancar
	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("starting tkCmd..\r\n\0"));
	CMD_write(cmd_printfBuff, sizeof(cmd_printfBuff) );

	// Fijo el timeout del READ
	ticks = 5;
	FreeRTOS_ioctl( &pdUART_USB,ioctlSET_TIMEOUT, &ticks );
	FreeRTOS_ioctl( &pdUART_BT,ioctlSET_TIMEOUT, &ticks );

	//FRTOS_CMD_regtest();
	// loop
	for( ;; )
	{

		cmd_pin = IO_read_TERMCTL_PIN();
		if ( cmd_pin == 1 ) {
			vTaskDelay( ( TickType_t)( 3000 / portTICK_RATE_MS ) );
		} else {
			c = '\0';	// Lo borro para que luego del un CR no resetee siempre el timer.
			// el read se bloquea 50ms. lo que genera la espera.
			while ( CMD_read( &c, 1 ) == 1 ) {
				FRTOS_CMD_process(c);
			}
		}
	}

}
/*------------------------------------------------------------------------------------*/
static void cmdResetFunction(void)
{
	// Resetea al micro prendiendo el watchdog

//uint8_t temp;

 	//Software reset
	CCP = 0xD8;			// Protected configuration signature
	RST.CTRL = 0x01;	// Reset.

	while(1)
		;

/*
	temp = WDT_ENABLE_bm | WDT_CEN_bm |  WDT_PER_8KCLK_gc ;
	CCP = CCP_IOREG_gc;
	WDT.CTRL = temp;
	while(WDT_IsSyncBusy());

	WDT_Reset();

	while(1)
		;
*/
}
/*------------------------------------------------------------------------------------*/
static void cmdWriteFunction(void)
{

int8_t retS = FALSE;
uint8_t argc;
RtcTimeType_t rtc;
uint8_t outReg;
char *msg = NULL;

	argc = FRTOS_CMD_makeArgv();

	// SLEEP
	if (!strcmp_P( strupr(argv[1]), PSTR("SLEEP\0")) ) {
		sleep_enable();
		sei();
		//sleep_cpu();
		sleep_mode();
		sleep_disable();
		cli();
		sleepFlag = true;
		return;
	}

	// write gprs pwr|sw {on|off}
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRS\0")) ) {

		if (!strcmp_P( strupr(argv[2]), PSTR("PWR\0")) ) {

			if (!strcmp_P( strupr(argv[3]), PSTR("ON\0")) ) {
				IO_set_GPRS_PWR();
				pv_snprintfP_OK();
				return;
			}

			if (!strcmp_P( strupr(argv[3]), PSTR("OFF\0")) ) {
				IO_clr_GPRS_PWR();
				pv_snprintfP_OK();
				return;
			}

			pv_snprintfP_ERR();
			return;
		}

		if (!strcmp_P( strupr(argv[2]), PSTR("SW\0")) ) {

			// EL pin SW invierte la señal con un transistor
			if (!strcmp_P( strupr(argv[3]), PSTR("ON\0")) ) {
				IO_clr_GPRS_SW();
				pv_snprintfP_OK();
				return;
			}

			if (!strcmp_P( strupr(argv[3]), PSTR("OFF\0")) ) {
				IO_set_GPRS_SW();
				pv_snprintfP_OK();
				return;
			}

			pv_snprintfP_ERR();
			return;
		}

		pv_snprintfP_ERR();
		return;
	}

	// EEPROM
	// EE: write ee pos string
	if (!strcmp_P( strupr(argv[1]), PSTR("EE\0")) ) {
		retS = EE_test_write( argv[2], argv[3]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// SPIFLASH
	// SPI: write spi wr pos string
	if (!strcmp_P( strupr(argv[1]), PSTR("SPI\0")) ) {

		if (!strcmp_P( strupr(argv[2]), PSTR("WR\0")) ) {
			retS = SPIFLASH_test_write( argv[3], argv[4]);
			retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
			return;
		}

		if (!strcmp_P( strupr(argv[2]), PSTR("SETCS\0")) ) {
			SPIFLASH_test_assertCS();
			pv_snprintfP_OK();
			return;
		}

		if (!strcmp_P( strupr(argv[2]), PSTR("RELCS\0")) ) {
			SPIFLASH_test_releaseCS();
			pv_snprintfP_OK();
			return;
		}

		if (!strcmp_P( strupr(argv[2]), PSTR("ENABLE\0")) ) {
			SPIFLASH_write_enable();
			pv_snprintfP_OK();
			return;
		}

		if (!strcmp_P( strupr(argv[2]), PSTR("DISABLE\0")) ) {
			SPIFLASH_write_disable();
			pv_snprintfP_OK();
			return;
		}

		if (!strcmp_P( strupr(argv[2]), PSTR("ERASE\0")) ) {
			SPIFLASH_chip_erase();
			pv_snprintfP_OK();
			return;
		}

		if (!strcmp_P( strupr(argv[2]), PSTR("PERASE\0")) ) {
			SPIFLASH_page_erase( (uint16_t) atol(argv[3]) );
			pv_snprintfP_OK();
			return;
		}

	}

	// RTC DS1340
	// write rtc YYMMDDhhmm
	if (!strcmp_P( strupr(argv[1]), PSTR("RTC\0")) ) {
		// Convierto el string a una estructura rtc
		DS1340_str2rtc(argv[2], &rtc);
		// Escribo el rtc
		DS1340_write( &rtc) ?  pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// NVMEE
	// NVM: write nvmee pos string
	if (!strcmp_P( strupr(argv[1]), PSTR("NVMEE\0")) ) {
		NVM_EEPROM_test_write( argv[2], argv[3]);
		pv_snprintfP_OK();
		return;
	}

	// memvcc on|off
	// write memvcc on|off
	if (!strcmp_P( strupr(argv[1]), PSTR("MEMVCC\0")) ) {
		if (!strcmp_P( strupr(argv[2]), PSTR("ON\0")) ) {
			IO_set_MEM_VCC();
			pv_snprintfP_OK();
			return;
		}
		if (!strcmp_P( strupr(argv[2]), PSTR("OFF\0")) ) {
			IO_clr_MEM_VCC();
			pv_snprintfP_OK();
			return;
		}

		pv_snprintfP_ERR();
		return;
	}

	// btvcc on|off
	// write btvcc on|off
	if (!strcmp_P( strupr(argv[1]), PSTR("BTVCC\0")) ) {
		if (!strcmp_P( strupr(argv[2]), PSTR("ON\0")) ) {
			IO_set_BT_PWR_CTL();
			pv_snprintfP_OK();
			return;
		}
		if (!strcmp_P( strupr(argv[2]), PSTR("OFF\0")) ) {
			IO_clr_BT_PWR_CTL();
			pv_snprintfP_OK();
			return;
		}

		pv_snprintfP_ERR();
		return;
	}

	// an3 on|off
	// write an3 on|off
	if (!strcmp_P( strupr(argv[1]), PSTR("AN3\0")) ) {
		if (!strcmp_P( strupr(argv[2]), PSTR("ON\0")) ) {
			IO_set_AN_3V3_CTL();
			pv_snprintfP_OK();
			return;
		}
		if (!strcmp_P( strupr(argv[2]), PSTR("OFF\0")) ) {
			IO_clr_AN_3V3_CTL();
			pv_snprintfP_OK();
			return;
		}

		pv_snprintfP_ERR();
		return;
	}

	// sens12 on|off
	// write sens12 on|off
	if (!strcmp_P( strupr(argv[1]), PSTR("SENS12\0")) ) {
		if (!strcmp_P( strupr(argv[2]), PSTR("ON\0")) ) {
			IO_set_SENS_12V_CTL();
			pv_snprintfP_OK();
			return;
		}
		if (!strcmp_P( strupr(argv[2]), PSTR("OFF\0")) ) {
			IO_clr_SENS_12V_CTL();
			pv_snprintfP_OK();
			return;
		}

		pv_snprintfP_ERR();
		return;
	}

	// INA
	// write ina confRegValue
	if (!strcmp_P( strupr(argv[1]), PSTR("INA\0")) ) {
		INA3221_test_write("0",argv[2]);
		pv_snprintfP_OK();
		return;
	}

	// clrd 0|1
	// write sens12 on|off
	if (!strcmp_P( strupr(argv[1]), PSTR("CLRD\0")) ) {
		if (atoi(argv[2]) == 0) {
			IO_clr_CLRD();
			pv_snprintfP_OK();
			return;
		}
		if (atoi(argv[2]) == 1) {
			IO_set_CLRD();
			pv_snprintfP_OK();
			return;
		}

		pv_snprintfP_ERR();
		return;
	}

	// OUT 8814
	// write out sleep|reset|phase(A/B)|enable(A/B)| {0|1}
	if (!strcmp_P( strupr(argv[1]), PSTR("OUT\0")) ) {

		// write out sleep 0,1
		if (!strcmp_P( strupr(argv[2]), PSTR("SLEEP\0")) ) {
			if (atoi(argv[3]) == 0) {
				IO_clr_SLP();
				pv_snprintfP_OK();
				return;
			}
			if (atoi(argv[3]) == 1) {
				IO_set_SLP();
				pv_snprintfP_OK();
				return;
			}
			pv_snprintfP_ERR();
			return;
		}

		// write out reset 0,1
		if (!strcmp_P( strupr(argv[2]), PSTR("RESET\0")) ) {
			if (atoi(argv[3]) == 0) {
				IO_clr_RES();
				pv_snprintfP_OK();
				return;
			}
			if (atoi(argv[3]) == 1) {
				IO_set_RES();
				pv_snprintfP_OK();
				return;
			}
			pv_snprintfP_ERR();
			return;
		}

		// write out phase (a/b) (0/1)
		if (!strcmp_P( strupr(argv[2]), PSTR("PHASE\0")) ) {
			if ( toupper(argv[3][0]) == 'A'  ) {
				if (atoi(argv[4]) == 0) {
					IO_clr_PHA();
					pv_snprintfP_OK();
					return;
				}
				if (atoi(argv[4]) == 1) {
					IO_set_PHA();
					pv_snprintfP_OK();
					return;
				}
				pv_snprintfP_ERR();
				return;
			}

			if ( toupper(argv[3][0]) == 'B'  ) {
				if (atoi(argv[4]) == 0) {
					IO_clr_PHB();
					pv_snprintfP_OK();
					return;
				}
				if (atoi(argv[4]) == 1) {
					IO_set_PHB();
					pv_snprintfP_OK();
					return;
				}
				pv_snprintfP_ERR();
				return;
			}
		}

		// write out enable (a/b) (0/1)
		if (!strcmp_P( strupr(argv[2]), PSTR("ENABLE\0")) ) {
			if ( toupper(argv[3][0]) == 'A'  ) {
				if (atoi(argv[4]) == 0) {
					IO_clr_ENA();
					pv_snprintfP_OK();
					return;
				}
				if (atoi(argv[4]) == 1) {
					IO_set_ENA();
					pv_snprintfP_OK();
					return;
				}
				pv_snprintfP_ERR();
				return;
			}

			if ( toupper(argv[3][0]) == 'B'  ) {
				if (atoi(argv[4]) == 0) {
					IO_clr_ENB();
					pv_snprintfP_OK();
					return;
				}
				if (atoi(argv[4]) == 1) {
					IO_set_ENB();
					pv_snprintfP_OK();
					return;
				}
				pv_snprintfP_ERR();
				return;
			}
		}

		// write out pulse (A/B) (+/-) (ms)
		if (!strcmp_P( strupr(argv[2]), PSTR("PULSE\0")) ) {
			DRV8814_test_pulse(argv[3],argv[4],argv[5]);
			pv_snprintfP_OK();
			return;
		}

		// write out ctl12v on|off
		if (!strcmp_P( strupr(argv[2]), PSTR("CTL12V\0")) ) {

			if (!strcmp_P( strupr(argv[3]), PSTR("ON\0")) ) {
				IO_set_OUT_12V_CTL();
				pv_snprintfP_OK();
				return;
			}
			if (!strcmp_P( strupr(argv[3]), PSTR("OFF\0")) ) {
				IO_clr_OUT_12V_CTL();
				pv_snprintfP_OK();
				return;
			}

			pv_snprintfP_ERR();
			return;
		}

	}

	// OUT 8830
/*
	if (!strcmp_P( strupr(argv[1]), PSTR("OUT\0")) ) {

		// out set {0|1} {value}
		if (!strcmp_P( strupr(argv[2]), PSTR("SET\0")) ) {

			outReg = atoi(argv[3]);
			if (atoi(argv[3]) == 0) {
				DRV8830_write( DRV8830_A_ADDR ,DRV8830_CONTROL, &outReg);
				pv_snprintfP_OK();
				return;
			}
			if (atoi(argv[3]) == 0) {
				DRV8830_write( DRV8830_B_ADDR ,DRV8830_CONTROL,  &outReg);
				pv_snprintfP_OK();
				return;
			}
			pv_snprintfP_ERR();
			return;
		}

		// out clear {0|1}
		if (!strcmp_P( strupr(argv[2]), PSTR("CLEAR\0")) ) {

			outReg = 0x80;
			if (atoi(argv[3]) == 0) {
				DRV8830_write( DRV8830_A_ADDR , DRV8830_FAULT, &outReg);
				pv_snprintfP_OK();
				return;
			}
			if (atoi(argv[3]) == 1) {
				DRV8830_write( DRV8830_B_ADDR , DRV8830_FAULT, &outReg);
				pv_snprintfP_OK();
				return;
			}
			pv_snprintfP_ERR();
			return;
		}

		pv_snprintfP_ERR();
		return;
	}
*/


	// CMD NOT FOUND
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\nCMD NOT DEFINED\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
/*------------------------------------------------------------------------------------*/
static void cmdReadFunction(void)
{

uint8_t argc;
bool retS;
RtcTimeType_t rtc;
uint8_t status;
uint8_t din;
uint8_t out;

uint16_t i;

	argc = FRTOS_CMD_makeArgv();

	// EE
	// read ee address length
	if (!strcmp_P( strupr(argv[1]), PSTR("EE\0"))) {

		memset(cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
		retS = EE_test_read( argv[2], cmd_printfBuff, argv[3] );
		if ( retS ) {
			// El string leido lo devuelve en cmd_printfBuff por lo que le agrego el CR.
			snprintf_P( &cmd_printfBuff[atoi(argv[3])], sizeof(cmd_printfBuff),PSTR( "\r\n\0"));
			CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// SPI
	// read spi re address length
	if (!strcmp_P( strupr(argv[1]), PSTR("SPI\0"))) {

		if (!strcmp_P( strupr(argv[2]), PSTR("RD\0")) ) {
			memset(cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
			retS = SPIFLASH_test_read( argv[3], cmd_printfBuff, argv[4] );
			if ( retS ) {
				// El string leido lo devuelve en cmd_printfBuff por lo que le agrego el CR.
				snprintf_P( &cmd_printfBuff[atoi(argv[4])], sizeof(cmd_printfBuff),PSTR( "\r\n\0"));
				CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
			}
			retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
			return;
		}

		if (!strcmp_P( strupr(argv[2]), PSTR("STATUS\0")) ) {
			status = SPIFLASH_read_status();
			snprintf_P( cmd_printfBuff, sizeof(cmd_printfBuff),PSTR( "SPI STATUS = %x\r\n\0"), status);
			CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
			return;
		}

	}

	// RTC DS1340
	// read rtc
	if (!strcmp_P( strupr(argv[1]), PSTR("RTC\0"))) {

		DS1340_read( &rtc);
		DS1340_rtc2str(cmd_printfBuff, &rtc);
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;
	}

	// SIGNATURE
	// read id
	if (!strcmp_P( strupr(argv[1]), PSTR("ID\0"))) {

		NVM_readID(cmd_printfBuff);
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;
	}

	// NVMEE
	// read nvmee address length
	if (!strcmp_P( strupr(argv[1]), PSTR("NVMEE\0"))) {

		memset(cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
		NVM_EEPROM_test_read( argv[2], cmd_printfBuff, argv[3] );
		// El string leido lo devuelve en cmd_printfBuff por lo que le agrego el CR.
		snprintf_P( &cmd_printfBuff[atoi(argv[3])], sizeof(cmd_printfBuff),PSTR( "\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		pv_snprintfP_OK();
		return;
	}

	// INA3221
	// read ina3221 {conf|chxshv|chxbusv|mfid|dieid}
	if (!strcmp_P( strupr(argv[1]), PSTR("INA\0"))) {

		if (!strcmp_P( strupr(argv[2]), PSTR("CONF\0"))) {
			INA3221_test_read(INA3231_CONF_ADDR);
		} else if (!strcmp_P( strupr(argv[2]), PSTR("CH1SHV\0"))) {
			INA3221_test_read(INA3221_CH1_SHV);
		} else if (!strcmp_P( strupr(argv[2]), PSTR("CH1BUSV\0"))) {
			INA3221_test_read(INA3221_CH1_BUSV);
		} else if (!strcmp_P( strupr(argv[2]), PSTR("CH2SHV\0"))) {
			INA3221_test_read(INA3221_CH2_SHV);
		} else if (!strcmp_P( strupr(argv[2]), PSTR("CH2BUSV\0"))) {
			INA3221_test_read(INA3221_CH2_BUSV);
		} else if (!strcmp_P( strupr(argv[2]), PSTR("CH3SHV\0"))) {
			INA3221_test_read(INA3221_CH3_SHV);
		} else if (!strcmp_P( strupr(argv[2]), PSTR("CH3BUSV\0"))) {
			INA3221_test_read(INA3221_CH3_BUSV);
		} else if (!strcmp_P( strupr(argv[2]), PSTR("MFID\0"))) {
			INA3221_test_read(INA3221_MFID);
		} else if (!strcmp_P( strupr(argv[2]), PSTR("DIEID\0"))) {
			INA3221_test_read(INA3221_DIEID);
		} else {
			pv_snprintfP_ERR();
			return;
		}
		return;
	}

	// SLEEP
	// read sleep
	if (!strcmp_P( strupr(argv[1]), PSTR("SLEEP\0"))) {
		din = IO_read_SLEEP_CTL();
		snprintf_P( cmd_printfBuff, sizeof(cmd_printfBuff),PSTR( "Sleep_pin=%d\r\n\0"),din);
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;
	}

	// DIGITAL IN
	// read d0p,d0l,d1p,d1l
	if (!strcmp_P( strupr(argv[1]), PSTR("D0L\0"))) {
		din = IO_read_D0L();
		snprintf_P( cmd_printfBuff, sizeof(cmd_printfBuff),PSTR( "D0L=%d\r\n\0"),din);
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;
	}
	if (!strcmp_P( strupr(argv[1]), PSTR("D0P\0"))) {
		din = IO_read_D0P();
		snprintf_P( cmd_printfBuff, sizeof(cmd_printfBuff),PSTR( "D0P=%d\r\n\0"),din);
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;
	}
	if (!strcmp_P( strupr(argv[1]), PSTR("D1L\0"))) {
		din = IO_read_D1L();
		snprintf_P( cmd_printfBuff, sizeof(cmd_printfBuff),PSTR( "D1L=%d\r\n\0"),din);
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;
	}
	if (!strcmp_P( strupr(argv[1]), PSTR("D1P\0"))) {
		din = IO_read_D1P();
		snprintf_P( cmd_printfBuff, sizeof(cmd_printfBuff),PSTR( "D1P=%d\r\n\0"),din);
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;
	}

	// OUT
	// read out {0|1}
	if (!strcmp_P( strupr(argv[1]), PSTR("OUT\0"))) {

		if ( atoi(argv[2]) == 0 ) {
			out = DRV8830_read( DRV8830_A_ADDR , DRV8830_FAULT, &out );
			snprintf_P( cmd_printfBuff, sizeof(cmd_printfBuff),PSTR( "OUT_0=0x%02x\r\n\0"),out);
			CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
			return;
		}

		if ( atoi(argv[2]) == 1 ) {
			out = DRV8830_read( DRV8830_B_ADDR , DRV8830_FAULT, &out );
			snprintf_P( cmd_printfBuff, sizeof(cmd_printfBuff),PSTR( "OUT_1=0x%02x\r\n\0"),out);
			CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
			return;
		}
		pv_snprintfP_ERR();
		return;
	}

	// CMD NOT FOUND
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\nCMD NOT DEFINED\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;

}
/*------------------------------------------------------------------------------------*/
static void cmdClearScreen(void)
{
	// ESC [ 2 J
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\x1B[2J\0"));
	CMD_write(cmd_printfBuff, sizeof(cmd_printfBuff) );
}
/*------------------------------------------------------------------------------------*/
static void cmdHelpFunction(void)
{

	memset( &cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\nSpymovil %s %s %s %s\r\n\0"), SP6K_MODELO, SP6K_VERSION, SP6K_REV, SP6K_DATE);
	CMD_write(cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Available commands are:\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-cls\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-reset\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-help\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-status\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-write\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  ee {pos}{string}\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  spi wr {pos}{string}\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  spi setcs relcs enable disable erase\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  spi perase {page}\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  nvmee {pos}{string}\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  rtc {YYMMDDhhmm}\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  memvcc on|off\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  btvcc,an3,sens12 on|off\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  ina {conf}\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  clrd {0|1}\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  gprs pwr|sw {on|off}\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  atcmd cmd\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	/*
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  out set {0|1} {value}\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  out clear {0|1}\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	*/
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  out sleep|reset|phase(A/B)|enable(A/B) {0|1}\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  out pulse (A/B) (+/-) (ms)\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  out ctl12v on|off\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-read\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  ee {pos}{lenght}\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  spi rd {pos}{lenght}, status\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  rtc,id\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  ina {conf|chxshv|chxbusv|mfid|dieid}\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  d0l,d0p,d1l,d1p\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  out {0|1}\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  sleep\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  gprs\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

}
/*------------------------------------------------------------------------------------*/
static void pv_snprintfP_OK(void )
{
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ok\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
}
/*------------------------------------------------------------------------------------*/
static void pv_snprintfP_ERR(void)
{
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("error\r\n\0"));
	CMD_write(  cmd_printfBuff, sizeof(cmd_printfBuff) );
}
/*------------------------------------------------------------------------------------*/
