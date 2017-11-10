/*
 * l_iopines.h
 *
 *  Created on: 7 de jul. de 2017
 *      Author: pablo
 */

#include <avr/io.h>

#ifndef SRC_SP6KLIBS_L_IOPINES_H_
#define SRC_SP6KLIBS_L_IOPINES_H_

#define PORT_SetPinAsOutput( _port, _bitPosition ) ( (_port)->DIR = (_port)->DIR | (1 << _bitPosition) )
#define PORT_SetPinAsInput( _port, _bitPosition ) ( (_port)->DIR = (_port)->DIR & ~(1 << _bitPosition) )

#define PORT_SetOutputBit( _port, _bitPosition ) ( (_port)->OUT = (_port)->OUT | (1 << _bitPosition) )
#define PORT_ClearOutputBit( _port, _bitPosition ) ( (_port)->OUT = (_port)->OUT & ~(1 << _bitPosition) )

#define PORT_GetBitValue( _port , _bitPosition ) ( (((_port)->IN) >> _bitPosition ) & 0x01 )

#define PORT_TogglePins( _port, _toggleMask ) ( (_port)->OUTTGL = _toggleMask )

void PORT_ConfigureInterrupt0( PORT_t * port,PORT_INT0LVL_t intLevel,uint8_t pinMask);
void PORT_ConfigureInterrupt1( PORT_t * port,PORT_INT1LVL_t intLevel, uint8_t pinMask);
//------------------------------------------------------------------------------------
#define SENS_12V_CTL_BITPOS	0
#define SENS_12V_CTL_PORT	PORTB

#define IO_config_SENS_12V_CTL()	PORT_SetPinAsOutput( &SENS_12V_CTL_PORT, SENS_12V_CTL_BITPOS)
#define IO_set_SENS_12V_CTL()		PORT_SetOutputBit( &SENS_12V_CTL_PORT, SENS_12V_CTL_BITPOS)
#define IO_clr_SENS_12V_CTL()		PORT_ClearOutputBit( &SENS_12V_CTL_PORT, SENS_12V_CTL_BITPOS)
//void IO_config_SENS_12V_CTL(void);
//void IO_set_SENS_12V_CTL(void);
//void IO_clr_SENS_12V_CTL(void);
//------------------------------------------------------------------------------------
// Control de PWR de INA

#define AN_3V3_CTL_BITPOS	0
#define AN_3V3_CTL_PORT		PORTA
#define AN_3V3_CTL_MASK		0x01

#define IO_config_AN_3V3_CTL()	PORT_SetPinAsOutput( &AN_3V3_CTL_PORT, AN_3V3_CTL_BITPOS)
#define IO_set_AN_3V3_CTL()		PORT_SetOutputBit( &AN_3V3_CTL_PORT, AN_3V3_CTL_BITPOS)
#define IO_clr_AN_3V3_CTL()		PORT_ClearOutputBit( &AN_3V3_CTL_PORT, AN_3V3_CTL_BITPOS)
#define IO_toggle_AN_3V3_CTL() 	PORT_TogglePins(&AN_3V3_CTL_PORT, AN_3V3_CTL_MASK )

//void IO_config_AN_3V3_CTL(void);
//void IO_set_AN_3V3_CTL(void);
//void IO_clr_AN_3V3_CTL(void);

//------------------------------------------------------------------------------------
// MAIN POWER SLEEP MODE

#define PWR_SLEEP_BITPOS	1
#define PWR_SLEEP_PORT	PORTA

#define IO_config_PWR_SLEEP()	PORT_SetPinAsOutput( &PWR_SLEEP_PORT, PWR_SLEEP_BITPOS)
#define IO_set_PWR_SLEEP()		PORT_SetOutputBit( &PWR_SLEEP_PORT, PWR_SLEEP_BITPOS)
#define IO_clr_PWR_SLEEP()		PORT_ClearOutputBit( &PWR_SLEEP_PORT, PWR_SLEEP_BITPOS)

//------------------------------------------------------------------------------------
// LED

#define LED1_BITPOS	2
#define LED1_PORT	PORTA

#define IO_config_LED1()	PORT_SetPinAsOutput( &LED1_PORT, LED1_BITPOS)
#define IO_set_LED1()		PORT_SetOutputBit( &LED1_PORT, LED1_BITPOS)
#define IO_clr_LED1()		PORT_ClearOutputBit( &LED1_PORT, LED1_BITPOS)

#define LED_PA0_BITPOS	0
#define LED_PA0_PORT	PORTA

#define IO_config_LED_PA0()		PORT_SetPinAsOutput( &LED_PA0_PORT, LED_PA0_BITPOS)
#define IO_set_LED_PA0()		PORT_SetOutputBit( &LED_PA0_PORT, LED_PA0_BITPOS)
#define IO_clr_LED_PA0()		PORT_ClearOutputBit( &LED_PA0_PORT, LED_PA0_BITPOS)

//------------------------------------------------------------------------------------
// BLUETOOTH POWER CTL

#define BT_PWR_BITPOS	5
#define BT_PWR_PORT		PORTD

#define IO_config_BT_PWR_CTL()	PORT_SetPinAsOutput( &BT_PWR_PORT, BT_PWR_BITPOS)
#define IO_set_BT_PWR_CTL()		PORT_SetOutputBit( &BT_PWR_PORT, BT_PWR_BITPOS)
#define IO_clr_BT_PWR_CTL()		PORT_ClearOutputBit( &BT_PWR_PORT, BT_PWR_BITPOS)
//void IO_config_BT_PWR_CTL(void);
//void IO_set_BT_PWR_CTL(void);
//void IO_clr_BT_PWR_CTL(void);

//------------------------------------------------------------------------------------
// I2C DEVICES POWER

#define MEM_VCC_BITPOS	7
#define MEM_VCC_PORT	PORTD

#define IO_config_MEM_VCC()		PORT_SetPinAsOutput( &MEM_VCC_PORT, MEM_VCC_BITPOS)
#define IO_set_MEM_VCC()		PORT_SetOutputBit( &MEM_VCC_PORT, MEM_VCC_BITPOS)
#define IO_clr_MEM_VCC() 		PORT_ClearOutputBit( &MEM_VCC_PORT, MEM_VCC_BITPOS)
//void IO_config_MEM_VCC(void);
//void IO_set_MEM_VCC(void);
//void IO_clr_MEM_VCC(void);

//------------------------------------------------------------------------------------
// FLASH MEMORY SELECT

#define SPIMEM_CS_BITPOS		4
#define SPIMEM_CS_PORT			PORTC
#define IO_config_SPIMEM_CS()	PORT_SetPinAsOutput( &SPIMEM_CS_PORT, SPIMEM_CS_BITPOS)
#define IO_set_SPIMEM_CS()		PORT_SetOutputBit( &SPIMEM_CS_PORT, SPIMEM_CS_BITPOS)
#define IO_clr_SPIMEM_CS()		PORT_ClearOutputBit( &SPIMEM_CS_PORT, SPIMEM_CS_BITPOS)

#define SPIMEM_MOSI_BITPOS		5
#define SPIMEM_MOSI_PORT		PORTC
#define IO_config_SPIMEM_MOSI()	PORT_SetPinAsOutput( &SPIMEM_MOSI_PORT, SPIMEM_MOSI_BITPOS)
#define IO_set_SPIMEM_MOSI()	PORT_SetOutputBit( &SPIMEM_MOSI_PORT, SPIMEM_MOSI_BITPOS)
#define IO_clr_SPIMEM_MOSI()	PORT_ClearOutputBit( &SPIMEM_MOSI_PORT, SPIMEM_MOSI_BITPOS)

#define SPIMEM_SCK_BITPOS		7
#define SPIMEM_SCK_PORT			PORTC
#define IO_config_SPIMEM_SCK()	PORT_SetPinAsOutput( &SPIMEM_SCK_PORT, SPIMEM_SCK_BITPOS)
#define IO_set_SPIMEM_SCK()		PORT_SetOutputBit( &SPIMEM_SCK_PORT, SPIMEM_SCK_BITPOS)
#define IO_clr_SPIMEM_SCK()		PORT_ClearOutputBit( &SPIMEM_SCK_PORT, SPIMEM_SCK_BITPOS)

//void IO_config_SPIMEM_CS(void);
//void IO_set_SPIMEM_CS(void);
//void IO_clr_SPIMEM_CS(void);

//------------------------------------------------------------------------------------
// INTERFASE DIGITAL

#define CLR_D_BITPOS	3
#define CLR_D_PORT		PORTA

#define D1L_BITPOS		4
#define D1L_PORT		PORTA

#define D1P_BITPOS		5
#define D1P_PORT		PORTA
//#define D1P_BITPOS		2
//#define D1P_PORT		PORTB

#define D0L_BITPOS		6
#define D0L_PORT		PORTA

#define D0P_BITPOS		7
#define D0P_PORT		PORTA

#define IO_config_DOP()		PORT_SetPinAsInput( &D0P_PORT, D0P_BITPOS)
#define IO_config_DOL()		PORT_SetPinAsInput( &D0L_PORT, D0L_BITPOS)
#define IO_config_D1P()		PORT_SetPinAsInput( &D1P_PORT, D1P_BITPOS)
#define IO_config_D1L()		PORT_SetPinAsInput( &D1L_PORT, D1L_BITPOS)
#define IO_config_CLRD()	PORT_SetPinAsOutput( &CLR_D_PORT, CLR_D_BITPOS)
#define IO_set_CLRD()		PORT_SetOutputBit( &CLR_D_PORT, CLR_D_BITPOS)
#define IO_clr_CLRD()		PORT_ClearOutputBit( &CLR_D_PORT, CLR_D_BITPOS)
//void IO_config_DOP(void);
//void IO_config_DOL(void);
//void IO_config_D1P(void);
//void IO_config_D1L(void);
//void IO_config_CLRD(void);
//void IO_set_CLRD(void);
//void IO_clr_CLRD(void);
uint8_t IO_read_D0L(void);
uint8_t IO_read_D0P(void);
uint8_t IO_read_D1L(void);
uint8_t IO_read_D1P(void);
//------------------------------------------------------------------------------------
// TERMINAL CONTROL PIN

#define TERMCTL_PIN_BITPOS		1
#define TERMCTL_PIN_PORT		PORTA

#define IO_config_TERMCTL_PIN()		PORT_SetPinAsInput( &TERMCTL_PIN_PORT, TERMCTL_PIN_BITPOS)
uint8_t IO_read_TERMCTL_PIN(void);
//------------------------------------------------------------------------------------
// Control de Frecuencia del TICK

#define TICK_MONITOR_BITPOS		3
#define TICK_MONITOR_PORT		PORTA
#define TICK_MONITOR_MASK		0x04

#define IO_config_TICK_MONITOR()	PORT_SetPinAsOutput( &TICK_MONITOR_PORT, TICK_MONITOR_BITPOS)
#define IO_set_TICK_MONITOR()		PORT_SetOutputBit( &TICK_MONITOR_PORT, TICK_MONITOR_BITPOS)
#define IO_clr_TICK_MONITOR()		PORT_ClearOutputBit( &TICK_MONITOR_PORT, TICK_MONITOR_BITPOS)
#define IO_toggle_TICK_MONITOR() 	PORT_TogglePins(&TICK_MONITOR_PORT, TICK_MONITOR_MASK )
//------------------------------------------------------------------------------------
// OUTPUTS 8814 CONTROL

#define OUT_12V_CTL_BITPOS			1
#define OUT_12V_CTL_PORT			PORTB
#define IO_config_OUT_12V_CTL()		PORT_SetPinAsOutput( & OUT_12V_CTL_PORT,  OUT_12V_CTL_BITPOS)
#define IO_set_OUT_12V_CTL()		PORT_SetOutputBit( & OUT_12V_CTL_PORT,  OUT_12V_CTL_BITPOS)
#define IO_clr_OUT_12V_CTL()		PORT_ClearOutputBit( & OUT_12V_CTL_PORT,  OUT_12V_CTL_BITPOS)

#define PHB_BITPOS			1
#define PHB_PORT			PORTA
#define IO_config_PHB()		PORT_SetPinAsOutput( &PHB_PORT, PHB_BITPOS)
#define IO_set_PHB()		PORT_SetOutputBit( &PHB_PORT, PHB_BITPOS)
#define IO_clr_PHB()		PORT_ClearOutputBit( &PHB_PORT, PHB_BITPOS)

#define ENB_BITPOS			3
#define ENB_PORT			PORTB
#define IO_config_ENB()		PORT_SetPinAsOutput( &ENB_PORT, ENB_BITPOS)
#define IO_set_ENB()		PORT_SetOutputBit( &ENB_PORT, ENB_BITPOS)
#define IO_clr_ENB()		PORT_ClearOutputBit( &ENB_PORT, ENB_BITPOS)

#define ENA_BITPOS			4
#define ENA_PORT			PORTB
#define IO_config_ENA()		PORT_SetPinAsOutput( &ENA_PORT, ENA_BITPOS)
#define IO_set_ENA()		PORT_SetOutputBit( &ENA_PORT, ENA_BITPOS)
#define IO_clr_ENA()		PORT_ClearOutputBit( &ENA_PORT, ENA_BITPOS)

#define PHA_BITPOS			5
#define PHA_PORT			PORTB
#define IO_config_PHA()		PORT_SetPinAsOutput( &PHA_PORT, PHA_BITPOS)
#define IO_set_PHA()		PORT_SetOutputBit( &PHA_PORT, PHA_BITPOS)
#define IO_clr_PHA()		PORT_ClearOutputBit( &PHA_PORT, PHA_BITPOS)

#define SLP_BITPOS			6
#define SLP_PORT			PORTB
#define IO_config_SLP()		PORT_SetPinAsOutput( &SLP_PORT, SLP_BITPOS)
#define IO_set_SLP()		PORT_SetOutputBit( &SLP_PORT, SLP_BITPOS)
#define IO_clr_SLP()		PORT_ClearOutputBit( &SLP_PORT, SLP_BITPOS)

#define RES_BITPOS			7
#define RES_PORT			PORTB
#define IO_config_RES()		PORT_SetPinAsOutput( &RES_PORT, RES_BITPOS)
#define IO_set_RES()		PORT_SetOutputBit( &RES_PORT, RES_BITPOS)
#define IO_clr_RES()		PORT_ClearOutputBit( &RES_PORT, RES_BITPOS)

//------------------------------------------------------------------------------------
#define SLEEP_CTL_BITPOS		0
#define SLEEP_CTL_PORT			PORTB

#define IO_config_SLEEP_CTL()		PORT_SetPinAsInput( &SLEEP_CTL_PORT, SLEEP_CTL_BITPOS)
uint8_t IO_read_SLEEP_CTL(void);
//------------------------------------------------------------------------------------
// GPRS

#define GPRS_SW_BITPOS			4
#define GPRS_SW_PORT			PORTD
#define IO_config_GPRS_SW()		PORT_SetPinAsOutput( &GPRS_SW_PORT, GPRS_SW_BITPOS)
#define IO_set_GPRS_SW()		PORT_SetOutputBit( &GPRS_SW_PORT, GPRS_SW_BITPOS)
#define IO_clr_GPRS_SW()		PORT_ClearOutputBit( &GPRS_SW_PORT, GPRS_SW_BITPOS)

#define GPRS_PWR_BITPOS			5
#define GPRS_PWR_PORT			PORTD
#define IO_config_GPRS_PWR()	PORT_SetPinAsOutput( &GPRS_PWR_PORT, GPRS_PWR_BITPOS)
#define IO_set_GPRS_PWR()		PORT_SetOutputBit( &GPRS_PWR_PORT, GPRS_PWR_BITPOS)
#define IO_clr_GPRS_PWR()		PORT_ClearOutputBit( &GPRS_PWR_PORT, GPRS_PWR_BITPOS)

#define GPRS_RTS_BITPOS			6
#define GPRS_RTS_PORT			PORTD
#define IO_config_GPRS_RTS()	PORT_SetPinAsOutput( &GPRS_RTS_PORT, GPRS_RTS_BITPOS)
#define IO_set_GPRS_RTS()		PORT_SetOutputBit( &GPRS_RTS_PORT, GPRS_RTS_BITPOS)
#define IO_clr_GPRS_RTS()		PORT_ClearOutputBit( &GPRS_RTS_PORT, GPRS_RTS_BITPOS)

#define GPRS_CTS_BITPOS			6
#define GPRS_CTS_PORT			PORTF
#define IO_config_GPRS_CTS()	PORT_SetPinAsInput( &GPRS_CTS_PORT, GPRS_CTS_BITPOS)

#define GPRS_DCD_BITPOS			5
#define GPRS_DCD_PORT			PORTE
#define IO_config_GPRS_DCD()	PORT_SetPinAsInput( &GPRS_DCD_PORT, GPRS_DCD_BITPOS)

#define GPRS_RI_BITPOS			4
#define GPRS_RI_PORT			PORTE
#define IO_config_GPRS_RI()		PORT_SetPinAsInput( &GPRS_RI_PORT, GPRS_RI_BITPOS)

#define GPRS_RX_BITPOS			2
#define GPRS_RX_PORT			PORTE
#define IO_config_GPRS_RX()		PORT_SetPinAsInput( &GPRS_RX_PORT, GPRS_RX_BITPOS)

#define GPRS_TX_BITPOS			3
#define GPRS_TX_PORT			PORTE
#define IO_config_GPRS_TX()		PORT_SetPinAsOutput( &GPRS_TX_PORT, GPRS_TX_BITPOS)
#define IO_set_GPRS_TX()		PORT_SetOutputBit( &GPRS_TX_PORT, GPRS_TX_BITPOS)
#define IO_clr_GPRS_TX()		PORT_ClearOutputBit( &GPRS_TX_PORT, GPRS_TX_BITPOS)

//------------------------------------------------------------------------------------
#endif /* SRC_SP6KLIBS_L_IOPINES_H_ */
