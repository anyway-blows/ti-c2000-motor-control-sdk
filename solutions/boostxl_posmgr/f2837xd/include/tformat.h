//----------------------------------------------------------------------------------
//	FILE:			tformat.h
//
//	Description:	Contains all the initialization, data declarations and setup
//					for tformat encoder interface. This file serves are a template for
//					using PM_tformat Library to interface and incorporates all the encoder
//					and library specific initializations and other important aspects of usage.
//
//	Version: 		1.0
//
//  Target:  		TMS320F28377D,
//
//----------------------------------------------------------------------------------
//  Copyright Texas Instruments 锟� 2004-2015
//----------------------------------------------------------------------------------
//  Revision History:
//----------------------------------------------------------------------------------
//  Date	  | Description / Status
//----------------------------------------------------------------------------------
// Mar 2018  - Example project for PM T-format Library Usage
//----------------------------------------------------------------------------------

#include "PM_tformat_Include.h"
#include "driverlib.h"
#include "device.h"

#define TFORMAT_FREQ_DIVIDER	20
//Clock Frequency = SYSCLK/(4*TFORMAT_FREQ_DIVIDER) ==> (2.5MHz)

extern void tformat_setupGPIO(void);
extern void tformat_configXBAR(void);
extern uint16_t tformat_exCommands(void);
extern uint16_t tformat_checkCRC (uint16_t expectCRC5, uint16_t receiveCRC5);
extern void tformat_configEPWM4(void);

extern interrupt void spiRxFIFOISR(void);
extern void tformat_init(void);
extern void tformat_error();
