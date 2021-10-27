//###########################################################################
//  This software is licensed for use with Texas Instruments C28x
//  family Microcontrollers.  This license was provided to you prior to installing
//  the software.  You may review this license by consulting a copy of
//  the agreement in the doc directory of this library.
// ------------------------------------------------------------------------
//          Copyright (C) 2015 Texas Instruments, Incorporated.
//                          All Rights Reserved.
// ==========================================================================
//
// FILE:   PM_tformat_Include.h
//
// TITLE:  Prototypes and Definitions for the Position Manager Tamagawa-T Library
//
//###########################################################################
//#############################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:27 CDT 2021 $
// $Copyright:
// Copyright (C) 2017-2020 Texas Instruments Incorporated
//
//     http://www.ti.com/ ALL RIGHTS RESERVED
// $
//#############################################################################
//Library of TamagawaT functions
#include <stdint.h>

#define PM_TFORMAT_DATAID0	0x0			//0b00000
#define PM_TFORMAT_DATAID1	0x11		//0b10001
#define PM_TFORMAT_DATAID2	0x9			//0b01001
#define PM_TFORMAT_DATAID3	0x18		//0b11000
#define PM_TFORMAT_DATAID6	0xC			//0b01100
#define PM_TFORMAT_DATAIDD	0x17		//0b10111
#define PM_TFORMAT_DATAID7	0x1D		//0b11101
#define PM_TFORMAT_DATAID8	0x3			//0b00011
#define PM_TFORMAT_DATAIDC	0x6			//0b00110

#define PM_TFORMAT_SPI SPIB_BASE

typedef struct  {                                 // bit descriptions
    uint16_t  controlField;
    uint16_t  statusField;
    uint16_t  dataField0;
    uint16_t  dataField1;
    uint16_t  dataField2;
    uint16_t  dataField3;
    uint16_t  dataField4;
    uint16_t  dataField5;
    uint16_t  dataField6;
    uint16_t  dataField7;
    uint16_t  crc;
    uint16_t  eepromAddress;
    uint16_t  eepromWrDtata;
    uint16_t  eepromRdDtata;
    uint32_t  sdata[16];     // Send data buffer
    uint32_t  rdata[16];     // Receive data buffer
    volatile uint16_t  dataReady;
    uint16_t  fifoLevel;
    uint32_t  rxPkts[3];
} PM_tformat_DataStruct; //PM_tformat_DataStruct

extern PM_tformat_DataStruct tformatData;

extern uint16_t PM_tformat_setupCommand(uint16_t dataID, uint16_t eepromAddr, uint16_t eepromData, uint16_t crc);
extern uint16_t PM_tformat_receiveData(uint16_t dataID);
extern void PM_tformat_setupPeriph(uint32_t devLSPCLKFreq);
extern void PM_tformat_setFreq(uint32_t freq_us);
extern void PM_tformat_startOperation(void);

//*****************************************************************************
// CRC related declarations
//*****************************************************************************

#define PM_TFORMAT_NBITS_POLY1      8
#define PM_TFORMAT_POLY1            0x01
#define PM_TFORMAT_CRCTABLE_SIZE    256

//*****************************************************************************
// globals
//*****************************************************************************
extern uint16_t tformatCRCtable[PM_TFORMAT_CRCTABLE_SIZE];

//*****************************************************************************
// prototypes
//*****************************************************************************
extern void PM_tformat_generateCRCTable(uint16_t nBits, uint16_t polynomial, uint16_t *pTable);
extern uint16_t PM_tformat_getCRC(uint16_t inputCRCaccum, uint16_t nBitsData,  uint16_t nBitsPoly, uint16_t * msg, uint16_t *crcTable, uint16_t rxLen);

