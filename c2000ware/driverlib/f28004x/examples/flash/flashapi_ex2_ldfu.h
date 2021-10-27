//###########################################################################
//
// FILE:    flashapi_ex2_LDFU.h
//
// TITLE:   Live DFU Header
//
//###########################################################################
// $TI Release: F28004x Support Library v1.12.00.00 $
// $Release Date: Fri Feb 12 18:57:27 IST 2021 $
// $Copyright:
// Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

#ifndef __FLASHAPI_EX2_LDFU_H__
#define __FLASHAPI_EX2_LDFU_H__

//
// Includes
//
#include "flashapi_ex2_bootrom.h"
#include "F021_F28004x_C28x.h"
#include "flashapi_ex2_flash_programming_c28.h" 
#include "flashapi_ex2_erase.h"
#include "flashapi_ex2_verify.h"
#include "driverlib.h"
#include "flashapi_ex2_commands.h"
#include "assert.h"
#include "device.h"

//
// Defines
//

//
// START: Value written to the address of 'B0_START_ADD' or 'B1_START_ADD' after the erase operation is done
//        Indicates that programming/verifying is about to start.
//
#define START                     0x5A5A

//
// KEY: Value written to the address of 'B0_KEY_ADD' or 'B1_KEY_ADD' after programming/verifying is done
//
#define KEY                       0x5B5B5B5B

//
// B0_START_ADD: The address at which the START value is written to for bank 0
//
#define B0_START_ADD              0x82000

//
// B0_KEY_ADD: The address at which the KEY value is written to for bank 0
//
#define B0_KEY_ADD                0x82004

//
// B0_REV_ADD: The address at which the REV value is written to for bank 0
//
#define B0_REV_ADD                0x82006

//
// B0_RESERVED: A program being loaded during an update cannot be written below this adddress inside of bank 0. 
//              0x80000 - 0x82007 is reserved for the flash kernel
//
#define B0_RESERVED               0x82008

//
// B1_START_ADD: The address at which the START value is written to for bank 1
//
#define B1_START_ADD              0x92000

//
// B1_KEY_ADD: The address at which the KEY value is written to for bank 1
//
#define B1_KEY_ADD                0x92004

//
// B1_REV_ADD: The address at which the REV value is written to for bank 1
//
#define B1_REV_ADD                0x92006

//
// B1_RESERVED: A program being loaded during an update cannot be written below this adddress inside of bank 1. 
//              0x90000 - 0x92007 is reserved for the flash kernel
//
#define B1_RESERVED               0x92008

//
// WORDS_IN_FLASH_BUFFER: Specifies the size of the buffer used to write the 
//                        START, KEY, and REV values for bank 1 and bank 0
//
#define  WORDS_IN_FLASH_BUFFER    0x04

//
// Do not change BUFFER_SIZE; BUFFER_SIZE is the size of a buffer
// that stores the bytes of a program received through SCI
//
#define BUFFER_SIZE               0x80

//
// Function Prototypes
//
void liveDFU(void);
uint32_t ldfuLoad(void);
void ldfuCopyData(void);
void bankSelect(void);
void fmstatFail(void);
uint32_t getLongData(void);
void revInit(void);
void readReservedFn(void);
uint16_t findSize(uint32_t address);
extern uint16_t sciaGetWordData(void);
extern uint32_t sciGetFunction(uint32_t  BootMode);
extern void initFlashSectors(void);
extern void exampleError(Fapi_StatusType status);
extern void sciSendChecksum(void);

#endif // __FLASHAPI_EX2_LDFU_H__

//
// End of file
//
