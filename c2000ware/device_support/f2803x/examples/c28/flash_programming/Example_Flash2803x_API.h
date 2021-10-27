//###########################################################################
//
// FILE:  Example_Flash2803x_API.h
//
// TITLE: F2803x Flash API example include file
//
// DESCRIPTION: Function prototypes and macros for the F2803x Flash API 
//              library examples.
//
//###########################################################################
// $TI Release: F2803x Support Library v2.03.00.00 $
// $Release Date: Fri Feb 12 19:14:41 IST 2021 $
// $Copyright:
// Copyright (C) 2009-2021 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef EXAMPLE_FLASH2803x_API_H
#define EXAMPLE_FLASH2803x_API_H

//
// Included Files
//
#include "Flash2803x_API_Library.h"     // flash program files
#include "DSP28x_Project.h"             // device support includes 

//
// Function Prototypes
//
void Example_ToggleTest(Uint16 Pin);   // Run the ToggleTest
Uint16 Example_CsmUnlock(void);        // Unlock the Code Security Module
void Example_CallFlashAPI(void);       // Kernel function interfaces to the API
void Example_Error(Uint16 Status);     // If an error, stop here
void Example_Done(void);               // If done, stop here
void Example_MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, 
                     Uint16* DestAddr);
void CsmPwlDummyRead(void);

//
// External Globals
//
// These symbols are used to copy support functions from Flash to RAM
// These symbols are assigned by the linker.  Refer to the .cmd file
//
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;

//
//  Specify the PLLCR (PLL Control Register) value.
//
//  Uncomment the appropriate line by removing the leading double slash: //
//  Only one statement should be uncommented.
//
//  The user's application must set the PLLCR Register before calling any
//  of the Flash API functions.
//
//  Example:  CLKIN is a 10MHz crystal.
//            The user wants to have a 60Mhz CPU clock (SYSCLKOUT = 60MHz).
//            In this case, PLLCR must be set to 12 (0x000C)
//            Uncomment the line: #define PLLCR_VALUE 0x000C
//            Comment out the remaining lines with a double slash: //
//
#define PLLCR_VALUE  0x000C     // SYSCLKOUT = (OSCLK*12)/2
// #define PLLCR_VALUE   0x000B     // SYSCLKOUT = (OSCLK*11)/2
// #define PLLCR_VALUE   0x000A     // SYSCLKOUT = (OSCLK*10)/2
// #define PLLCR_VALUE   0x0009     // SYSCLKOUT = (OSCLK*9)/2
// #define PLLCR_VALUE   0x0008     // SYSCLKOUT = (OSCLK*8)/2
// #define PLLCR_VALUE   0x0007     // SYSCLKOUT = (OSCLK*7)/2
// #define PLLCR_VALUE   0x0006     // SYSCLKOUT = (OSCLK*6)/2
// #define PLLCR_VALUE   0x0005     // SYSCLKOUT = (OSCLK*5)/2
// #define PLLCR_VALUE   0x0004     // SYSCLKOUT = (OSCLK*4)/2
// #define PLLCR_VALUE   0x0003     // SYSCLKOUT = (OSCLK*3)/2
// #define PLLCR_VALUE   0x0002     // SYSCLKOUT = (OSCLK*2)/2
// #define PLLCR_VALUE   0x0001     // SYSCLKOUT = (OSCLK*1)/2
// #define PLLCR_VALUE   0x0000     // SYSCLKOUT = (OSCLK)/2 (PLL Bypassed)

//
// These key values are used to unlock the CSM by this example
// They are defined in Example_Flash2803x_CsmKeys.asm
//
extern Uint16 PRG_key0;        //   CSM Key values
extern Uint16 PRG_key1;
extern Uint16 PRG_key2;
extern Uint16 PRG_key3;
extern Uint16 PRG_key4;
extern Uint16 PRG_key5;
extern Uint16 PRG_key6;
extern Uint16 PRG_key7;

#endif // ---- End of EXAMPLE_FLASH2803x_API_H

//
// End of File
//
