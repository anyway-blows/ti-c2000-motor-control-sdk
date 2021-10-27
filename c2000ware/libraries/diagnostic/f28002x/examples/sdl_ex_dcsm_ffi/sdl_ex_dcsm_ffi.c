//#############################################################################
//
// FILE:   sdl_ex_dcsm_ffi.c
//
// TITLE:  DCSM Freedom From Interference example
//
//! \defgroup sdl_ex_dcsm_ffi Using DCSM for Freedom From Interference (FFI)
//! <h1>sdl_ex_dcsm_ffi</h1>
//!
//! This example demonstrates how to configure and use DCSM and how the DCSM
//! blocks writes from one secured zone to another, simulating the its use to
//! create firewalls between code of different safety requirement criticality.
//!
//! It uses the default passwords to allocate one LSRAM block and one flash
//! sector to zone 1 and another to zone 2. In this example, zoning of memories
//! is done by programming the DCSM OTP. The values that are programmed can be
//! found in in dcsm.asm. Secured RAM blocks are partitioned to contain data
//! arrays used in the example, and secured Flash sectors contain the program
//! code. Once secure, both zone1LockedArray and zone2LockedArray are immutable
//! outside of the safety functions residing in the same zone.
//!
//! !!IMPORTANT!! By default, assignments in dcsm.asm are commented out and
//! the sections mapped to OTP in dcsm.cmd are given the "type = DSECT"
//! attribute to prevent programming of the OTP. In this state, this example is
//! expected to fail (result = FAIL) because the memories won't actually be
//! secured. If you are sure you want to permanently update the zones' link
//! pointers and zone select blocks, uncomment the code in dcsm.asm and remove
//! "type = DSECT" from dcsm.cmd.
//!
//! Parts of this example where generated with assistance from the DCSM Tool.
//! We strongly recommend using this tool to generate the code for your DCSM
//! configuration. Refer to the following guide for more information:
//! http://www.ti.com/lit/pdf/spracp8
//!
//! \note This example assumes you have not written to the DCSM OTP before and
//! that the first zone select block is available for use. If this is not the
//! case, you will need to update the link pointer, zone select block address,
//! and passwords to use the next available zone select block.
//!
//! \b External \b Connections \n
//!  - None.
//!
//! \b Watch \b Variables \n
//!  - \b result - Status indicating success of blocking writes to
//!       secured zones.
//!  - \b errorZone1NotChanged, errorZone2NotChanged, errorZone1Changed,
//!       errorZone2Changed - Count of errors found during execution.
//!  - \b zone1LockedArray - Array located in zone 1 secured memory.
//!  - \b zone2LockedArray - Array located in zone 2 secured memory.
//!
//
//#############################################################################
// $TI Release: C2000 Diagnostic Library v2.01.00 $
// $Release Date: Fri Feb 12 19:23:23 IST 2021 $
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
//#############################################################################

//
// Included Files
//
#include "driverlib.h"
#include "device.h"

//
// Defines
//
#define PASS 0U
#define FAIL 1U

#define ARRAY_SIZE          256U
#define INIT_ARRAY1         0U
#define INIT_ARRAY2         1U
#define Z1_CHANGE_Z1        2U
#define Z1_CHANGE_Z2        3U
#define Z2_CHANGE_Z1        4U
#define Z2_CHANGE_Z2        5U
#define UNSEC_CHANGE_Z1     6U
#define UNSEC_CHANGE_Z2     7U

//
// Globals
//
uint32_t result = FAIL;

uint16_t zone1LockedArray[ARRAY_SIZE];     // Located in zone 1
uint16_t zone2LockedArray[ARRAY_SIZE];     // Located in zone 2

#pragma DATA_SECTION(zone1LockedArray, "ZONE1_RAM");
#pragma DATA_SECTION(zone2LockedArray, "ZONE2_RAM");

//
// Function Prototypes
//
void initArray1(void);          // Initializes array to value, located in Z1
void initArray2(void);          // Initializes array to value, located in Z2
void secureFunction1(void);     // Located in zone 1
void secureFunction2(void);     // Located in zone 2
void unsecureFunction(void);    // Located in unsecured memory
uint16_t checkArray1(uint16_t); // Checks Z1 data against value, located in Z1
uint16_t checkArray2(uint16_t); // Checks Z2 data against value, located in Z2

#pragma CODE_SECTION(initArray1, "ZONE1_CODE");
#pragma CODE_SECTION(initArray2, "ZONE2_CODE");
#pragma CODE_SECTION(secureFunction1, "ZONE1_CODE");
#pragma CODE_SECTION(secureFunction2, "ZONE2_CODE");
#pragma CODE_SECTION(checkArray1, "ZONE1_CODE");
#pragma CODE_SECTION(checkArray2, "ZONE2_CODE");

//
// Main
//
void main(void)
{
    uint16_t errorZone1NotChanged = 0U, errorZone1Changed = 0U;
    uint16_t errorZone2NotChanged = 0U, errorZone2Changed = 0U;

    //
    // Initialize device clock and peripherals.
    //
    Device_init();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // Initialize arrays.
    //
    initArray1();
    initArray2();

    //
    // Make sure both zone 1 and zone 2 are locked
    //
    DCSM_secureZone1();
    DCSM_secureZone2();

    //*************************************************************************
    //
    // Calling secure function 1 - Zone 1 array will be updated. Will attempt
    // to update zone 2 but should be blocked.
    //
    //*************************************************************************
    secureFunction1();

    //
    // Checking array contents from secure code.
    //
    errorZone1NotChanged = checkArray1(Z1_CHANGE_Z1);
    errorZone2Changed = checkArray2(INIT_ARRAY2);

    //*************************************************************************
    //
    // Calling secure function 2 - Zone 2 array will be updated. Will attempt
    // to update zone 1 but should be blocked.
    //
    //*************************************************************************
    secureFunction2();

    //
    // Checking array contents from secure code.
    //
    errorZone1Changed = checkArray1(Z1_CHANGE_Z1);
    errorZone2NotChanged = checkArray2(Z2_CHANGE_Z2);

    //*************************************************************************
    //
    // Calling unsecure function - Will attempt to write to both arrays but
    // should be blocked in both zones.
    //
    //*************************************************************************
    unsecureFunction();

    //
    // Checking array contents from secure code.
    //
    errorZone1Changed += checkArray1(Z1_CHANGE_Z1);
    errorZone2Changed += checkArray2(Z2_CHANGE_Z2);

    //
    // Report whether any data was incorrectly updated.
    //
    if((errorZone1Changed != 0U) || (errorZone1NotChanged != 0U) ||
       (errorZone2Changed != 0U) || (errorZone2NotChanged != 0U))
    {
        result = FAIL;
    }
    else
    {
        result = PASS;
    }

    //
    // Loop here and check results in the CCS Expressions view.
    //
    while(1);
}

//
// initArray1 - Fill the zone 1 array with a known value to start.
//
void initArray1(void)
{
    uint16_t i = 0U;

    for(i = 0U; i < ARRAY_SIZE; i++)
    {
        zone1LockedArray[i] = INIT_ARRAY1;
    }
}

//
// initArray2 - Fill the zone 2 array with a known value to start.
//
void initArray2(void)
{
    uint16_t i = 0U;

    for(i = 0U; i < ARRAY_SIZE; i++)
    {
        zone2LockedArray[i] = INIT_ARRAY2;
    }
}

//
// secureFunction1 - This function resides secure memory grabbed by DCSM
// zone 1. zone1LockedArray is modified whereas writes to zone2LockedArray are
// blocked because it resides zone 2 memory.
//
void secureFunction1(void)
{
    uint16_t i = 0U;

    for(i = 0U; i < ARRAY_SIZE; i++)
    {
        zone1LockedArray[i] = Z1_CHANGE_Z1;
        zone2LockedArray[i] = Z1_CHANGE_Z2;
    }
}

//
// secureFunction2 - This function resides secure memory grabbed by DCSM
// zone 2. zone2LockedArray is modified whereas writes to zone1LockedArray are
// blocked because it resides zone 2 memory.
//
void secureFunction2(void)
{
    uint16_t i = 0U;

    for(i = 0U; i < ARRAY_SIZE; i++)
    {
        zone1LockedArray[i] = Z2_CHANGE_Z1;
        zone2LockedArray[i] = Z2_CHANGE_Z2;
    }
}

//
// unsecureFunction - This function resides in unsecure memory and is blocked
// from writing to either array.
//
void unsecureFunction(void)
{
    uint16_t i = 0U;

    for(i = 0U; i < ARRAY_SIZE; i++)
    {
        zone1LockedArray[i] = UNSEC_CHANGE_Z1;
        zone2LockedArray[i] = UNSEC_CHANGE_Z2;
    }
}

//
// checkArray1 - This function resides in secure memory grabbed by DCSM
// zone 1. It compares zone1LockedArray against a parameter value and returns
// the count of unexpected values.
//
uint16_t checkArray1(uint16_t value)
{
    uint16_t i = 0U;
    uint16_t count = 0U;

    for(i = 0U; i < ARRAY_SIZE; i++)
    {
        if(zone1LockedArray[i] != value)
        {
            count++;
        }
    }

    return(count);
}

//
// checkArray2 - This function resides in secure memory grabbed by DCSM
// zone 2. It compares zone2LockedArray against a parameter value and returns
// the count of unexpected values.
//
uint16_t checkArray2(uint16_t value)
{
    uint16_t i = 0U;
    uint16_t count = 0U;

    for(i = 0U; i < ARRAY_SIZE; i++)
    {
        if(zone2LockedArray[i] != value)
        {
            count++;
        }
    }

    return(count);
}

//
// End of File
//
