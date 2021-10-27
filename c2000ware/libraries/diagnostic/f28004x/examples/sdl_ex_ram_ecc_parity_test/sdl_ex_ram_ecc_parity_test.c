//#############################################################################
//
// FILE:   sdl_ex_ram_ecc_parity_test.c
//
// TITLE:  Test of parity/ECC logic in SRAM
//
//! \defgroup sdl_ex_ram_ecc_parity_test Test of parity/ECC logic in SRAM
//! <h1>sdl_ex_ram_ecc_parity_test</h1>
//!
//! This example demonstrates how to check the SRAM ECC or parity logic
//! functionality.
//!
//! Three types of SRAM errors are generated to demonstrate the use of the SRAM
//! test modes and the checking of the related flag and information registers.
//! These types are parity errors (LSx, GSx, and CLAMSG RAMs) and correctable
//! and uncorrectable ECC errors (M0 and M1 RAMs).
//!
//! \b External \b Connections \n
//!  - None.
//!
//! \b Watch \b Variables \n
//!  - \b nmiISRFlag - Indicates that the NMI was triggered and called the ISR.
//!  - \b nmiStatus - NMI status flags read in the ISR.
//!  - \b errorISRFlag - Indicates that the correctable RAM error interrupt was
//!       triggered and called the ISR.
//!  - \b errorStatus - Correctable RAM error status flags read in the ISR.
//!  - \b errorCount - Number of correctable errors detected read in the ISR.
//!  - \b errorAddr - Address at which the error was detected.
//!  - \b result - Status of a successful detection and handling of RAM errors.
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
#define PASS    0U
#define FAIL    1U

//
// Globals
//
uint32_t result = FAIL;
volatile bool nmiISRFlag;
uint16_t nmiStatus;
volatile bool errorISRFlag;
uint16_t errorStatus;
uint32_t errorAddr;
uint16_t errorCount;

//
// Reserving memory locations where errors will be injected. Using DATA_SECTION
// to specify which RAM section since some have parity and some have ECC.
//
#pragma DATA_SECTION(gs0Data, "ramgs0");
volatile uint32_t gs0Data = 0xABCDEF00U;
#pragma DATA_SECTION(m0Data, "ramm0");
volatile uint32_t m0Data[2] = {0xAAAA5555U, 0x55005500U};

//
// Function Prototypes
//
__interrupt void nmiISR(void);
__interrupt void corrErrorISR(void);
uint16_t runParityTest(void);
uint16_t runCorrectableECCTest(void);
uint16_t runUncorrectableECCTest(void);

//
// Main
//
void main(void)
{
    uint16_t failCount;

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
    // Clear all the NMI and RAM error status flags.
    //
    MemCfg_clearCorrErrorStatus(MEMCFG_CERR_CPUREAD);
    MemCfg_clearCorrErrorInterruptStatus(MEMCFG_UCERR_CPUREAD);
    MemCfg_clearUncorrErrorStatus(MEMCFG_UCERR_CPUREAD);
    SysCtl_clearAllNMIFlags();

    //
    // Configure the correctable error interrupt threshold to 2 errors.
    //
    MemCfg_setCorrErrorThreshold(2U);

    //
    // Plug the NMI and RAM correctable error ISRs.
    //
    Interrupt_register(INT_NMI, &nmiISR);
    Interrupt_register(INT_RAM_CORR_ERR, &corrErrorISR);

    //
    // Enabling the NMI global interrupt (typically already enabled by boot ROM
    // or GEL file).
    //
    SysCtl_enableNMIGlobalInterrupt();

    //
    // Enable RAM correctable error interrupt.
    //
    Interrupt_enable(INT_RAM_CORR_ERR);
    MemCfg_enableCorrErrorInterrupt(MEMCFG_CERR_CPUREAD);

    //
    // Enable Global Interrupt (INTM) and Real Time interrupt (DBGM).
    //
    EINT;
    ERTM;

    //
    // Test parity functionality by injecting an error in gs0Data.
    //
    failCount = runParityTest();

    //
    // Test ECC functionality by injecting a few correctable errors in m0Data.
    //
    failCount += runCorrectableECCTest();

    //
    // Test ECC functionality by injecting an uncorrectable error in m0Data.
    //
    failCount += runUncorrectableECCTest();

    //
    // Status of a successful handling of the RAM ECC/parity errors.
    //
    if(failCount != 0U)
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
// nmiISR -  The interrupt service routine called when the NMI
//           is generated on an uncorrectable parity/ECC error.
//
__interrupt void nmiISR(void)
{
    //
    // Set a flag indicating the NMI ISR occurred and get the NMI status.
    //
    nmiISRFlag = true;
    nmiStatus = SysCtl_getNMIFlagStatus();

    //
    // Record the address where the error was injected.
    //
    errorAddr = MemCfg_getUncorrErrorAddress(MEMCFG_UCERR_CPUREAD);

    //
    // Clear all the flags.
    //
    MemCfg_clearUncorrErrorStatus(MEMCFG_UCERR_CPUREAD);
    SysCtl_clearAllNMIFlags();
}

//
// corrErrorISR - The interrupt service routine called when the correctable
//                error count hits the configured interrupt threshold.
__interrupt void corrErrorISR(void)
{
    //
    // Set a flag indicating the RAM error ISR occurred.
    //
    errorISRFlag = true;

    //
    // Record the address where the error was injected, the status flags, and
    // the number of single-bit ECC errors detected.
    //
    errorAddr = MemCfg_getCorrErrorAddress(MEMCFG_CERR_CPUREAD);
    errorStatus = MemCfg_getCorrErrorStatus();
    errorCount = MemCfg_getCorrErrorCount();

    //
    // Clear all the flags.
    //
    MemCfg_clearCorrErrorStatus(MEMCFG_CERR_CPUREAD);
    MemCfg_clearCorrErrorInterruptStatus(MEMCFG_CERR_CPUREAD);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP12);
}

//
// runParityTest - Runs a test of the parity logic in GS0RAM.
//
uint16_t runParityTest(void)
{
    uint16_t fail = 0U;
    volatile uint32_t temp;
    uint32_t original = gs0Data;

    //
    // Put the GS0RAM into test mode so we can write directly to the parity,
    // flip a bit, and put the RAM back into the normal functional mode.
    //
    MemCfg_setTestMode(MEMCFG_SECT_GS0, MEMCFG_TEST_WRITE_PARITY);
    gs0Data ^= 0x00000001U;
    MemCfg_setTestMode(MEMCFG_SECT_GS0, MEMCFG_TEST_FUNCTIONAL);

    //
    // Clear error status variables.
    //
    nmiISRFlag = false;
    nmiStatus = 0U;
    errorAddr = 0U;

    //
    // Read the data where the error was injected to trigger the detection.
    //
    temp = gs0Data;

    //
    // Wait until the NMI is fired.
    //
    while(nmiISRFlag != true);

    //
    // Check if the NMI triggered was due to an uncorrectable RAM error.
    //
    if((nmiStatus & SYSCTL_NMI_RAMUNCERR) != SYSCTL_NMI_RAMUNCERR)
    {
        fail++;
    }

    //
    // Check if the error address was gs0Data.
    //
    if(errorAddr != (uint32_t)&gs0Data)
    {
        fail++;
    }

    //
    // Restore gs0Data.
    //
    gs0Data = original;

    return(fail);
}

//
// runCorrectableECCTest - Runs a test of the ECC logic in M0RAM, particularly
//                         triggering correctable errors.
//
uint16_t runCorrectableECCTest(void)
{
    uint16_t fail = 0U;
    volatile uint32_t temp;
    uint32_t original[2];

    original[0] = m0Data[0];
    original[1] = m0Data[1];

    //
    // Put the M0RAM into test mode so we can write directly to the ECC, flip a
    // bit, and put the RAM back into the normal functional mode. Do this for
    // two memory locations since the interrupt threshold was set to 2.
    //
    MemCfg_setTestMode(MEMCFG_SECT_M0, MEMCFG_TEST_WRITE_ECC);
    m0Data[0] ^= 0x00000001U;
    m0Data[1] ^= 0x00000100U;
    MemCfg_setTestMode(MEMCFG_SECT_M0, MEMCFG_TEST_FUNCTIONAL);

    //
    // Clear error status variables.
    //
    errorISRFlag = false;
    errorStatus = 0U;
    errorAddr = 0U;
    errorCount = 0U;

    //
    // Read the data where the errors were injected to trigger the detection.
    //
    temp = m0Data[0];
    temp = m0Data[1];

    //
    // Wait until the error interrupt is fired.
    //
    while(errorISRFlag != true);

    //
    // Check if the appropriate correctable RAM error flag was set.
    //
    if(errorStatus != MEMCFG_CERR_CPUREAD)
    {
        fail++;
    }

    //
    // Check if the error address was m0Data[0].
    //
    if(errorAddr != (uint32_t)&m0Data[0])
    {
        fail++;
    }

    //
    // Check if the error count went up to 2 (the previously configured
    // threshold for generating interrupts).
    //
    if(errorCount != 2U)
    {
        fail++;
    }

    //
    // Restore m0Data.
    //
    m0Data[0] = original[0];
    m0Data[1] = original[1];

    return(fail);
}

//
// runUncorrectableECCTest - Runs a test of the ECC logic in M0RAM,
//                           particularly triggering an uncorrectable error.
//
uint16_t runUncorrectableECCTest(void)
{
    uint16_t fail = 0U;
    volatile uint32_t temp;
    uint32_t original = m0Data[0];

    //
    // Put the M0RAM into test mode so we can write to the data without updating
    // the ECC, flip multiple bits, and put the RAM back into the normal
    // functional mode.
    //
    MemCfg_setTestMode(MEMCFG_SECT_M0, MEMCFG_TEST_WRITE_DATA);
    m0Data[0] ^= 0x00001001U;
    MemCfg_setTestMode(MEMCFG_SECT_M0, MEMCFG_TEST_FUNCTIONAL);

    //
    // Clear error status variables.
    //
    nmiISRFlag = false;
    nmiStatus = 0U;
    errorAddr = 0U;

    //
    // Read the data where the error was injected to trigger the detection.
    //
    temp = m0Data[0];

    //
    // Wait until the error interrupt is fired.
    //
    while(nmiISRFlag != true);

    //
    // Check if the NMI triggered was due to an uncorrectable RAM error.
    //
    if((nmiStatus & SYSCTL_NMI_RAMUNCERR) != SYSCTL_NMI_RAMUNCERR)
    {
        fail++;
    }

    //
    // Check if the error address was m0Data[0].
    //
    if(errorAddr != (uint32_t)&m0Data[0])
    {
        fail++;
    }

    //
    // Restore m0Data[0].
    //
    m0Data[0] = original;

    return(fail);
}

//
// End of File
//
