//#############################################################################
//
// FILE:   sdl_ex_flash_prefetch_test.c
//
// TITLE:  Software Test of Flash Prefetch, Data Cache and Wait-States
//
//! \defgroup sdl_ex_flash_prefetch_test Test of Flash Prefetch, Data Cache and Wait-States
//! <h1>sdl_ex_flash_prefetch_test</h1>
//!
//! This example demonstrates how to test the behavior of the flash prefetch,
//! data cache, and wait-states. Each of these configurations are altered one
//! by one and the impact they have on a test function's execution time is
//! measured. It is expected that an increase in the execution time will be
//! observed in each case.
//!
//! Note that as mentioned in the safety manual description of this diagnostic,
//! a golden value may be used to check the execution time. This wasn't used
//! for this example because of possible compiler differences between users'
//! environments, but can be used in your actual application.
//!
//! \b External \b Connections \n
//!  - None.
//!
//! \b Watch \b Variables \n
//!  - \b baseCycles - Execution time of runPrefetchTestCode();
//!  - \b failCount - Number of tests that returned failures.
//!  - \b result - Status of flash configuration logic test.
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

#include "sta_util.h"

//
// Defines
//
#define PASS                0U
#define FAIL                1U
#define PIPELINE_FLUSH      __asm(" RPT #7 || NOP")

//
// Globals
//
uint32_t result = FAIL;

//
// Function Prototypes
//
uint32_t testBaseline(void);
uint16_t testPrefetch(uint32_t baseCycles);
uint16_t testDataCache(uint32_t baseCycles);
uint16_t testWaitStates(uint32_t baseCycles);

//
// Place functions in RAM since they will be modifying flash configurations.
//
#pragma CODE_SECTION(testBaseline, ".TI.ramfunc")
#pragma CODE_SECTION(testPrefetch, ".TI.ramfunc")
#pragma CODE_SECTION(testDataCache, ".TI.ramfunc")
#pragma CODE_SECTION(testWaitStates, ".TI.ramfunc")
#pragma FUNC_CANNOT_INLINE(testBaseline);
#pragma FUNC_CANNOT_INLINE(testPrefetch);
#pragma FUNC_CANNOT_INLINE(testDataCache);
#pragma FUNC_CANNOT_INLINE(testWaitStates);

//
// This is the function that will be profiled under different flash conditions.
// Its implementation can be found in sdl_ex_flash_prefetch_test_func.asm.
//
extern void runPrefetchTestCode(void);

//
// Main
//
void main(void)
{
    uint16_t failCount = 0U;
    uint32_t baseCycles;

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
    // Enable Global Interrupt (INTM) and Real Time interrupt (DBGM).
    //
    EINT;
    ERTM;

    //
    // Borrowing the STA profiling function for this test. Set up CPU Timer 1.
    //
    STA_Util_configProfiler(CPUTIMER1_BASE);

    //
    // Get the original execution time.
    //
    baseCycles = testBaseline();

    //
    // Alter the flash configurations and check that the execution time
    // increases as a result.
    //
    failCount += testPrefetch(baseCycles);
    failCount += testDataCache(baseCycles);
    failCount += testWaitStates(baseCycles);

    //
    // Status of flash configuration logic test.
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
// testBaseline - Call the test function with the typical flash configurations:
// prefetch and data cache enabled with the recommended number of wait-states
// for the system clock speed.
//
uint32_t testBaseline(void)
{
    uint32_t cycles;

    //
    // Call the test function and capture the execution time.
    //
    STA_Util_startProfiler(CPUTIMER1_BASE);
    runPrefetchTestCode();
    cycles = STA_Util_stopProfiler(CPUTIMER1_BASE);

    return(cycles);
}

//
// testPrefetch - Disable prefetch and confirm that the execution time
//                of the test function increases.
//
uint16_t testPrefetch(uint32_t baseCycles)
{
    uint16_t fail = 1U;
    uint32_t cycles;

    Flash_disablePrefetch(FLASH0CTRL_BASE);

    //
    // Call the test function and capture the execution time.
    //
    STA_Util_startProfiler(CPUTIMER1_BASE);
    runPrefetchTestCode();
    cycles = STA_Util_stopProfiler(CPUTIMER1_BASE);

    Flash_enablePrefetch(FLASH0CTRL_BASE);

    //
    // Confirm that execution time increased.
    //
    if(cycles > baseCycles)
    {
        fail = 0U;
    }

    return(fail);
}

//
// testDataCache - Disable data cache and confirm that the execution time
//                 of the test function increases.
//
uint16_t testDataCache(uint32_t baseCycles)
{
    uint16_t fail = 1U;
    uint32_t cycles;

    Flash_disableCache(FLASH0CTRL_BASE);

    //
    // Call the test function and capture the execution time.
    //
    STA_Util_startProfiler(CPUTIMER1_BASE);
    runPrefetchTestCode();
    cycles = STA_Util_stopProfiler(CPUTIMER1_BASE);

    Flash_enableCache(FLASH0CTRL_BASE);

    //
    // Confirm that execution time increased.
    //
    if(cycles > baseCycles)
    {
        fail = 0U;
    }

    return(fail);
}

//
// testWaitStates - Increase the number of wait-states and confirm that the
//                  execution time of the test function increases.
//
uint16_t testWaitStates(uint32_t baseCycles)
{
    uint16_t fail = 1U;
    uint32_t cycles;

    //
    // Need to disable prefetch and cache before altering wait-states.
    //
    Flash_disablePrefetch(FLASH0CTRL_BASE);
    Flash_disableCache(FLASH0CTRL_BASE);
    Flash_setWaitstates(FLASH0CTRL_BASE, DEVICE_FLASH_WAITSTATES * 2);
    Flash_enablePrefetch(FLASH0CTRL_BASE);
    Flash_enableCache(FLASH0CTRL_BASE);

    PIPELINE_FLUSH;

    //
    // Call the test function and capture the execution time.
    //
    STA_Util_startProfiler(CPUTIMER1_BASE);
    runPrefetchTestCode();
    cycles = STA_Util_stopProfiler(CPUTIMER1_BASE);

    //
    // Set wait-states back to original number.
    //
    Flash_disablePrefetch(FLASH0CTRL_BASE);
    Flash_disableCache(FLASH0CTRL_BASE);
    Flash_setWaitstates(FLASH0CTRL_BASE, DEVICE_FLASH_WAITSTATES);
    Flash_enablePrefetch(FLASH0CTRL_BASE);
    Flash_enableCache(FLASH0CTRL_BASE);

    PIPELINE_FLUSH;

    //
    // Confirm that execution time increased.
    //
    if(cycles > baseCycles)
    {
        fail = 0U;
    }

    return(fail);
}

//
// End of File
//
