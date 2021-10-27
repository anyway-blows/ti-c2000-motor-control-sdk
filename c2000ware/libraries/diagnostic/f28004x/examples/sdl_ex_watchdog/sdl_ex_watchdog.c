//#############################################################################
//
// FILE:   sdl_ex_watchdog.c
//
// TITLE:  Software test of watchdog operation
//
//! \defgroup sdl_ex_watchdog Software test of watchdog operation
//! <h1>sdl_ex_watchdog</h1>
//!
//! This example shows how to use the wakeup interrupt to test the operation of
//! the watchdog. The example generates two watchdog interrupts. The first is
//! caused by delaying the reset of the watchdog too long, causing it to
//! overflow. The second uses the windowing functionality to reset the counter
//! before the minimum threshold can be reached.
//!
//! \b External \b Connections \n
//!  - None.
//!
//! \b Watch \b Variables \n
//!  - \b wakeISRFlag - Indicates the watchdog ISR was executed.
//!  - \b result - Status of successful generation of expected watchdog pulse.
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
bool wakeISRFlag;

//
// Function Prototypes
//
__interrupt void wakeupISR(void);

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
    // Re-map watchdog wake interrupt signal to call the ISR function in this
    // example.
    //
    Interrupt_register(INT_WAKE, &wakeupISR);

    //
    // Clear the test status.
    //
    wakeISRFlag = false;
    failCount = 0U;

    //
    // Set the watchdog to generate an interrupt signal instead of a
    // reset signal.
    //
    SysCtl_setWatchdogMode(SYSCTL_WD_MODE_INTERRUPT);

    //
    // Enable the watchdog wake interrupt signal.
    //
    Interrupt_enable(INT_WAKE);

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM).
    //
    EINT;
    ERTM;

    //
    // Enable the watchdog counter to be reset and then reset it. Note that
    // during run time, it is good practice to write these keys in separate
    // parts of the code but for a start up test this is sufficient.
    //
    EALLOW;
    SysCtl_enableWatchdogReset();
    SysCtl_resetWatchdog();
    EDIS;

    //
    // Set lower boundary of the watchdog window.
    //
    SysCtl_setWatchdogWindowValue(0x20U);

    //
    // Enable the watchdog.
    //
    SysCtl_enableWatchdog();

    //
    // Fail to reset the watchdog in time by delaying until it overflows. The
    // minimum delay was determined by considering the watchdog is an 8-bit
    // timer running off of INTOSC1 (10MHz) with a default pre-divider of /512.
    //
    DEVICE_DELAY_US(13500U);

    //
    // If watchdog interrupt has not occurred by now, increment the fail count.
    //
    if(wakeISRFlag != true)
    {
        failCount++;
    }

    //
    // Test the windowing feature by resetting the watchdog before it has a
    // chance to reach the window threshold.
    //
    wakeISRFlag = false;

    EALLOW;
    SysCtl_enableWatchdogReset();
    SysCtl_resetWatchdog();
    EDIS;

    EALLOW;
    SysCtl_enableWatchdogReset();
    SysCtl_resetWatchdog();
    EDIS;

    //
    // Short delay to allow the interrupt to occur.
    //
    DEVICE_DELAY_US(1U);

    //
    // If watchdog interrupt has not occurred by now, increment the fail count.
    //
    if(wakeISRFlag != true)
    {
        failCount++;
    }

    //
    // Status of successful generation of expected watchdog pulse.
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
// wakeupISR - The interrupt service routine called when the watchdog
//             triggers the wake interrupt signal.
//
__interrupt void
wakeupISR(void)
{
    wakeISRFlag = true;

    //
    // Acknowledge this interrupt located in group 1.
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

//
// End of File
//
