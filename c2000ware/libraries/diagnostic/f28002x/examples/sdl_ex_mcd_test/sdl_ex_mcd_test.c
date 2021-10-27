//#############################################################################
//
// FILE:   sdl_ex_mcd_test.c
//
// TITLE:  Missing clock detection
//
//! \defgroup sdl_ex_mcd_test Missing clock detection
//! <h1>sdl_ex_mcd_test</h1>
//!
//! This example demonstrates how to check the missing clock detection
//! functionality.
//!
//! Once the MCD is simulated by disconnecting the clock to the MCD module
//! an NMI is generated. This NMI determines that an MCD error was generated
//! due to a clock failure which is handled in the ISR.
//!
//! Before the MCD the clock frequency will be as per device initialization.
//! Post MCD error, the clock will be running at the speed of INTOSC1 with the
//! PLL disabled (10 MHz).
//!
//! The example also shows how to lock the PLL after missing clock detection,
//! by first explicitly switching the clock source to INTOSC1, resetting the
//! missing clock detect circuit and then re-locking the PLL. After that the
//! clock frequency will be restored to the original configuration.
//!
//! \b External \b Connections \n
//!  - None.
//!
//! \b Watch \b Variables \n
//!  - \b fail - Indicates that a missing clock was either not detected or was
//!       not handled correctly.
//!  - \b nmiISRFlag - Indicates that the missing clock failure caused an
//!       NMI to be triggered and called the ISR to handle it.
//!  - \b mcdDetect - Indicates that a missing clock was detected.
//!  - \b result - Status of a successful handling of missing clock detection.
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
bool mcdDetect = false;
volatile bool nmiISRFlag = false;
uint16_t nmiStatus = 0U;
uint32_t result = FAIL;

//
// Function Prototypes
//
__interrupt void nmiISR(void);

//
// Main
//
void main(void)
{
    uint32_t fail = 0U;

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
    // Re-map NMI signal to call the ISR function in this example.
    //
    SysCtl_clearAllNMIFlags();
    Interrupt_register(INT_NMI, &nmiISR);

    //
    // Enabling the NMI global interrupt (typically already enabled by boot ROM
    // or GEL file).
    //
    SysCtl_enableNMIGlobalInterrupt();

    //
    // Enable Global Interrupt (INTM) and Real Time interrupt (DBGM).
    //
    EINT;
    ERTM;

    //
    // Check the PLL System clock before the Missing clock detection
    // is simulated. It should be same as that set by Device_init().
    //
    if(SysCtl_getClock(DEVICE_OSCSRC_FREQ) != DEVICE_SYSCLK_FREQ)
    {
        fail++;
    }

    //
    // Enable the missing clock detection (MCD) Logic as a precaution.
    // The is continuously active unless the MCD was disabled previously.
    //
    SysCtl_enableMCD();

    //
    // Simulate a missing clock.
    //
    SysCtl_disconnectMCDClockSource();

    //
    // Wait till the NMI is fired on clock failure.
    //
    while(nmiISRFlag != true);

    //
    // A missing clock was detected.
    //
    if(mcdDetect != true)
    {
        fail++;
    }

    //
    // Check if the NMI triggered was due to a clock failure.
    //
    if((nmiStatus & SYSCTL_NMI_CLOCKFAIL) != SYSCTL_NMI_CLOCKFAIL)
    {
        fail++;
    }

    //
    // Check the PLL System clock after the Missing clock detection
    // is simulated. It should be equal to INTOSC1(10Mhz).
    //
    if(SysCtl_getClock(DEVICE_OSCSRC_FREQ) != SYSCTL_DEFAULT_OSC_FREQ)
    {
        fail++;
    }

    //
    // To lock the PLL after missing clock detection, we first explicitly
    // switch the clock source to INTOSC1, reset the missing clock detect
    // circuit, and then re-lock the PLL.
    //
    // Configure oscillator source to INTOSC1.
    //
    SysCtl_selectOscSource(SYSCTL_OSCSRC_OSC1);

    //
    // Re-connect missing clock detection to clock source to stop simulating
    // clock failure.
    //
    SysCtl_connectMCDClockSource();

    //
    // Reset the missing clock detection logic after clock failure.
    //
    SysCtl_resetMCD();

    //
    // Set up PLL control and clock dividers using the original clock source.
    //
    SysCtl_setClock(DEVICE_SETCLOCK_CFG);

    //
    // Check the system clock after the missing clock detection is cleared
    // and handled using INTOSC1 as clock source.
    //
    if(SysCtl_getClock(DEVICE_OSCSRC_FREQ) != DEVICE_SYSCLK_FREQ)
    {
        fail++;
    }

    //
    // Status of a successful handling of missing clock detection.
    //
    if(fail != 0U)
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
//           is generated on clock failure detection.
//
__interrupt void nmiISR(void)
{
    nmiISRFlag = true;
    mcdDetect = SysCtl_isMCDClockFailureDetected();
    nmiStatus = SysCtl_getNMIFlagStatus();
    SysCtl_clearAllNMIFlags();
}

//
// End of File
//
