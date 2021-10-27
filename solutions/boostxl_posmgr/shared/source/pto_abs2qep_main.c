//#############################################################################
//
// FILE:            pto_abs2qep_main.c
//
// Description:     Example project for using the pto_abs2qep Library.
//                  This file is shared across device families:
//                  F2838x, F2827x, F28004x
//
//#############################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:28 CDT 2021 $
// $Copyright:
// Copyright (C) 2017-2021 Texas Instruments Incorporated - http://www.ti.com/
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
#include <stdlib.h>
#include "abs2qep.h"
#include "device.h"
#include "driverlib.h"

//
// Globals used by the position sampling ISR
//
volatile uint16_t ptoDirection;

//
// Globals used for testing the absolute position compared to
// the incremental position measured by the eQEP module
//
volatile uint32_t absolutePositionPrevious = 0;
volatile uint32_t absolutePositionCurrent = 0;
volatile uint32_t absolutePositionNext = 0;


//
// Main
//
__attribute__((ramfunc))
int16_t main(void)
{
    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Disable pin locks and enable internal pull-ups.
    //
    Device_initGPIO();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // Initialization routine for pto_abs2qep operation
    // Configures the peripherals and enables clocks
    // Configures GPIO and XBar as needed
    // Configures the QEP for testing
    // Configures the PWM for sampling the new position
    //
    pto_initAbs2QEP();

    //
    // Enable Global Interrupt (INTM) and real-time interrupt (DBGM)
    //
    EINT;
    ERTM;

    //
    // Start the PWM which will serve as the timer to sample
    // a new absolute position
    //
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    while(1)
    {

    }
}



//
// Timer for position sampling
//
#pragma INTERRUPT (pto_EPWM3ISR, HPI);
__attribute__((ramfunc))
__interrupt void pto_EPWM3ISR(void)
{
    //
    // Visual marker for viewing waveforms
    // GPIO toggles at the start of the PWM ISR
    //
    // During the PTO:
    //  - It will stay low if the direction is reverse
    //  - It will stay high if the direction is forward
    //
    GPIO_togglePin(PTO_ABSQEP_TEST_PIN_1);
    asm("  RPT #10 ||  NOP");
    GPIO_togglePin(PTO_ABSQEP_TEST_PIN_1);

    //
    // Start the PTO
    //
    int32_t incrementalPosition;
    incrementalPosition = EQEP_getPosition(EQEP1_BASE);
    #pragma FORCEINLINE
    pto_abs2qep_runPulseGen(ptoDirection);

    //
    // Test the previous PTO
    //
    // Compare the absolute encoder angle to the incremental encoder angle
    // For the PTO which just completed. The output of the PTO should be
    // connected to an eQEP module as described in the documentation.
    //
    pto_checkPosition(incrementalPosition, absolutePositionPrevious);
    absolutePositionPrevious = absolutePositionCurrent;

    //
    // Generate a new absolute position based on the current position
    //
    absolutePositionNext = pto_generateTestAbsPosition(absolutePositionCurrent);

    //
    // Process the position and and load the HCL FIFO
    // The first time through, the counters will be
    // loaded.
    //
    ptoDirection = pto_abs2qep_translatePosition(absolutePositionNext);

    //
    // When the ISR is entered again, the next position will
    // be current.
    //
    // Clear INT flag for this timer
    // Acknowledge interrupt group
    //
    absolutePositionCurrent = absolutePositionNext;
    EPWM_clearEventTriggerInterruptFlag(EPWM3_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}



