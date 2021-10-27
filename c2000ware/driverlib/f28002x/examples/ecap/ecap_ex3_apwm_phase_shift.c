//#############################################################################
//
// FILE:    ecap_ex3_apwm_phase_shift.c
//
// TITLE:   eCAP APWM Phase-shift Example.
//
//! \addtogroup driver_example_list
//! <h1>eCAP APWM Phase-shift Example</h1>
//!
//! This program sets up the eCAP1 and eCAP2 modules in APWM mode to generate
//! the two phase-shifted PWM outputs of same duty and frequency value
//! The frequency, duty and phase values can be programmed of choice by updating
//! the defined macros. By default 10 Khz frequency, 50% duty and 30% phase
//! shift values are used. eCAP2 output leads the eCAP1 output by 30%
//! GPIO14 and GPIO15 are used as eCAP1/2 outputs and can be probed using 
//! analyzer/CRO to observe the waveforms 
//!
//
//#############################################################################
// $TI Release: F28002x Support Library v3.04.00.00 $
// $Release Date: Fri Feb 12 18:58:34 IST 2021 $
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
#define PWM_FREQUENCY        10000U // 10 Khz
#define PWM_DUTY             0.5f   // 50% duty 
#define PWM_PHASE_SHIFT      0.3f   // 30% Phase shift 
#define PWM_PRD_VAL          DEVICE_SYSCLK_FREQ / PWM_FREQUENCY
#define PWM_CMP_VAL          (uint32_t)(PWM_DUTY * PWM_PRD_VAL)
#define PWM_PHASE_VAL        (uint32_t)(PWM_PHASE_SHIFT * PWM_PRD_VAL)

void main(void)
{
    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Disable pin locks and enable internal pull ups.
    //
    Device_initGPIO();

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
    // Select eCAP1OUT on MUX 0. Make GPIO14 as eCAP1OUT for PWM output
    //
    XBAR_setOutputMuxConfig(OUTPUTXBAR_BASE, XBAR_OUTPUT3, 
                            XBAR_OUT_MUX00_ECAP1_OUT);
    XBAR_enableOutputMux(OUTPUTXBAR_BASE, XBAR_OUTPUT3, XBAR_MUX00);
    GPIO_setPadConfig(14, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_14_OUTPUTXBAR3);
    
    //
    // Select eCAP2OUT on MUX 2. Make GPIO15 as eCAP2OUT for PWM output
    //
    XBAR_setOutputMuxConfig(OUTPUTXBAR_BASE, XBAR_OUTPUT4, 
                            XBAR_OUT_MUX02_ECAP2_OUT);
    XBAR_enableOutputMux(OUTPUTXBAR_BASE, XBAR_OUTPUT4, XBAR_MUX02);
    GPIO_setPadConfig(15, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_15_OUTPUTXBAR4);

    //
    // Configure eCAP1 in APWM mode with defined period and duty values
    // Polarity as low and configure SYNCOUT at CTR=PRD
    //
    ECAP_stopCounter(ECAP1_BASE);
    ECAP_enableAPWMMode(ECAP1_BASE);
    ECAP_setAPWMPeriod(ECAP1_BASE, PWM_PRD_VAL);
    ECAP_setAPWMCompare(ECAP1_BASE, PWM_CMP_VAL);
    ECAP_setAPWMPolarity(ECAP1_BASE, ECAP_APWM_ACTIVE_LOW); 
    ECAP_setSyncOutMode(ECAP1_BASE, ECAP_SYNC_OUT_COUNTER_PRD);
    
    //
    // Configured eCAP2 in APWM mode with defined period and duty values
    // Configure the phase shift value as defined and enable the load counter
    // on receiving synchronization pulse from ECAP1
    //
    ECAP_stopCounter(ECAP2_BASE);
    ECAP_enableAPWMMode(ECAP2_BASE);
    ECAP_setAPWMPeriod(ECAP2_BASE, PWM_PRD_VAL);
    ECAP_setAPWMCompare(ECAP2_BASE, PWM_CMP_VAL);
    ECAP_setAPWMPolarity(ECAP2_BASE, ECAP_APWM_ACTIVE_LOW);
    ECAP_setSyncInPulseSource(ECAP2_BASE, ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP1);    
    ECAP_setPhaseShiftCount(ECAP2_BASE, PWM_PHASE_VAL); 
    ECAP_enableLoadCounter(ECAP2_BASE);
    
    //
    // Start counters for both eCAP1/2
    //
    ECAP_startCounter(ECAP1_BASE); 
    ECAP_startCounter(ECAP2_BASE);

    while(1)
    {
        
    }
}

