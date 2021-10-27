//#############################################################################
//
// FILE:   hrpwm_ex5_slider_qformat.c
//
// TITLE:  HRPWM MEP Slider example
//
//! \addtogroup driver_example_list
//! <h1>HRPWM Slider Test</h1>
//!
//! This example modifies the MEP control registers to show edge displacement
//! due to HRPWM. Control blocks of the respective ePWM module channel A and B
//! will have fine edge movement due to HRPWM logic. Load the
//! f2838x_hrpwm_slider.gel file. Select the HRPWM_eval from the GEL menu.
//! A FineDuty slider graphics will show up in CCS. Load the program and run.
//! Use the Slider to and observe the EPWM edge displacement for each slider
//! step change. This explains the MEP control on the EPwmxA channels.
//!
//! Monitor ePWM1 & ePWM2 A/B pins on an oscilloscope.
//
//
//
//#############################################################################
// $TI Release: F2838x Support Library v3.04.00.00 $
// $Release Date: Fri Feb 12 19:08:49 IST 2021 $
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
#include "board.h"

//
// Globals
//
uint32_t  dutySliderVal, update, i, j, n;

//
// Function Prototypes
//
void initHRPWMModule(uint32_t base, uint32_t period);
void initEPWMModule(uint32_t base, uint32_t period);

void main(void)
{
    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Disable pin locks and enable internal pull-ups.
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
    // user specific code
    //
    update = 1;
    dutySliderVal = 0;

    //
    // Configure ePWM1, ePWM2 GPIOs
    //
    Board_init();

    //
    // Disable sync(Freeze clock to PWM as well)
    //
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    //
    // Initialize ePWM1 and ePWM2. For ePWM1, period = 30 & for
    // ePWM2, period = 20
    //
    initHRPWMModule(myEPWM1_BASE, 30U);
    initHRPWMModule(myEPWM2_BASE, 20U);

    //
    // Enable sync and clock to PWM
    //
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    //
    // Enable Global Interrupt (INTM) and real time interrupt (DBGM)
    //
    EINT;
    ERTM;

    while(update == 1U)
    {
        //
        // Write to HRPWM extension of CMPA/ CMPB
        //
        HWREGH(EPWM1_BASE + HRPWM_O_CMPA) =
                (HWREGH(EPWM1_BASE + HRPWM_O_CMPA) & ~(0xFF00)) |
                (dutySliderVal << 8U);
        HWREGH(EPWM1_BASE + HRPWM_O_CMPB) =
                (HWREGH(EPWM1_BASE + HRPWM_O_CMPB) & ~(0xFF00)) |
                (dutySliderVal << 8U);

        //
        // Write to HRPW extension of CMPA/ CMPB
        //
        HWREGH(EPWM2_BASE + HRPWM_O_CMPA) =
                (HWREGH(EPWM2_BASE + HRPWM_O_CMPA) & ~(0xFF00)) |
                (dutySliderVal << 8U);
        HWREGH(EPWM2_BASE + HRPWM_O_CMPB) =
                (HWREGH(EPWM2_BASE + HRPWM_O_CMPB) & ~(0xFF00)) |
                (dutySliderVal << 8U);
    }

    while(1)
    {
    }
}

//
// initHRPWMModule - Configure ePWM1
//
void initHRPWMModule(uint32_t base, uint32_t period)
{
    initEPWMModule(base, period);

    //
    // Initialize HRPWM extension.
    //
    HWREGH(base + HRPWM_O_CMPA) =
            (HWREGH(base + HRPWM_O_CMPA) & ~(0xFF00)) | (1U << 8U);
    HWREGH(base + HRPWM_O_CMPB) =
            (HWREGH(base + HRPWM_O_CMPB) & ~(0xFF00)) | (1U << 8U);


    //
    // Configure HRPWM for channel A & B
    //
    HRPWM_setMEPEdgeSelect(base, HRPWM_CHANNEL_A, HRPWM_MEP_CTRL_FALLING_EDGE);
    HRPWM_setMEPControlMode(base, HRPWM_CHANNEL_A, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setCounterCompareShadowLoadEvent(base, HRPWM_CHANNEL_A,
                                           HRPWM_LOAD_ON_CNTR_ZERO);

    HRPWM_setMEPEdgeSelect(base, HRPWM_CHANNEL_B, HRPWM_MEP_CTRL_FALLING_EDGE);
    HRPWM_setMEPControlMode(base, HRPWM_CHANNEL_B, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setCounterCompareShadowLoadEvent(base, HRPWM_CHANNEL_B,
                                           HRPWM_LOAD_ON_CNTR_ZERO);
}

void initEPWMModule(uint32_t base, uint32_t period)
{
    //
    // Set Immediate Load.
    //
    EPWM_setPeriodLoadMode(base, EPWM_PERIOD_DIRECT_LOAD);

    //
    // Set Period.
    //
    EPWM_setTimeBasePeriod(base, (period - 1U));

    //
    // Set duty cycle of 50% by setting CMPA & CMPB values for EPWM1A & EPWM1B
    // signals respectively.
    //
    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_A, (period / 2));
    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_B, (period / 2));

    //
    // Configure CMPA & CMPB load event & shadow modes.
    //
    EPWM_setCounterCompareShadowLoadMode(base, EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareShadowLoadMode(base, EPWM_COUNTER_COMPARE_B,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set phase shift value to zero & disable phase shift loading & sync
    // output.
    //
    EPWM_setPhaseShift(base, 0U);
    EPWM_disablePhaseShiftLoad(base);
    EPWM_disableSyncOutPulseSource(base, EPWM_SYNC_OUT_PULSE_ON_ALL);

    //
    // Set counter mode to up-counter.
    //
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP);

    //
    // Configure TBCLK. TBCLK = EPWMCLK/(highSpeedPrescaler * pre-scaler)
    //
    EPWM_setClockPrescaler(base, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

    //
    // Configure Action Qualifier
    //
    //
    // Action for ePWMxA output. Set output to high when TBCTR = 0.
    // Set output to low when TBCTR = CMPA value.
    //
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    //
    // Action for ePWMxB output. Set output to high when TBCTR = 0.
    // Set output to low when TBCTR = CMPB value.
    //
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
}
