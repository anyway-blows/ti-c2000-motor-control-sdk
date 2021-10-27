//#############################################################################
//
// FILE:    hrpwm_ex7_prd_updown_sfo_qformat.c
//
// TITLE:   HRPWM SFO V8 High-Resolution Period (Up-Down Count) example
//
//! \addtogroup driver_example_list
//! <h1>HRPWM Period Up-Down Count</h1>
//!
//! This example modifies the MEP control registers to show edge displacement
//! for high-resolution period with ePWM in Up-Down count mode
//! due to the HRPWM control extension of the respective ePWM module.
//!
//! This example calls the following TI's MEP Scale Factor Optimizer (SFO)
//! software library V8 functions:
//!
//! \b int \b SFO(); \n
//! - updates MEP_ScaleFactor dynamically when HRPWM is in use
//! - updates HRMSTEP register (exists only in EPwm1Regs register space)
//!   with MEP_ScaleFactor value
//! - returns 2 if error: MEP_ScaleFactor is greater than maximum value of 255
//!   (Auto-conversion may not function properly under this condition)
//! - returns 1 when complete for the specified channel
//! - returns 0 if not complete for the specified channel
//!
//! This example is intended to explain the HRPWM capabilities. The code can be
//! optimized for code efficiency. Refer to TI's Digital power application
//! examples and TI Digital Power Supply software libraries for details.
//!
//! To run this example:
//! -# Run this example at maximum SYSCLKOUT
//! -# Activate Real time mode
//! -# Run the code
//!
//! \b External \b Connections \n
//!  - Monitor ePWM1/2 A/B pins on an oscilloscope.
//!
//! \b Watch \b Variables \n
//!  - updateFine - Set to 1 use HRPWM capabilities and observe in fine MEP
//!                 steps(default)
//!                 Set to 0 to disable HRPWM capabilities and observe in
//!                 coarse SYSCLKOUT cycle steps
//!
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
#include "sfo_v8.h"
#include "board.h"

//
// Defines
//

// # of PWM channels
#define PWM_CH            2
#define STATUS_SUCCESS    1
#define STATUS_FAIL       0

//
// Globals
//
uint16_t updateFine, periodFine, status;

//
// Global variable used by the SFO library
// Result can be used for all HRPWM channels
// This variable is also copied to HRMSTEP
// register by SFO(0) function.
//
int MEP_ScaleFactor = 0;

//
// Used by SFO library (ePWM[0] is a dummy value that isn't used)
//
volatile uint32_t ePWM[(PWM_CH + 1)] = {0, EPWM1_BASE, EPWM2_BASE};

//
// Function Prototypes
//
void error(void);
void initEPWMModule(uint32_t base, uint32_t period);
void initHRPWMModule(uint32_t base, uint32_t period);

//
// Main
//
void main(void)
{
    uint16_t i;
    uint32_t base;

    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Disable pin locks and enable internal pull-ups.
    //
    Device_initGPIO();

    //
    // Initialize PIE and clear PIE registers.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // Configure ePWM1 and ePWM2 GPIOs
    //
    Board_init();

    //
    // Setup example variables
    //
    updateFine = 1;
    periodFine = 0;
    status = SFO_INCOMPLETE;

    //
    // Calling SFO() updates the HRMSTEP register with calibrated
    // MEP_ScaleFactor. HRMSTEP must be populated with a scale factor
    // value prior to enabling high resolution period control.
    //
    while(status == SFO_INCOMPLETE)
    {
        status = SFO();
        if(status == SFO_ERROR)
        {
            //
            // SFO function returns 2 if an error occurs & # of MEP
            // steps/coarse step exceeds maximum of 255.
            //
            error();
        }
    }

    //
    // EPWM & HRPWM configurations
    //
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    for(i = 0; i < PWM_CH; i++)
    {
        base = EPWM1_BASE + (i * 0x100U);
        initHRPWMModule(base, 20U);
    }

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

    for(;;)
    {
        //
        // Sweep periodFine as a Q16 number from 0.2 - 0.999
        //
        for(periodFine = 0x3333; periodFine < 0xFFBF; periodFine++)
        {
            if(updateFine)
            {
                //
                // Because auto-conversion is enabled, the desired
                // fractional period must be written directly to the
                // TBPRDHR (or TBPRDHRM) register in Q16 format
                // (lower 8-bits are ignored)
                //
                // EPwm1Regs.TBPRDHR = periodFine;
                //
                // The hardware will automatically scale
                // the fractional period by the MEP_ScaleFactor
                // in the HRMSTEP register (which is updated
                // by the SFO calibration software).
                //
                // Hardware conversion:
                // MEP delay movement = ((TBPRDHR(15:0) >> 8) *  HRMSTEP(7:0) +
                //                       0x80) >> 8
                //
                for(i = 0; i < PWM_CH; i++)
                {
                    base = EPWM1_BASE + (i * 0x100U);

                    //
                    // Write fractional period value in Q16 format
                    //
                    HWREGH(base + HRPWM_O_TBPRDHR) = periodFine;
                }
            }
            else
            {
                //
                // No high-resolution movement on TBPRDHR.
                //
                for(i = 0; i < PWM_CH; i++)
                {
                    HWREGH(base + HRPWM_O_TBPRDHR) = 0U;
                }
            }

            //
            // Call the scale factor optimizer lib function SFO(0)
            // periodically to track for any change due to temp/voltage.
            // This function generates MEP_ScaleFactor by running the
            // MEP calibration module in the HRPWM logic. This scale
            // factor can be used for all HRPWM channels. HRMSTEP
            // register is automatically updated by the SFO function.
            // In background, MEP calibration module continuously updates
            // MEP_ScaleFactor
            //
            status = SFO();

            if(status == SFO_ERROR)
            {
                //
                // SFO function returns 2 if an error occurs & # of
                // MEP steps/coarse step exceeds maximum of 255.
                //
                error();
            }
        }
    }
}

//
// initHRPWMModule - Configure HRPWM module
//
void initHRPWMModule(uint32_t base, uint32_t period)
{
    initEPWMModule(base, period);

    //
    // Initialize HRPWM extension
    //
    HWREGH(base + HRPWM_O_CMPA) = (1U << 8U);
    HRPWM_setCounterCompareShadowLoadEvent(base, HRPWM_CHANNEL_A,
                                           HRPWM_LOAD_ON_CNTR_ZERO_PERIOD);

    HWREG(base + HRPWM_O_CMPB) |= (1U << 8U);
    HRPWM_setCounterCompareShadowLoadEvent(base, HRPWM_CHANNEL_B,
                                           HRPWM_LOAD_ON_CNTR_ZERO_PERIOD);

    //
    // Configure MEP edge & control mode for channel A & B. MEP Edge control is
    // on falling edge. Control mode is duty control.
    //
    HRPWM_setMEPEdgeSelect(base, HRPWM_CHANNEL_A,
                           HRPWM_MEP_CTRL_RISING_AND_FALLING_EDGE);
    HRPWM_setMEPControlMode(base, HRPWM_CHANNEL_A, HRPWM_MEP_DUTY_PERIOD_CTRL);


    HRPWM_setMEPEdgeSelect(base, HRPWM_CHANNEL_B,
                           HRPWM_MEP_CTRL_RISING_AND_FALLING_EDGE);
    HRPWM_setMEPControlMode(base, HRPWM_CHANNEL_B, HRPWM_MEP_DUTY_PERIOD_CTRL);

    //
    // Enable auto-conversion logic.
    //
    HRPWM_enableAutoConversion(base);

    //
    // Enable HRPWM phase synchronization on software sync. Required for
    // up-down count HR control.
    //
    HRPWM_enablePhaseShiftLoad(base);

    //
    // Enable high-resolution period control.
    //
    HRPWM_enablePeriodControl(base);

    //
    // Enable TBCLK within EPWM.
    //
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    //
    // Synchronize high resolution phase to start HR period
    //
    EPWM_forceSyncPulse(base);
}

//
// initEPWMModule - Configure EPWM module
//
void initEPWMModule(uint32_t base, uint32_t period)
{
    //
    // Set Shadow Load.
    //
    EPWM_setPeriodLoadMode(base, EPWM_PERIOD_SHADOW_LOAD);

    //
    // Set counter mode to up-counter.
    //
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP_DOWN);

    //
    // Set Period.
    //
    EPWM_setTimeBasePeriod(base, period);

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
    // Set time base counter value to zero.
    //
    EPWM_setTimeBaseCounter(base, 0U);

    //
    // Set emulation mode to free run.
    //
    EPWM_setEmulationMode(base, EPWM_EMULATION_FREE_RUN);

    //
    // Configure TBCLK. TBCLK = EPWMCLK/(highSpeedPrescaler * pre-scaler)
    //
    EPWM_setClockPrescaler(base, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

    //
    // Configure Action Qualifier
    //
    //
    // Enable shadow mode for CMPA & CMPB values.
    //
    EPWM_setActionQualifierShadowLoadMode(base, EPWM_ACTION_QUALIFIER_A,
                                          EPWM_AQ_LOAD_ON_CNTR_ZERO);
    EPWM_setActionQualifierShadowLoadMode(base, EPWM_ACTION_QUALIFIER_B,
                                          EPWM_AQ_LOAD_ON_CNTR_ZERO);

    //
    // Action for ePWM1A output. Set output to high when TBCTR = 0.
    // Set output to low when TBCTR = CMPA value.
    //
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    //
    // Action for ePWM1B output. Set output to high when TBCTR = 0.
    // Set output to low when TBCTR = CMPB value.
    //
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
}


//
// error - Halt debugger when error occurs
//
void error (void)
{
    //
    // Stop here and handle error
    //
    ESTOP0;
}

//
// End of file
//
