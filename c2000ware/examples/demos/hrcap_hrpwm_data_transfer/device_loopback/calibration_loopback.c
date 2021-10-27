//#############################################################################
//
// FILE: calibration_loopback.c
//
//!
//! The receiver decoded raw value has constant offset thus one-time
//! calibration is required. Include this file for build and exclude
//! "device_loopback_example.c" and run the example. Add "offset to"
//! the watch window and observe its value once "offset_cal_flag" sets
//! to 1. Use this offset value to program "offset" variable in
//! "device_loopback_example.c" for data transfer on same device
//!
//!
//! \b External \b Connections \n
//! - Connect GPIO0 (EPWM1A) to GPIO2 (eCAP input) on F28004x Control card
//!
//! \b Watch \b Variables \n
//! - \b offset : Record this value once calibration flag gets set
//! - \b offset_cal_flag : wait till this flag is "1"
//#############################################################################
// $TI Release: v3.04.00.00 $
// $Release Date: 2020 $
// $Copyright:
// Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
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

//
// Configuring transmission frequency as 100Khz
//
#define PWMCLK     100U
#define PWM_FREQ   100U

//
//  SFO calibration flags
//
#define STATUS_SUCCESS    1
#define STATUS_FAIL       0

//
// Globals
//
float32_t offset = 0.0f;
uint16_t status;
volatile float32_t dig_value_input_norm = 0.0f, duty = 0.3656249f;
volatile float32_t duty_output = 0.0f, average = 0.0f;
volatile uint16_t  count = 0, offset_cal_flag = 0;
volatile uint32_t cap1Count = 0, cap2Count = 0, cap3Count = 0;
volatile uint32_t absCountOn1 = 0, absCountPeriod1 = 0, ecapIntCalCount = 0;
uint16_t time_period = 0;

//
// Global variable used by the SFO library
// Result can be used for all HRPWM channels
// This variable is also copied to HRMSTEP
// register by SFO() function.
//
int MEP_ScaleFactor;

//
// Used by SFO library (ePWM[0] is a dummy value that isn't used)
//
volatile uint32_t ePWM[2] = {0, EPWM1_BASE};

//
// Function Prototypes
//
void error(void);
void initEPWM1GPIO(void);
void initEPWMModule(uint32_t base, uint32_t period);
void initHRPWMModule(uint32_t base, uint32_t period);
void configDuty (void);
void initGPIO(void);
void configECAP(void);
__interrupt void ecap6ISR(void);

//
// Main
//
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
    // Initialize PIE and clear PIE registers.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // Interrupts that are used in this example are re-mapped to ISR functions
    // found within this file.
    //
    Interrupt_register(INT_ECAP6, &ecap6ISR);

    //
    // Configure ePWM1 GPIOs
    //
    initEPWM1GPIO();

    //
    // Initialize GPIOs for ECAP
    //
    initGPIO();

    //
    // Configure eCAP
    //
    configECAP();

    //
    // Enable HRCAP clock
    //
    HRCAP_enableHighResolutionClock(HRCAP6_BASE);

    //
    // Add small delay for HRCLK to start
    //
    __asm(" RPT #25 || NOP");

    //
    // Enable High resolution capture
    //
    HRCAP_enableHighResolution(HRCAP6_BASE);

    //
    // Calling SFO() updates the HRMSTEP register with calibrated
    // MEP_ScaleFactor. HRMSTEP must be populated with a scale factor value
    // prior to enabling high resolution period control.
    //
    status = SFO_INCOMPLETE;
    while(status == SFO_INCOMPLETE)
    {
        status = SFO();
        if(status == SFO_ERROR)
        {
            error();   // SFO function returns 2 if an error occurs & # of MEP
        }              // steps/coarse step exceeds maximum of 255.
    }

    //
    // Specifying SFO calibration flag as incomplete for next round
    //
    status = SFO_INCOMPLETE;


    //
    // Calculating Time_period based on PWM module clock and PWM transmission
    // frequency
    //
    time_period = (PWMCLK / (float)PWM_FREQ) * 1000U;

    //
    // ePWM and HRPWM register initialization
    //
    initHRPWMModule(EPWM1_BASE, time_period);

    //
    // Enable ECAP interrupt
    //
    Interrupt_enable(INT_ECAP6);

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;


    //
    // Update duty based on digital input value
    //
    configDuty();

    while(1)
    {


    } // end infinite for loop
}

//
// Function for updating HRPWM duty
//
void configDuty (void) {

    //
    // Writing the fixed point converted version of duty value to the CMPA
    // register as automatic conversion mode of HRPWM is used
    //
    HWREG(EPWM1_BASE + HRPWM_O_CMPA) = (uint32_t)((duty*(float32_t)time_period)
                                          * (float32_t)((uint32_t)1 << 16) +
                                                 0.5f);
}

//
// initHRPWMModule - Configure ePWM1
//
void initHRPWMModule(uint32_t base, uint32_t period)
{
    //
    // Configuring EPWM module with specified period value
    //
    initEPWMModule(base, period);

    //
    // Initialize HRPWM extension.
    //
    HWREGH(base + HRPWM_O_CMPA) = (1U << 8U);
    HRPWM_setCounterCompareShadowLoadEvent(base, HRPWM_CHANNEL_A,
                                               HRPWM_LOAD_ON_CNTR_ZERO);

    HWREG(base + HRPWM_O_CMPB) |= (1U << 8U);
    HRPWM_setCounterCompareShadowLoadEvent(base, HRPWM_CHANNEL_B,
                                               HRPWM_LOAD_ON_CNTR_ZERO);

    //
    // Configure MEP edge & control mode for channel A & B. MEP Edge control is
    // on falling edge. Control mode is duty control.
    //
    HRPWM_setMEPEdgeSelect(base, HRPWM_CHANNEL_A, HRPWM_MEP_CTRL_FALLING_EDGE);
    HRPWM_setMEPControlMode(base, HRPWM_CHANNEL_A, HRPWM_MEP_DUTY_PERIOD_CTRL);


    HRPWM_setMEPEdgeSelect(base, HRPWM_CHANNEL_B, HRPWM_MEP_CTRL_FALLING_EDGE);
    HRPWM_setMEPControlMode(base, HRPWM_CHANNEL_B, HRPWM_MEP_DUTY_PERIOD_CTRL);

    //
    //Enable Automatic Conversion mode
    //
     HRPWM_enableAutoConversion(base);

    //
    // Disable high-resolution period control.
    //
    HRPWM_disablePeriodControl(base);
}

//
// EPWM configuration
//
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

    //
    // Set time base counter value to zero.
    //
    EPWM_setTimeBaseCounter(base, 0U);

    //
    // Set emulation mode to free run.
    //
    EPWM_setEmulationMode(base, EPWM_EMULATION_FREE_RUN);

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
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    //
    // Action for ePWM1B output. Set output to high when TBCTR = 0.
    // Set output to low when TBCTR = CMPB value.
    //
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
}

//
// initEPWM1GPIO - Configure ePWM GPIO
//
void initEPWM1GPIO(void)
{
    //
    // Disable pull up on GPIO 0 and GPIO 1 and configure them as PWM1A and
    // PWM1B output respectively.
    //
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_0_EPWM1A);

    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_1_EPWM1B);
}

//
// error - Halt debugger when called
//
void error(void)
{
    ESTOP0;         // Stop here and handle error
}

//
// initGPIO - Configure GPIO2 as the eCAP input
//
void initGPIO(void)
{
    //
    // Configure GPIO2 as the eCAP input
    //
    XBAR_setInputPin(XBAR_INPUT7, 2);
    GPIO_setPinConfig(GPIO_2_GPIO2);
    GPIO_setDirectionMode(2, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(2, GPIO_QUAL_ASYNC);
}

//
// configECAP - Configure eCAP 6
//
void configECAP()
{
    //
    // Disable, clear all capture flags and interrupts
    //
    ECAP_disableInterrupt(ECAP6_BASE,
                          (ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |
                           ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |
                           ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |
                           ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |
                           ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                           ECAP_ISR_SOURCE_COUNTER_PERIOD   |
                           ECAP_ISR_SOURCE_COUNTER_COMPARE));
    ECAP_clearInterrupt(ECAP6_BASE,
                        (ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |
                         ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                         ECAP_ISR_SOURCE_COUNTER_PERIOD   |
                         ECAP_ISR_SOURCE_COUNTER_COMPARE));

    //
    // Disable CAP1-CAP4 register loads
    //
    ECAP_disableTimeStampCapture(ECAP6_BASE);

    //
    // Make sure the counter is stopped
    //
    ECAP_stopCounter(ECAP6_BASE);

    //
    // Capture mode
    //
    ECAP_enableCaptureMode(ECAP6_BASE);

    //
    // Continuous mode, stop at 3 events
    //
    ECAP_setCaptureMode(ECAP6_BASE,
                        ECAP_CONTINUOUS_CAPTURE_MODE,
                                ECAP_EVENT_3);
    //
    // Event 1, Event 3 rising edge. Event 2 falling edge
    //
    ECAP_setEventPolarity(ECAP6_BASE,
                          ECAP_EVENT_1,
                          ECAP_EVNT_RISING_EDGE);
    ECAP_setEventPolarity(ECAP6_BASE,
                          ECAP_EVENT_2,
                          ECAP_EVNT_FALLING_EDGE);
    ECAP_setEventPolarity(ECAP6_BASE,
                          ECAP_EVENT_3,
                          ECAP_EVNT_RISING_EDGE);

    //
    // Disabling counter reset on any of the event
    //
    ECAP_disableCounterResetOnEvent(ECAP6_BASE, ECAP_EVENT_1);
    ECAP_disableCounterResetOnEvent(ECAP6_BASE, ECAP_EVENT_2);
    ECAP_disableCounterResetOnEvent(ECAP6_BASE, ECAP_EVENT_3);

    //
    // Select input
    //
    ECAP_selectECAPInput(ECAP6_BASE, ECAP_INPUT_INPUTXBAR7);

    //
    //  Enable load counter
    //
    ECAP_enableLoadCounter(ECAP6_BASE);

    //
    // Set SYNCO
    //
    ECAP_setSyncOutMode(ECAP6_BASE, ECAP_SYNC_OUT_SYNCI);

    //
    // Enable capture units
    //
    ECAP_enableTimeStampCapture(ECAP6_BASE);

    //
    // Start counter
    //
    ECAP_startCounter(ECAP6_BASE);

    //
    // Reset counters and re-arm module
    //
    ECAP_resetCounters(ECAP6_BASE);
    ECAP_reArm(ECAP6_BASE);

    //
    // Enable CAP1-CAP3 register loads
    //
    ECAP_enableTimeStampCapture(ECAP6_BASE);

    //
    // 3 events = 1 interrupt
    //
    ECAP_enableInterrupt(ECAP6_BASE, ECAP_ISR_SOURCE_CAPTURE_EVENT_3);

}

//
// ecap6ISR -  eCAP 6 ISR
//
__interrupt void ecap6ISR(void)
{
    ecapIntCalCount++;
    if(ecapIntCalCount > 10)
    {
         //
         // Get the raw time stamps
         //
         cap1Count = HWREG(ECAP6_BASE + ECAP_O_CAP1);
         cap2Count = HWREG(ECAP6_BASE + ECAP_O_CAP2);
         cap3Count = HWREG(ECAP6_BASE + ECAP_O_CAP3);

         //
         // Reset the ECAP counters
         //
         EALLOW;
         HWREGH(ECAP6_BASE + ECAP_O_ECCTL2) |= ECAP_ECCTL2_CTRFILTRESET;
         EDIS;

         //
         // Relative measurements
         //
         absCountOn1 = cap2Count - cap1Count;
         absCountPeriod1 = cap3Count - cap1Count;

         //
         // Calculating duty value
         //
         duty_output = __divf32((float32_t)absCountOn1, (float32_t)absCountPeriod1);

         //
         // Calculating average measured duty for 500 values
         // Subtracting it from input duty to calculate offset
         //
         if (offset_cal_flag == 0)
         {
             if (count < 500)
             {
                 average += duty_output;
                 count++;
             }
             else
             {
                 average = average / 500.0f;
                 offset = average - duty;
                 offset_cal_flag = 1;
             }
         }

    }

    //
    // Clearing the interrupt event flags
    //
    HWREGH(ECAP6_BASE + ECAP_O_ECCLR) |= ECAP_ISR_SOURCE_CAPTURE_EVENT_3;
    HWREGH(ECAP6_BASE + ECAP_O_ECCLR) |= ECAP_ECCLR_INT;

    //
    // Acknowledge the PIE interrupt group
    //
    HWREGH(PIECTRL_BASE + PIE_O_ACK) = INTERRUPT_ACK_GROUP4;

}

//
// End of file
//


