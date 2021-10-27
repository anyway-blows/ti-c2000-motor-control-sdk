//#############################################################################
//
// FILE:    hrcap_ex1_capture.c
//
// TITLE:   HRCAP Capture and Calibration Example
//
//! \addtogroup driver_example_list
//! <h1>HRCAP Capture and Calibration Example</h1>
//!
//! This example configures ECAP3 to use HRCAP functionality to capture time
//! between edges on input GPIO2.
//!
//! \b External \b Connections \n
//! The user must provide a signal to GPIO2. XCLKOUT has been configured to
//! GPIO16 and can be externally jumped to serve this purpose.
//!
//! \b Watch \b Variables \n
//! - onTime1, onTime2
//! - offTime1, offTime2
//! - period1, period2
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
#include "device.h"
#include "hrcap_ex1_capture.h"

//
// Globals
//
volatile uint32_t cap1Count;
volatile uint32_t cap2Count;
volatile uint32_t cap3Count;
volatile uint32_t cap4Count;

uint32_t absCountOn1, absCountOn2;
uint32_t absCountOff1, absCountOff2;
uint32_t absCountPeriod1, absCountPeriod2;

float32_t onTime1 = 0, onTime2 = 0;
float32_t offTime1 = 0, offTime2 = 0;
float32_t period1 = 0, period2 = 0;

uint32_t ecapIntCount;
uint16_t hrcapIntCount = 0;
uint16_t ecapIntCalCount = 0;
uint16_t calStatus = 0;

HRCAPCAL_CalResultObj hrcapCalResult;

uint64_t totalCount = 0;
float32_t inputFreqMHz = 0;

//
// Function Prototypes
//
void initGPIO(void);
void initXCLKOUT(void);
void configECAP(void);
void configHRCAP(void);
__interrupt void hrcap3CalISR(void);
__interrupt void ECAP3ISR(void);

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
    // Disable pin locks and enable internal pullups.
    //
    Device_initGPIO();

    //
    // Initialize GPIOs for ECAP, XCLKOUT
    //
    initGPIO();

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
    // Interrupts that are used in this example are re-mapped to ISR functions
    // found within this file.
    //
    Interrupt_register(INT_ECAP3, &ECAP3ISR);
    Interrupt_register(INT_ECAP3_2, &hrcap3CalISR);

    //
    // Initialize XCLKOUT
    //
    initXCLKOUT();

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    //
    // Configure eCAP
    //
    configECAP();

    //
    // HRCAP initialization sequence
    //
    configHRCAP();

    //
    // Enable interrupts required for this example
    //
    Interrupt_enable(INT_ECAP3);
    Interrupt_enable(INT_ECAP3_2);

    //
    // Initialize counters
    //
    ecapIntCount = 0;
    cap2Count = 0U;
    cap3Count = 0U;
    cap4Count = 0U;

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    //
    // Enable Global Interrupt (INTM) and Real time interrupt (DBGM)
    //
    EINT;
    ERTM;

    //
    // Loop indefinitely
    //
    while(1)
    {
        //
        // Wait for a value to be captured by the HRCAP
        //
        if (period1 != 0)
        {
            //
            // Convert from nS to MHz
            //
            inputFreqMHz = 1 / (period1 / 1000);
        }

    }
}

//
// initGPIO - Configure GPIO16 for use as XCLKOUT and GPIO2 as the eCAP input
//
void initGPIO(void)
{
    //
    // Configure GPIO16 as XCLKOUT
    //
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_16_XCLKOUT);

    //
    // Configure GPIO2 as the eCAP input
    //
    XBAR_setInputPin(INPUTXBAR_BASE, XBAR_INPUT7, 2);
    GPIO_setPinConfig(GPIO_2_GPIO2);
    GPIO_setDirectionMode(2, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(2, GPIO_QUAL_ASYNC);
}

//
// initXCLKOUT - Configure XCLKOUT
//
void initXCLKOUT(void)
{
    //
    // Clock source is SYSCLK
    //
    SysCtl_selectClockOutSource(SYSCTL_CLOCKOUT_XTALOSC);

    //
    // XCLKOUT = Clock Source / 8
    //
    EALLOW;
    SysCtl_setXClk(SYSCTL_XCLKOUT_DIV_8);
    EDIS;
}

//
// configECAP - Configure eCAP 6
//
void configECAP()
{
    //
    // Disable, clear all capture flags and interrupts
    //
    ECAP_disableInterrupt(ECAP3_BASE,
                          (ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |
                           ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |
                           ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |
                           ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |
                           ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                           ECAP_ISR_SOURCE_COUNTER_PERIOD   |
                           ECAP_ISR_SOURCE_COUNTER_COMPARE));
    ECAP_clearInterrupt(ECAP3_BASE,
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
    ECAP_disableTimeStampCapture(ECAP3_BASE);

    //
    // Make sure the counter is stopped
    //
    ECAP_stopCounter(ECAP3_BASE);

    //
    // Capture mode
    //
    ECAP_enableCaptureMode(ECAP3_BASE);

    //
    // One - shot mode, stop at 4 events
    //
    ECAP_setCaptureMode(ECAP3_BASE,
                        ECAP_ONE_SHOT_CAPTURE_MODE,
                        ECAP_EVENT_4);

    //
    // Event 1, Event 3 falling edge. Event 2, Event 4 rising edge
    //
    ECAP_setEventPolarity(ECAP3_BASE,
                          ECAP_EVENT_1,
                          ECAP_EVNT_RISING_EDGE);
    ECAP_setEventPolarity(ECAP3_BASE,
                          ECAP_EVENT_2,
                          ECAP_EVNT_FALLING_EDGE);
    ECAP_setEventPolarity(ECAP3_BASE,
                          ECAP_EVENT_3,
                          ECAP_EVNT_RISING_EDGE);
    ECAP_setEventPolarity(ECAP3_BASE,
                          ECAP_EVENT_4,
                          ECAP_EVNT_FALLING_EDGE);

    ECAP_disableCounterResetOnEvent(ECAP3_BASE, ECAP_EVENT_1);
    ECAP_disableCounterResetOnEvent(ECAP3_BASE, ECAP_EVENT_2);
    ECAP_disableCounterResetOnEvent(ECAP3_BASE, ECAP_EVENT_3);
    ECAP_disableCounterResetOnEvent(ECAP3_BASE, ECAP_EVENT_4);

    //
    // Select input
    //
    ECAP_selectECAPInput(ECAP3_BASE, ECAP_INPUT_INPUTXBAR7);

    ECAP_enableLoadCounter(ECAP3_BASE);

    //
    // Set SYNCO
    //
    ECAP_setSyncOutMode(ECAP3_BASE, ECAP_SYNC_OUT_SYNCI);

    //
    // Enable capture units
    //
    ECAP_enableTimeStampCapture(ECAP3_BASE);

    //
    // Start counter
    //
    ECAP_startCounter(ECAP3_BASE);

    //
    // Reset counters
    //
    ECAP_resetCounters(ECAP3_BASE);
    ECAP_reArm(ECAP3_BASE);

    //
    // Enable CAP1-CAP4 register loads
    //
    ECAP_enableTimeStampCapture(ECAP3_BASE);

    //
    // 4 events = 1 interrupt
    //
    ECAP_enableInterrupt(ECAP3_BASE, ECAP_ISR_SOURCE_CAPTURE_EVENT_4);
}

//
// configHRCAP - Initialization and calibration sequence for HRCAP
//
void configHRCAP(void)
{
    //
    // Configure calibration period as 1.6ms (recommended)
    //
    HRCAP_setCalibrationPeriod(HRCAP3_BASE, DEVICE_SYSCLK_FREQ);
    
    //
    // Configure in continuous calibration mode
    //
    HRCAP_setCalibrationMode(HRCAP3_BASE);
    
    //
    // Enable High Resolution module
    //
    HRCAP_enableHighResolution(HRCAP3_BASE);
    
    //
    // Enable High resolution clock
    //
    HRCAP_enableHighResolutionClock(HRCAP3_BASE);
    
    //
    // Add small delay for HRCLK to start
    //
    __asm(" RPT #25 || NOP");
    
    //
    // Reset all counters and HR error flags
    //
    ECAP_resetCounters(ECAP3_BASE);

    //
    // Enable HRCAP calibration interrupts
    //
    HRCAP_enableCalibrationInterrupt(HRCAP3_BASE, HRCAP_CALIBRATION_DONE);
    HRCAP_enableCalibrationInterrupt(HRCAP3_BASE,
                                     HRCAP_CALIBRATION_PERIOD_OVERFLOW);

    //
    // Start HR calibration
    //
    HRCAP_startCalibration(HRCAP3_BASE);    
}

//
// ECAP3ISR -  eCAP 6 ISR
//
__interrupt void ECAP3ISR(void)
{
    ecapIntCalCount++;
    if(ecapIntCount++ > 10)
    {
        ecapIntCount = 0;

        //
        // Get the raw time stamps
        //
        cap1Count = ECAP_getEventTimeStamp(ECAP3_BASE, ECAP_EVENT_1);
        cap2Count = ECAP_getEventTimeStamp(ECAP3_BASE, ECAP_EVENT_2);
        cap3Count = ECAP_getEventTimeStamp(ECAP3_BASE, ECAP_EVENT_3);
        cap4Count = ECAP_getEventTimeStamp(ECAP3_BASE, ECAP_EVENT_4);

        absCountOn1 = cap2Count - cap1Count;
        absCountOff1 = cap3Count - cap2Count;
        absCountPeriod1 = cap3Count - cap1Count;

        absCountOn2 = cap4Count - cap3Count;
        absCountOff2 = cap3Count - cap2Count;
        absCountPeriod2 = cap4Count - cap2Count;

        //
        // Convert counts to nanoseconds using the scale factor
        //
        onTime1 = HRCAP_convertEventTimeStampNanoseconds(absCountOn1,
                                                   hrcapCalResult.scaleFactor);
        offTime1 = HRCAP_convertEventTimeStampNanoseconds(absCountOff1,
                                                   hrcapCalResult.scaleFactor);
        period1 = HRCAP_convertEventTimeStampNanoseconds(absCountPeriod1,
                                                   hrcapCalResult.scaleFactor);

        onTime2 = HRCAP_convertEventTimeStampNanoseconds(absCountOn2,
                                                   hrcapCalResult.scaleFactor);
        offTime2 = HRCAP_convertEventTimeStampNanoseconds(absCountOff2,
                                                   hrcapCalResult.scaleFactor);
        period2 = HRCAP_convertEventTimeStampNanoseconds(absCountPeriod2,
                                                   hrcapCalResult.scaleFactor);

        totalCount++;
    }

    ECAP_clearInterrupt(ECAP3_BASE, ECAP_ISR_SOURCE_CAPTURE_EVENT_4);
    ECAP_clearGlobalInterrupt(ECAP3_BASE);
    ECAP_reArm(ECAP3_BASE);
    ECAP_resetCounters(ECAP3_BASE);

    //
    // Acknowledge the PIE interrupt group
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP4);
}

//
// hrcap3CalISR -  eCAP 3 Calibration ISR
//
__interrupt void hrcap3CalISR(void)
{
    hrcapIntCount++;

    //
    // Get calibration interrupt sources
    //
    calStatus = HRCAP_getCalibrationFlags(HRCAP3_BASE);

    //
    // Get calibration clock counts
    //
    hrcapCalResult.hrclkCount = HRCAP_getCalibrationClockPeriod(HRCAP3_BASE,
                                            HRCAP_CALIBRATION_CLOCK_HRCLK);
    hrcapCalResult.sysclkcount = HRCAP_getCalibrationClockPeriod(HRCAP3_BASE,
                                            HRCAP_CALIBRATION_CLOCK_SYSCLK);

    //
    // The following options are possible
    //   - HRCALCAL_STATUS_DONE_ISR
    //   - HRCALCAL_STATUS_DONE_PERIOD_OVERFLOW_ISR
    //   - Software forced generated interrupt
    //
    if(HRCALCAL_STATUS_DONE_ISR == calStatus)
    {
        //
        // Calculate scale factor
        //
        hrcapCalResult.scaleFactor = HRCAP_getScaleFactor(HRCAP3_BASE);
    }
    else if (HRCALCAL_STATUS_DONE_PERIOD_OVERFLOW_ISR == calStatus)
    {
        //
        // Calibration done with an overflow. Determine which counter has
        // overflowed
        //
        if(hrcapCalResult.hrclkCount > hrcapCalResult.sysclkcount)
        {
            //
            // HRCLK has overflowed
            //
            hrcapCalResult.scaleFactor = hrcapCalResult.sysclkcount *
                                         HRCAPCAL_INV_OVERFLOW;
        }
        else if(hrcapCalResult.hrclkCount < hrcapCalResult.sysclkcount)
        {
            //
            // SYSCLK has overflowed
            //
            hrcapCalResult.scaleFactor = HRCAPCAL_OVERFLOW /
                                         hrcapCalResult.hrclkCount;
        }
        else
        {
            //
            // Both SYSCLK and HRCLK have overflowed
            //
            hrcapCalResult.scaleFactor = 1.0f;
        }
    }
    else
    {
        //
        // Software generated interrupt
        //
    }

    //
    // Clear the interrupts
    //
    HRCAP_clearCalibrationFlags(HRCAP3_BASE, calStatus);

    //
    // Acknowledge the PIE interrupt group
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP4);
}
