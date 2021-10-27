//###########################################################################
//
// FILE:   adc_ex7_ppb_offset.c
//
// TITLE:  ADC offset adjust using post-processing block
//
//! \addtogroup driver_example_list
//! <h1> ADC PPB Offset (adc_ppb_offset)</h1>
//!
//! This example software triggers the ADC.  Some SOCs have automatic offset
//! adjustment applied by the post-processing block. After the program runs,
//! the memory will contain ADC & post-processing block(PPB) results.
//!
//! \b External \b Connections \n
//!  - A2, C2 pins should be connected to signals to convert
//!
//! \b Watch \b Variables \n
//!  - \b adcAResult \b: a digital representation of the voltage on pin A2
//!  - \b adcAPPBResult \b : a digital representation of the voltage
//!      on pin A2, minus 100 LSBs of automatically added offset
//!  - \b adcCResult \b: a digital representation of the voltage on pin C2
//!  - \b adcCPPBResult \b : a digital representation of the voltage
//!      on pin C2 plus 100 LSBs of automatically added offset
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

//
// Included Files
//
#include "driverlib.h"
#include "device.h"


//
// Function Prototypes
//
void configureADC(uint32_t adcBas);
void setupADCSoftware(uint32_t adcBase);
void setupPPBOffset(int16_t aOffset, int16_t bOffset);

//
// Globals
//
uint16_t adcAResult;
uint16_t adcAPPBResult;
uint16_t adcCResult;
uint16_t adcCPPBResult;

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
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // Enable internal reference on ADCs
    //
    ADC_setVREF(ADCA_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setVREF(ADCC_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);

    //
    // Configure the ADCs and power them up
    //
    configureADC(ADCA_BASE);
    configureADC(ADCC_BASE);

    //
    // Setup the ADCs for software conversions
    //
    setupADCSoftware(ADCA_BASE);
    setupADCSoftware(ADCC_BASE);

    //
    // Setup PPB offset correction.
    // conversion on channel A will subtract 100.
    // conversion on channel C will add 100.
    //
    setupPPBOffset(100, -100);

    //
    // Enable global Interrupts and higher priority real-time debug events:
    //
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    //
    // Take conversions indefinitely in loop
    //
    do
    {
        //
        // Convert, wait for completion, and store results.
        // Start conversions immediately via software, ADCA
        //
        ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER0);
        ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER1);

        //
        // Wait for ADCA to complete, then acknowledge flag
        //
        while(ADC_getInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1) == false);
        ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

        //
        // Start conversions immediately via software, ADCC
        //
        ADC_forceSOC(ADCC_BASE, ADC_SOC_NUMBER0);
        ADC_forceSOC(ADCC_BASE, ADC_SOC_NUMBER1);

        //
        // Wait for ADCC to complete, then acknowledge flag
        //
        while(ADC_getInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1) == false);
        ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1);

        //
        // Store results
        //
        adcAResult = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);
        adcAPPBResult = ADC_readPPBResult(ADCARESULT_BASE, ADC_PPB_NUMBER1);
        adcCResult = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER0);
        adcCPPBResult = ADC_readPPBResult(ADCCRESULT_BASE, ADC_PPB_NUMBER1);

        //
        // Software breakpoint, hit run again to get updated conversions
        //
        asm("   ESTOP0");
    }
    while(1);
}

//
// configureADC - Write ADC configurations and power up the ADC for both
//                ADC A and ADC C
//
void configureADC(uint32_t adcBase)
{
    //
    // Set ADCCLK divider to /2
    //
    ADC_setPrescaler(adcBase, ADC_CLK_DIV_2_0);


    //
    // Set pulse positions to late
    //
    ADC_setInterruptPulseMode(adcBase, ADC_PULSE_END_OF_CONV);

    //
    // Power up the ADCs and then delay for 1 ms
    //
    ADC_enableConverter(adcBase);

    //
    // Delay for 1ms to allow ADC time to power up
    //
    DEVICE_DELAY_US(1000);
}

//
// setupADCSoftware - Configure ADC SOC and acquisition window
//
void setupADCSoftware(uint32_t adcBase)
{
    uint16_t acqps;

    //
    // Determine minimum acquisition window (in SYSCLKS)
    //
    acqps = 8; // 80ns
    //
    // - NOTE: A longer sampling window will be required if the ADC driving
    //   source is less than ideal (an ideal source would be a high bandwidth
    //   op-amp with a small series resistance). See TI application report
    //   SPRACT6 for guidance on ADC driver design.
    //

    //
    // Select the channels to convert and end of conversion flag
    // ADCA
    //
    ADC_setupSOC(adcBase, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY,
                 (ADC_Channel)2U, acqps);
    ADC_setupSOC(adcBase, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY,
                 (ADC_Channel)2U, acqps);

    //
    // Configure source as EOC1, clear & enable the interrupt
    //
    ADC_setInterruptSource(adcBase, ADC_INT_NUMBER1, ADC_SOC_NUMBER1);
    ADC_clearInterruptStatus(adcBase, ADC_INT_NUMBER1);
    ADC_enableInterrupt(adcBase, ADC_INT_NUMBER1);
}

//
// setupPPBOffset - Configure PPB for SOC
//
void setupPPBOffset(int16_t aOffset, int16_t bOffset)
{
    //
    // PPB1 is associated with SOC1. PPB1 will subtract OFFCAL value
    // from associated SOC
    //
    // ADCA
    ADC_setupPPB(ADCA_BASE, ADC_PPB_NUMBER1, ADC_SOC_NUMBER1);
    ADC_setPPBCalibrationOffset(ADCA_BASE, ADC_PPB_NUMBER1, aOffset);

    // ADCB
    ADC_setupPPB(ADCC_BASE, ADC_PPB_NUMBER1, ADC_SOC_NUMBER1);
    ADC_setPPBCalibrationOffset(ADCC_BASE, ADC_PPB_NUMBER1, bOffset);
}

//
// End of file
//
