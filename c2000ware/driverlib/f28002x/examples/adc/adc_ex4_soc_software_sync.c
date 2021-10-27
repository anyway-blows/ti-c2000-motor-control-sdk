//###########################################################################
//
// FILE:   adc_ex4_soc_software_sync.c
//
// TITLE:  ADC synchronous software triggering
//
//! \addtogroup driver_example_list
//! <h1> ADC Synchronous SOC Software Force (adc_soc_software_sync)</h1>
//!
//! This example converts some voltages on ADCA and ADCC using input 5 of the
//! input X-BAR as a software force. Input 5 is triggered by toggling GPIO0,
//! but any spare GPIO could be used. This method will ensure that both ADCs
//! start converting at exactly the same time.
//!
//! \b External \b Connections \n
//!  - A2, A3, C2, C3 pins should be connected to signals to convert
//!
//! \b Watch \b Variables \n
//! - \b adcAResult0 \b: a digital representation of the voltage on pin A2\n
//! - \b adcAResult1 \b: a digital representation of the voltage on pin A3\n
//! - \b adcCResult0 \b: a digital representation of the voltage on pin C2\n
//! - \b adcCResult1 \b: a digital representation of the voltage on pin C3\n
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

//
// Included Files
//
#include "driverlib.h"
#include "device.h"

//
// Function Prototypes
//
void configureADC(uint32_t adcBase);
void setupADCSoftwareSync(uint32_t adcBase);
void setupInputXBAR5(void);


//
// Globals
//
uint16_t adcAResult0;
uint16_t adcAResult1;
uint16_t adcCResult0;
uint16_t adcCResult1;

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
    setupADCSoftwareSync(ADCA_BASE);
    setupADCSoftwareSync(ADCC_BASE);
    setupInputXBAR5();

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
        // Convert, wait for completion, and store results
        //

        //
        // Toggle GPIO0 in software.  This will cause a trigger to
        // both ADCs via input XBAR, line 5.
        //
        GPIO_writePin(0U, 1U); // Set pin
        GPIO_writePin(0U, 0U); // Clear pin

        //
        // Wait for ADCA to complete, then acknowledge the flag.
        // Since both ADCs are running synchronously, it isn't necessary
        // to wait for completion notification from both ADCs
        //
        while(ADC_getInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1) == 0U);
        ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

        //
        // Store results
        //
        adcAResult0 = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);
        adcAResult1 = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER1);
        adcCResult0 = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER0);
        adcCResult1 = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER1);

        //
        // At this point, conversion results are stored in
        // adcAResult0, adcAResult1, adcCResult0, and adcCResult1
        //

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
// setupInputXBAR5 - Setup GPIO 0 to trigger input XBAR line 5.  GPIO0 is used
//                   as an example, but any spare GPIO could be used. The
//                   physical GPIO pin should be allowed to float, since the
//                   code configures it as an output and changes the value.
//
void setupInputXBAR5(void)
{
    //
    // GPIO0 will trigger the input XBAR line 5
    //
    XBAR_setInputPin(INPUTXBAR_BASE, XBAR_INPUT5, 0);

    //
    // GPIO0 as an output
    //
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(0U, GPIO_DIR_MODE_OUT);
    GPIO_setPinConfig(GPIO_0_GPIO0);

    //
    // GPIO0 set as low
    //
    GPIO_writePin(0U, 0U);
}

//
// setupADCSoftwareSync - Setup ADC acquisition window and compare values
//
void setupADCSoftwareSync(uint32_t adcBase)
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
    //
    ADC_setupSOC(adcBase, ADC_SOC_NUMBER0, ADC_TRIGGER_GPIO,
                 ADC_CH_ADCIN2, acqps);
    ADC_setupSOC(adcBase, ADC_SOC_NUMBER1, ADC_TRIGGER_GPIO,
                 ADC_CH_ADCIN3, acqps);

    if(adcBase == ADCA_BASE)
    {
        ADC_setInterruptSource(adcBase, ADC_INT_NUMBER1, ADC_SOC_NUMBER1);
        ADC_enableInterrupt(adcBase, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(adcBase, ADC_INT_NUMBER1);
    }
}

//
// End of file
//
