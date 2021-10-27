//#############################################################################
//
// FILE: receiver.c
//
//! The example demonstrates digital data transfer between two C28x devices
//! just via single wire. The following project file is for receiver
//! (F28004x) which utilizes High resolution Capture (HRCAP) to decode the
//! data and the transmitter (F2838x) encodes it utilizing High Resolution
//! PWM signal. The transmitter example uses internal ADC to sample external
//! signal and the digitally converted data is sent to the receiver. For better
//! understanding the system functionality the user can set "USE_ADC_INPUT" to
//! 0, in this mode the user can manually write to the "dig_value_input" using
//! the expressions window and can observe whether the written data is received
//! correctly at receiver. If "USE_ADC_INPUT" = 0 then the sampled external data
//! is ignored and the manual user-written data is used for transfer.
//!
//! The example supports various user-configurable parameters :
//!
//! At Transmitter :
//!  BIT_RESOLUTION -> The resolution at which user wants to encode/decode data,
//!              please refer to performance table shown below to choose the
//!              appropriate resolution based on ENOB (Effective Number of Bits) 
//!              and latency requirements. By default, resolution is set to 11-bit
//!  ADC_NORM -> Based on selected BIT_RESOLUTION, the user needs to define
//!               the scale factor as (1 / 2^(BIT_RESOLUTION))
//!  PWMCLK -> The EPWM module clock at which user wants to operate, choose
//!             either 100Mhz/200Mhz depending upon max supported by particular
//!            device, by default its set to 100Mhz
//!  PWM_FREQ -> The PWM signal frequency used for transmission, please refer
//!              to performance table shown below to choose the appropriate PWM
//!              transmission frequency based on ENOB and latency requirements.
//!              By default, it is set to 200Khz
//!  ADC_SAMPLE_FREQ -> The ADC trigger frequency in Khz, by default its set to
//!                     25Khz. The maximum programmable value of ADC_SAMPLE_FREQ
//!                     is equal to "PWM_FREQ"
//!  USE_ADC_INPUT -> Set this to 1 if external signal need to be used for data
//!                   transfer. By default its set to 0 i.e. manual mode
//!
//! At Receiver :
//!  BIT_RESOLUTION -> The resolution at which user wants to encode/decode data,
//!              please refer to performance table shown below to choose the
//!              appropriate resolution based on ENOB and latency requirements.
//!              By default, resolution is set to 11-bit
//!  PWM_FREQ -> The PWM signal frequency used for transmission, please refer
//!              to performance table shown below to choose the appropriate PWM
//!              transmission frequency based on ENOB and latency requirements.
//!              By default, it is set to 200Khz
//!
//!  The performance table in terms of ENOB achieved and latency at different
//!  transmission frequencies are shown below :
//!
//!    Transmission PWM frequency        ENOB                Latency
//!        100 Khz                       11.3                 11.2 us
//!        200 Khz                        11                  6.2 us
//!        500 Khz                       10.4                 3.2 us
//!        800 Khz                       9.83                 2.45 us
//!
//!
//! \b External \b Connections \n
//! - GPIO0 (EPWM1A) of F2838x Control card-> GPIO2 (eCAP input) of F28004x card
//! - GND of F2838x -> GND of F28004x
//! - External Sinusoid Signal (0 to 1.5V, 1Khz) ->
//!                                      ADCA1 (Pin 11 on F2838x Control Card)
//! Note : For reliable transmission of data, the amplitude of external signal
//!        should be in accordance to the chosen "BIT_RESOLUTION". For example,
//!        the default resolution in this example is 11-bit thus the
//!        corresponding maximum analog voltage that could be transferred is
//!        (2^11 / 2^12) * 3.3V = 1.65V as 12-bit ADC resolution mode is used
//!
//!        Thus provide 0 to 1.5V, 1Khz sinusoid signal to evaluate the system
//!        with default configuration
//!
//!
//! \b Watch \b Variables \n
//!
//! At Transmitter :
//! - \b dig_value_input - Digital data to be transferred between device,
//!                        In case of manual mode i.e. USE_ADC_INPUT = 0 update
//!                         its value in the Expression window
//!
//!
//! At Receiver
//! - \b dig_value_output - Decoded digital value using HRCAP
//! - \b dig_output_buff[500] - Continuous output buffer of size 500 to plot
//!                             the signal with time. Please use Graph option
//!                             of CCS to plot this
//!
//!
//!
//!  NOTE : The receiver decoded raw value has constant offset thus one-time
//!         calibration is required. The example is already provided with some
//!         pre-calibrated offset value in receiver code. But in case the user
//!         observes some offset at receiver, an option calibration project is
//!         also provided at C2000Ware_x_x_x\demo examples\
//!         hrcap_hrpwm_data_transfer\internal_loopback to find the offset
//!         value. The offset value observed from the calibration project
//!         can be used to program "offset" variable in receiver code
//!
//!
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

//
// Example configuration parameters
//
#define BIT_RESOLUTION 11
#define PWM_FREQ   200U

//
// Globals
//
float32_t offset = 0.00016f;
float32_t duty_offset = 0.0f;
volatile float32_t duty_output = 0.0f, duty_output_minus_offset = 0.0f, dig_value_output_norm = 0.0f;
volatile uint16_t dig_value_output = 0, dig_output_buff[500] = {0}, count = 0;
volatile uint32_t cap1Count = 0, cap2Count = 0, cap3Count = 0;
volatile uint32_t absCountOn1 = 0, absCountPeriod1 = 0, ecapIntCalCount = 0;

//
// Function Prototypes
//
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
    // Initialize GPIOs for ECAP
    //
    initGPIO();

    //
    // Interrupts that are used in this example are re-mapped to ISR functions
    // found within this file.
    //
    Interrupt_register(INT_ECAP6, &ecap6ISR);


    //
    // Configure eCAP
    //
    configECAP();

    //
    // Configure HRCAP
    //
    HRCAP_enableHighResolutionClock(HRCAP6_BASE);

    //
    // Add small delay for HRCLK to start
    //
    __asm(" RPT #25 || NOP");

    //
    // Enable high resolution capture
    //
    HRCAP_enableHighResolution(HRCAP6_BASE);

    //
    // Calculating the offset in duty based on the offset value
    // at 100 Khz
    //
    duty_offset = offset * ((float)PWM_FREQ / 100.0f);
    
    //
    // Enable interrupts ECAP interrupt
    //
    Interrupt_enable(INT_ECAP6);

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

    while(1)
    {


    } // end infinite for loop
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
         // Removing constant offset
         //
         duty_output_minus_offset = duty_output - duty_offset;

         //
         // Decoding normalized output (i.e. between 0 to 1) using duty value
         // For duty value 'x', the decoded normalized output will be
         // (x-0.1)/0.8 i.e. (x-1)*1.25
         //
         dig_value_output_norm = (duty_output_minus_offset - 0.1f) * 1.25f;

         //
         // Scaling the normalized output with the desired bit resolution
         //
         dig_value_output = (uint16_t)(dig_value_output_norm * (uint16_t)(1 << BIT_RESOLUTION)
                                           + 0.5f);

         //
         //  Saving in a continuous output buffer of 500
         //
         if (count < 500)
             dig_output_buff[count++] = dig_value_output;
         else count = 0;

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


