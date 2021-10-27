//#############################################################################
//
// FILE: device_loopback_example.c
//
//! The example demonstrates digital data transfer on the same device just
//! via single wire. It establishes a loopback between HRPWM and HRCAP
//! on F28004x which utilizes High resolution PWM (HRPWM) signal to encode the
//! data and the data is decoded by utilizing High Resolution Capture
//! (HRCAP) module. The example uses internal ADC to sample external signal
//! and the digitally converted data is sent to the receiver. For better
//! understanding the system functionality the user can set "USE_ADC_INPUT" to
//! 0, in this mode the user can manually write to the "dig_value_input" using
//! the expressions window and can observe whether the written data is received
//! correctly by loopback. If "USE_ADC_INPUT" = 0 then the sampled external data
//! is ignored and the manual user-written data is used for transfer.
//!
//! The example supports various user-configurable parameters :
//!
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
//!              transmission frequency based on ENOB(Effective Number of Bits) 
//!              and latency requirements. By default, it is set to 200Khz
//!  ADC_SAMPLE_FREQ -> The ADC trigger frequency in Khz, by default its set to
//!                     25Khz. The maximum programmable value of ADC_SAMPLE_FREQ
//!                     is equal to "PWM_FREQ"
//!  USE_ADC_INPUT -> Set this to 1 if external signal need to be used for data
//!                   transfer. By default its set to 0 i.e. manual mode
//!
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
//! - GPIO0 (EPWM1A) -> GPIO2 (eCAP input) of F28004x Control card
//! - External Sinusoid Signal (0 to 1.5V, 1Khz) ->
//!                                      ADCA1 (Pin 11 on F28004x Control Card)
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
//! - \b dig_value_input - Digital data to be transferred between device,
//!                        In case of manual mode i.e. USE_ADC_INPUT = 0 update
//!                         its value in the Expression window
//!
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
//!         observes some offset at receiver, exclude the "internal_loopback_
//!         example.c" and include "calibration_loopback.c" for build. Run it
//!         to find the offset value and then again exclude this file and use
//!         the observed offset value to program "offset" variable in
//!         "internal_loopback_example.c" code
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
#include "sfo_v8.h"

//
// Example configuration parameters
//
#define BIT_RESOLUTION 11
#define ADC_NORM    (float)0.00048828125
#define PWMCLK     100U    // PWM module clock in Mhz
#define PWM_FREQ   200U     // Transmission PWM frequency in Khz
#define ADC_SAMPLE_FREQ 25U  // ADC trigger frequency in Khz
#define USE_ADC_INPUT 0U

//
//  SFO calibration flags
//
#define STATUS_SUCCESS    1
#define STATUS_FAIL       0

//
// Globals
//
float32_t offset = 0.00016f;
float32_t duty_offset = 0.0f;
uint16_t status;
volatile float32_t dig_value_input_norm = 0.0f, duty = 0.0f;
volatile float32_t duty_output = 0.0f, duty_output_minus_offset = 0.0f, dig_value_output_norm = 0.0f;
volatile uint16_t dig_value_input = 680, dig_value_output = 0;
volatile uint16_t dig_output_buff[500] = {0}, count = 0;
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
void initADC(void);
void initADCSOC(void);
void initEPWM3(void);
__interrupt void adcA1ISR(void);
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
    Interrupt_register(INT_ADCA1, &adcA1ISR);
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
    // Calculating the offset in duty based on the offset value
    // at 100 Khz
    //
    duty_offset = offset * ((float)PWM_FREQ / 100.0);

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
    // ADC module and ADCSOC initialization
    // Also initializing EPWM3 as trigger source of ADCSOC
    //
    initADC();
    initADCSOC();
    initEPWM3();

    //
    // Enable ADC and ECAP interrupts
    //
    Interrupt_enable(INT_ADCA1);
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
        while(status == SFO_INCOMPLETE)
            {
                status = SFO();
                if(status == SFO_ERROR)
                {
                    error(); // SFO function returns 2 if an error occurs & #
                             // of MEP steps/coarse step exceeds maximum of 255
                }
                else if (status == SFO_COMPLETE)
                    status = SFO_INCOMPLETE;
            }

    } // end infinite for loop
}

//
// Function for updating HRPWM duty
//
void configDuty (void) {

    //
    // Calculating normalized digital value and then calculating duty in
    // floating point format based on the normalized value. Then writing the
    // fixed point converted version of duty value to the CMPA register as
    // automatic conversion mode of HRPWM is used
    //
    dig_value_input_norm = dig_value_input * ADC_NORM;
    duty = 0.1f + dig_value_input_norm * 0.8f;
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
    GPIO_setPinConfig(GPIO_0_EPWM1_A);

    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_1_EPWM1_B);
}

//
// initADC - Function to configure and power up ADCA.
//
void initADC(void)
{
    //
    // Setup VREF as internal 3.3V
    //
    ADC_setVREF(ADCA_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);

    //
    // Set ADCCLK divider to /4
    //
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);

    //
    // Set pulse positions to late
    //
    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);

    //
    // Power up the ADC and then delay for 1 ms
    //
    ADC_enableConverter(ADCA_BASE);
    DEVICE_DELAY_US(1000);
}

//
// initADCSOC - Function to configure ADCA's SOC0 to be triggered by ePWM1.
//
void initADCSOC(void)
{
    //
    // Configure SOC0 of ADCA to convert pin A0 with a sample window of 10
    // SYSCLK cycles. The EPWM1SOCA signal will be the trigger.
    //
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM3_SOCA,
                 ADC_CH_ADCIN1, 10);

    //
    // Set SOC0 to set the interrupt 1 flag. Enable the interrupt and make
    // sure its flag is cleared.
    //
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER0);
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
}


//
// initEPWM - Function to configure ePWM1 to generate the SOC.
//
void initEPWM3(void)
{
    //
    // Disable SOCA
    //
    EPWM_disableADCTrigger(EPWM3_BASE, EPWM_SOC_A);

    //
    // Configure the SOC to occur on the first up-count event
    //
    EPWM_setADCTriggerSource(EPWM3_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_U_CMPA);
    EPWM_setADCTriggerEventPrescale(EPWM3_BASE, EPWM_SOC_A, 1);

    //
    // Configuring the prescalers of EPWM3 module
    //
    EPWM_setClockPrescaler(EPWM3_BASE, EPWM_CLOCK_DIVIDER_1,
                               EPWM_HSCLOCK_DIVIDER_1);

    //
    // Set the compare A and period values based on the specified sampling
    // frequency
    //
    EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_A,
                                  (PWMCLK / (float)ADC_SAMPLE_FREQ) * 500U);
    EPWM_setTimeBasePeriod(EPWM3_BASE, (PWMCLK / (float)ADC_SAMPLE_FREQ)
                                            * 1000U);
    //
    // Setting counter mode as UP and enabling the trigger
    //
    EPWM_enableADCTrigger(EPWM3_BASE, EPWM_SOC_A);
    EPWM_setTimeBaseCounterMode(EPWM3_BASE, EPWM_COUNTER_MODE_UP);
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
// adcA1ISR - ADC A Interrupt 1 ISR
//
__interrupt void adcA1ISR(void)
{

    //
    // Updating the digital input value if ADC INPUT mode is used
    //
#if USE_ADC_INPUT == 1U
    dig_value_input = ((uint16_t*)(ADCARESULT_BASE + ADC_RESULTx_OFFSET_BASE))
                                                         [ADC_SOC_NUMBER0];
#endif

    //
    // Update duty based on digital input value
    //
    configDuty();

    //
    // Clear the interrupt flag and issue ACK
    //
    HWREGH(ADCA_BASE + ADC_O_INTFLGCLR) |= 1U << (uint16_t)ADC_INT_NUMBER1;
    HWREGH(PIECTRL_BASE + PIE_O_ACK) = INTERRUPT_ACK_GROUP1;

}

//
// End of file
//


