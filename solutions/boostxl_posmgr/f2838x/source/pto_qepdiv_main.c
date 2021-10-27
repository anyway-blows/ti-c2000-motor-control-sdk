//#############################################################################
//
// FILE:           pto_qepdiv_main.c
//
// Description:    Example project for using PM pto_qepdiv Library.
//                 Includes pto_pqepdiv library and corresponding
//                 include files.
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

// Test Usage: This test can be run on the controlCards (F28379D, F280049C)
// refer to PTO library document for details of the connections needed.
// This test uses EPWM module as a way of simulating QEP input for test purposes
// Users can change the PWM frequency to test the operation and also change
// the divider values as needed

//
// Included Files
//
#include "device.h"
#include "driverlib.h"
#include "qepdiv.h"      // Include file for pto_qepdiv interface

//
// Function Prototypes
//
void InitEPwm4Example(void);
void InitEPwm5Example(void);
void InitGPIO_Testing(void);

//
// Defines
//
#define EPWM4_PRD 100
#define EPWM5_PRD 100
#define EPWM4_CMPA 50
#define EPWM4_CMPB 70
#define EPWM5_CMPA 50

//
// Globals
//
uint16_t retval1;
uint16_t divVal = 4, divValnext = 4, indexWidth = 10;
uint16_t pwmperiod = 100; //50000;
uint16_t pwmcmpa = 50; //25000;
uint16_t pwmcmpb = 70; //35000;

//
// Globals
//
void main(void)
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
    // Initialization routine for pto_qepdiv operation - defined in pto_qepdiv.c
    // Configures the peripherals and enables clocks for required modules
    // Configures GPIO and XBar as needed for pto_qepdiv operation
    // Test setup on docking station -
    //  EPWM4A (GPIO6)-> EQEPA (GPIO10)
    //  EPWM5A (GPIO8)-> EQEPB (GPIO11)
    //  EPWM4B (GPIO7)-> Index (GPIO9)
    // These are just for test purposes and do not correspond to real time usage
    //
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    //
    // Test Code - PWMs used as test stimulus to emulate QEP inputs
    //
    InitEPwm4Example();
    InitEPwm5Example();
    InitGPIO_Testing();
    //
    // Test Code - End
    //

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLB1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLB2);

    pto_qepdiv_init();
    SysCtl_delay(800L);                 //Delay 800us


    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    //
    // 2 => indicates the /4, change the number as needed -
    // should be 2^N Ex:/2, /4, /8, /16...
    // Frequency of the output QEPA or QEPB = Frequency of input QEPA or
    // QEPB / (2* divVal)
    // Index output comes when there is raising edge detected on Index input
    // Index output pulse width = indexWidth * (SYSCLK * 2) cycles
    //
    retval1 = pto_qepdiv_config(divVal, indexWidth);
    pto_qepdiv_startOperation(1);

    //
    // Infinite loop
    //
    while(1)
    {
        //
        // Check to see if value of the divider changes. If so, reset the qepdiv
        // parameters. Configure and start the divider using the new vlaue and
        // start operation
        //
        if(divVal != divValnext)
        {
            pto_qepdiv_reset();
            SysCtl_delay(10L);
            divVal = divValnext;
            retval1 = pto_qepdiv_config(divVal, indexWidth);
            pto_qepdiv_startOperation(1);
        }
        __asm(" NOP");
        __asm(" NOP");
        __asm(" NOP");
        __asm(" NOP");
    }
}

//
// InitEPwm4Example - Test code - EPWMs used as stimulus to emulate QEP inputs
// Not needed in customer application
//
void InitEPwm4Example()
{
    //
    // Set-up TBCLK
    //

    //
    // Set timer period 801 TBCLKs
    //
    EPWM_setTimeBasePeriod(EPWM4_BASE, pwmperiod);
    EPWM_setPhaseShift(EPWM4_BASE, 0U);         // Phase is 0
    EPWM_setTimeBaseCounter(EPWM4_BASE, 0U);   // Clear counter

    //
    // Set Compare values (CMPA and CMPB)
    //
    EPWM_setCounterCompareValue(EPWM4_BASE, EPWM_COUNTER_COMPARE_A, pwmcmpa);
    EPWM_setCounterCompareValue(EPWM4_BASE, EPWM_COUNTER_COMPARE_B, pwmcmpb);

    //
    // Setup counter mode - Count up and down
    //
    EPWM_setTimeBaseCounterMode(EPWM4_BASE, EPWM_COUNTER_MODE_UP_DOWN);

    //
    // Disable phase loading
    //
    EPWM_disablePhaseShiftLoad(EPWM4_BASE);

    //
    // Set ePWM clock pre-scaler (Clock ratio to SYSCLKOUT)
    //
    EPWM_setClockPrescaler(EPWM4_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM4_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareShadowLoadMode(EPWM4_BASE,
                                         EPWM_COUNTER_COMPARE_B,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set actions
    //

    //
    // Set PWM1A on event A, up count
    //
	//    EPWM_setActionQualifierAction(EPWM4_BASE,
	//                                      EPWM_AQ_OUTPUT_A,
	//                                      EPWM_AQ_OUTPUT_HIGH,
	//                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
	//
    //
    // Clear PWM1A on event A,
    //
	//    EPWM_setActionQualifierAction(EPWM4_BASE,
	//                                      EPWM_AQ_OUTPUT_A,
	//                                      EPWM_AQ_OUTPUT_LOW,
	//                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    EPWM_setActionQualifierAction(EPWM4_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM4_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);

    //
    // Set PWM1A on event A, up count
    //
    EPWM_setActionQualifierAction(EPWM4_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    //
    // Clear PWM1A on event A
    //
    EPWM_setActionQualifierAction(EPWM4_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
}

//
// InitEPwm5Example - Initialize EPWM2 configuration
//
void InitEPwm5Example()
{
    //
    // Setup TBCLK
    //

    //
    // Set timer period 801 TBCLKs
    //
    EPWM_setTimeBasePeriod(EPWM5_BASE, pwmperiod);
    EPWM_setPhaseShift(EPWM5_BASE, 0U);        // Phase is 0
    EPWM_setTimeBaseCounter(EPWM5_BASE, 0U);  // Clear counter

    //
    // Set Compare values
    //

    //
    // Set compare A value
    //
    EPWM_setCounterCompareValue(EPWM5_BASE, EPWM_COUNTER_COMPARE_A, pwmcmpa);

    //
    // Setup counter mode - Count up and down
    //
    EPWM_setTimeBaseCounterMode(EPWM5_BASE, EPWM_COUNTER_MODE_UP_DOWN);

    //
    // Disable phase loading
    //
    EPWM_disablePhaseShiftLoad(EPWM5_BASE);

    //
    // Set ePWM clock pre-scaler (Clock ratio to SYSCLKOUT)
    //
    EPWM_setClockPrescaler(EPWM5_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

    //
    // Setup shadowing (Load on Zero)
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM5_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareShadowLoadMode(EPWM5_BASE,
                                         EPWM_COUNTER_COMPARE_B,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set actions
    //
//    EPWM_setActionQualifierAction(EPWM5_BASE,
//                                      EPWM_AQ_OUTPUT_A,
//                                      EPWM_AQ_OUTPUT_HIGH,
//                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
//    EPWM_setActionQualifierAction(EPWM5_BASE,
//                                      EPWM_AQ_OUTPUT_A,
//                                      EPWM_AQ_OUTPUT_LOW,
//                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);

    //
    // Set PWM1A on event A, up count
    //
    EPWM_setActionQualifierAction(EPWM5_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    //
    // Clear PWM1A on event A,
    //
    EPWM_setActionQualifierAction(EPWM5_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
}

//
// InitGPIO_Testing - For Testing only
//
void InitGPIO_Testing(void)
{
    GPIO_setPinConfig(GPIO_6_EPWM4A);    // GPIO6 = PWM4A
    GPIO_setPinConfig(GPIO_7_EPWM4B);    // GPIO7 = PWM4B
    GPIO_setPinConfig(GPIO_8_EPWM5A);    // GPIO8 = PWM5A
}
//
// Test Code - End
//

//
// End of file
//
