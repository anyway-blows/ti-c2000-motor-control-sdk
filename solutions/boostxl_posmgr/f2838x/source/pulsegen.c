//###########################################################################
//
// FILE:           pto_pulsegen.c
//
// Description:    Contains all the initialization, data declarations and
//                 setup for pulsegen pto interface. This file serves are a
//                 template for using pto_pulsegen Library to interface and
//                 incorporates all the library specific initializations and
//                 other important aspects of usage.
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

//
// Included Files
//
#include "pulsegen.h"
#include "device.h"
#include "driverlib.h"

//
// Globals
//
uint16_t status;
uint32_t remVal;

uint16_t inumPulses1 = 50, iPeriod1 = 1000, iptoInterruptTime1 = 500;
uint16_t iptoDirection1 = 1, irun1 = 1;
uint16_t inumPulses2 = 25, iPeriod2 = 1000, iptoInterruptTime2 = 500;
uint16_t iptoDirection2 = 0, irun2 = 1;

//
// pto_initPulsegen - Function to initialize pto_pulsegen operation
//
void
pto_initPulsegen(void)
{
    //
    // Enable clocks to PWM1
    //
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLB1);

    //
    // Configure EPWM1 to drive default values on GPIO6 and GPIO7
    //
    pto_configEPWM();

    pto_setupGPIO();
    pto_pulsegen_setupPeriph();

    //
    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    //
    Interrupt_register(INT_CLB1, &ptoISR);
    Interrupt_enableInCPU(INTERRUPT_CPU_INT8);
    Interrupt_enable(INT_CLB1);

    SysCtl_delay(200000L);
}

//
// pto_setupGPIO - Configure GPIOs to monitor signals
//
void
pto_setupGPIO(void)
{
    GPIO_setMasterCore(26, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_26_OUTPUTXBAR3);

    GPIO_setMasterCore(15, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_15_OUTPUTXBAR4);
}

//
// pto_error -
//
void
pto_error(void)
{
    __asm("     ESTOP0");         // Test failed!! Stop!
    for (;;);
}

//
// pto_configEPWM - Configure EPWM1 to drive default values on GPIO6 and GPIO7
//
void
pto_configEPWM(void)
{
    EPWM_setTripZoneAction(EPWM1_BASE, EPWM_TZ_ACTION_EVENT_TZA,
                           EPWM_TZ_ACTION_HIGH);
    EPWM_setTripZoneAction(EPWM1_BASE, EPWM_TZ_ACTION_EVENT_TZB,
                           EPWM_TZ_ACTION_HIGH);
    EPWM_forceTripZoneEvent(EPWM1_BASE, EPWM_TZ_FORCE_EVENT_OST);
}

//
// pto_setOptions - This function can optionally used in turn to call and set
// the following parameters for pto_pulsegen_runPulseGen function:
//    numPulses - Number of pulses in the given pto period
//    Period- PTO period - specify in CLB clock cycles (SYSCLK/2)
//    ptoInterruptTime - PTO interrupt time, specify in
//                       CLB clock cycles (SYSCLK/2) - 40-60% of Period
//    ptoDirection - Direction output for the upcoming pto period
//    run - Run=1/Stop=0
//
// Users can choose to use the pto_pulsegen_runPulseGen() directly
//
uint32_t
pto_setOptions(uint32_t numPulses, uint32_t period, uint32_t ptoInterruptTime,
               uint16_t ptoDirection, uint16_t run)
{
    uint32_t pulseFreq, reminder;
    uint32_t pulseLo;
    uint32_t pulseHi;
    uint32_t ptoActivePeriod;
    uint32_t ptoFullPeriod;

    pulseFreq = period / numPulses;
    reminder = period - (pulseFreq * numPulses);
    pulseLo = (pulseFreq / 2 );
    pulseHi = pulseFreq - pulseLo;
    ptoActivePeriod = (pulseFreq * numPulses);
    ptoFullPeriod = period;

    pto_pulsegen_runPulseGen(pulseLo, pulseHi, ptoActivePeriod,
                                ptoFullPeriod, ptoInterruptTime, ptoDirection,
                                run);
    return(reminder);
}

//
// ptoISR -
//
__interrupt void
ptoISR(void)
{
    //
    // can be optionally used to accumulate reminder for the next period
    //
    // uint32_t  retval1;

    if(status == 0)
    {
        remVal = pto_setOptions(inumPulses1, iPeriod1, iptoInterruptTime1,
                                iptoDirection1, irun1);
        status = 1;
    }
    else
    {
        remVal = pto_setOptions(inumPulses2, iPeriod2, iptoInterruptTime2,
                                iptoDirection2, irun2);
        status = 0;
    }

    //
    // Issue PIE ack
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP5);
}

//
// End of File
//
