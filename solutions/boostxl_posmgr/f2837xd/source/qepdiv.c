//#############################################################################
//
// FILE:           pto_qepdiv.c
//
// Description:    Contains all the initialization, data declarations and
//                 setup for qepdiv pto interface. This file serves are a
//                 template for using pto_qepdiv Library to interface and
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
#include "qepdiv.h"
#include "device.h"
#include "driverlib.h"

//
// Globals
//
uint16_t i = 0;

//
// pto_qepdiv_init - Function to initialize pto_qepdiv operation
//
void
pto_qepdiv_init(void)
{
    //
    // Configure EPWM2 to drive default values on GPIO2 and GPIO3
    //
    EPWM2_Config();

    pto_qepdiv_setup_GPIO();
    pto_qepdiv_setupPeriph();

    SysCtl_delay(100L);
}

//
// pto_qepdiv_setup_GPIO - Configure QEP input via InputXBar. Configure QEP
// outputs via GPIOs and OUTPUTXBAR
//
void
pto_qepdiv_setup_GPIO(void)
{
    EALLOW;
    //
    // QEP inputs to be tapped from GPIO10/11/9 - via InputXBar Input4/5/6
    //
    XBAR_setInputPin(XBAR_INPUT4, 10);  // GPIO10 = QEPA
    XBAR_setInputPin(XBAR_INPUT5, 11);  // GPIO11 = QEPB
    XBAR_setInputPin(XBAR_INPUT6, 9);  // GPIO9 = QEPI

    //
    // QEP outputs available on GPIO2/3 - QEPA/B (over EPWM2A/B)
    //
    GPIO_setPinConfig(GPIO_2_EPWM2A);  // Configure GPIO2 (Pulse out A)
    GPIO_setDirectionMode(2, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_STD);

    GPIO_setPinConfig(GPIO_3_EPWM2B);  // Configure GPIO3 (Pulse out B)
    GPIO_setDirectionMode(3, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);

    //
    // QEP outputs Index on OUTPUTXBAR3 - on GPIO14
    // (users can choose any GPIO with OUTPUTXBAR3
    //
    GPIO_setMasterCore(14, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_14_OUTPUTXBAR3);
    EDIS;
}

//
// error -
//
void
error(void)
{
    __asm("     ESTOP0");         // Test failed!! Stop!
    for (;;);
}

//
// EPWM2_Config - Configure EPWM2 to drive default values on GPIO6, GPIO7 and
// GPIO8
//
void EPWM2_Config(void)
{
    EALLOW;
    EPWM_setTripZoneAction(EPWM2_BASE, EPWM_TZ_ACTION_EVENT_TZA,
                           EPWM_TZ_ACTION_HIGH);
    EPWM_setTripZoneAction(EPWM2_BASE, EPWM_TZ_ACTION_EVENT_TZB,
                           EPWM_TZ_ACTION_HIGH);
    EPWM_forceTripZoneEvent(EPWM2_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EDIS;
}

//
// End of File
//

