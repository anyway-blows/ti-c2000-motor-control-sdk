//#############################################################################
//
// FILE:  sta_main.c
//
// TITLE: Self Test Application Main source
//
//#############################################################################
// $TI Release: C2000 Diagnostic Library v2.01.00 $
// $Release Date: Fri Feb 12 19:23:23 IST 2021 $
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
// Included files for device configuration
//
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "driverlib.h"
#include "device.h"

//
// Included files for test-infrastructure.
//
#include "sta_timer.h"
#include "sta_tests.h"
#include "sta_comm.h"

//
// Defines
//
#define STA_MAIN_TIMEOUT    4000U

//
// Globals
//
int16_t enableErrorInject;
int16_t enableReset;

//
// Prototypes
//
void STA_Main_configXCLKOut(void);

#ifdef _FLASH
extern uint16_t HwbistLoadStart;
extern uint16_t HwbistLoadSize;
extern uint16_t HwbistRunStart;
#endif

//
// Main
//
void main(void)
{
#ifdef _FLASH
    memcpy(&HwbistRunStart, &HwbistLoadStart, (size_t)&HwbistLoadSize);
#endif

    unsigned char *msg;
    uint16_t testIndex = 0;

    enableErrorInject = 0;

    //
    // Configure PLL, disable WD, enable peripheral clocks.
    //
    Device_init();

    //
    // Disable pin locks and enable internal pull-ups.
    //
    Device_initGPIO();

    //
    // Put watchdog in interrupt mode.
    //
    SysCtl_setWatchdogMode(SYSCTL_WD_MODE_INTERRUPT);

    //
    // Disable global interrupts.
    //
    DINT;

    //
    // Initialize interrupt controller and vector table.
    //
    Interrupt_initModule();
    Interrupt_initVectorTable();

    //
    // Enable NMI if loading from debugger and absent from GEL file.
    //
    EALLOW;
    HWREGH(NMI_BASE + NMI_O_FLGCLR) = 0xFFFF;
    HWREGH(NMI_BASE + NMI_O_CFG) |= NMI_CFG_NMIE;
    EDIS;

    //
    // Debug
    //
    STA_Main_configXCLKOut();

    //
    // Configure SCI A as the main communication port
    //
    STA_Comm_configSCIA();

    //
    // Configure Timer 0 as the main time-out timer
    //
    STA_Timer_config(STA_MAIN_TIMEOUT);

#if STA_UTIL_PROFILE
    STA_Util_configProfiler(CPUTIMER1_BASE);
#endif

    //
    // Enable Global Interrupt (INTM) and real time interrupt (DBGM)
    //
    EINT;
    ERTM;

    //
    // Send starting message.
    //
    msg = "\r\n\n\nBegin Test\0";
    STA_Comm_transmitData(msg);

    for(;;)
    {
        if(STA_Timer_isTimedOut())
        {
            if(enableErrorInject)
            {
                STA_Tests_injectErrorEnable();
            }
            else
            {
                STA_Tests_injectErrorDisable();
            }

            STA_Comm_transmitData(
                     STA_Tests_testDevice(STA_Tests_testArray[testIndex++]));

            if(testIndex >= STA_TESTS_NUMBERS)
            {
                testIndex = 0U;
            }
            STA_Timer_restart();
        }
    }
}

//
// STA_Main_configXCLKOut(void)
//
void STA_Main_configXCLKOut(void)
{
    // Configure GPIO as SYSCLK out
    GPIO_setPadConfig(18, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_18_XCLKOUT);

    // Clock source is SYSCLK
    SysCtl_selectClockOutSource(SYSCTL_CLOCKOUT_SYSCLK);

    EALLOW;
    // XCLOCK out = clock source /8
    HWREG(CLKCFG_BASE + SYSCTL_O_XCLKOUTDIVSEL) = 3U;
    EDIS;
}

//
// End of File
//
