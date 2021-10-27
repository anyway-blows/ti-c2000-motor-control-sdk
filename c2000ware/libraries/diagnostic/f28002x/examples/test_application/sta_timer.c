//#############################################################################
//
// FILE:  sta_timer.c
//
// TITLE: Self Test Application Timer source
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
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "sta_timer.h"

//
// Defines
//
#define STA_TIMER_TIMEOUT_PRD_MS  100U
#define STA_TIMER_PRD_100_MS      (DEVICE_SYSCLK_FREQ/STA_TIMER_TIMEOUT_PRD_MS)

//
// Globals
//
bool timeOutFlag;
uint16_t timeOutCount;
uint16_t isrTimeout;

//*****************************************************************************
//
// STA_Timer_timer0Isr(void)
//
//*****************************************************************************
#pragma CODE_SECTION(STA_Timer_timer0Isr,".TI.ramfunc")
__interrupt void STA_Timer_timer0Isr(void)
{

    if(isrTimeout++ > timeOutCount)
    {
        timeOutFlag = true;
        CPUTimer_stopTimer(CPUTIMER0_BASE);
    }
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);

}

//*****************************************************************************
//
// STA_Timer_config(uint16_t msTimeOut)
//
//*****************************************************************************
void STA_Timer_config(uint16_t msTimeOut)
{

    Interrupt_register(INT_TIMER0, &STA_Timer_timer0Isr);

    timeOutCount = msTimeOut/100U;
    timeOutFlag = false;
    isrTimeout = 0;

    // Set period for a 100mS interrupt
    CPUTimer_setPeriod(CPUTIMER0_BASE, (STA_TIMER_PRD_100_MS - 1));
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0);
    CPUTimer_stopTimer(CPUTIMER0_BASE);
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);
    CPUTimer_setEmulationMode(CPUTIMER0_BASE,
                              CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
    CPUTimer_enableInterrupt(CPUTIMER0_BASE);
    CPUTimer_startTimer(CPUTIMER0_BASE);

    Interrupt_enable(INT_TIMER0);
}

//*****************************************************************************
//
// STA_Timer_isTimedOut(void)
//
//*****************************************************************************
bool STA_Timer_isTimedOut(void)
{
    return(timeOutFlag);
}

//*****************************************************************************
//
// STA_Timer_clearTimeOut(void)
//
//*****************************************************************************
void STA_Timer_clearTimeOut(void)
{
    timeOutFlag = false;
}

//*****************************************************************************
//
// STA_Timer_restart(void)
//
//*****************************************************************************
void STA_Timer_restart(void)
{
    isrTimeout = 0;
    timeOutFlag = false;
    CPUTimer_startTimer(CPUTIMER0_BASE);
}

//
// End of File
//
