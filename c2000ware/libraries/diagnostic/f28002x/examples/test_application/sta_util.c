//#############################################################################
//
// FILE:  sta_util.c
//
// TITLE: Self Test Application Utility source
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
#include <stdbool.h>
#include <stdint.h>
#include "driverlib.h"
#include "sta_util.h"
#include "sysctl.h"
#include "interrupt.h"

//*****************************************************************************
//
// STA_Util_corrErrorISR(void)
//
//*****************************************************************************
__interrupt void STA_Util_corrErrorISR(void)
{
    //
    // Clear error interrupt flag in flash wrapper.
    //
    EALLOW;
    HWREG(FLASH0ECC_BASE + FLASH_O_ERR_INTCLR) |=
                                          FLASH_ERR_INTCLR_SINGLE_ERR_INTCLR;
    EDIS;

    //
    // Acknowledge the PIE
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP12);
}

//*****************************************************************************
//
// STA_Util_uncorrErrorNMIISR(void)
//
//*****************************************************************************
__interrupt void STA_Util_uncorrErrorNMIISR(void)
{
    //
    // Clear error interrupt flag in flash wrapper.
    //
    EALLOW;
    HWREG(FLASH0ECC_BASE + FLASH_O_ERR_INTCLR) |=
                                             FLASH_ERR_INTCLR_UNC_ERR_INTCLR;
    EDIS;

    //
    // Clear the NMI flash uncorrectable error flag and NMIINT flag if it is
    // the only flag left.
    //
    SysCtl_clearNMIStatus(SYSCTL_NMI_FLUNCERR);
}

#if STA_UTIL_PROFILE
//*****************************************************************************
//
// void STA_Util_configProfiler
//
//*****************************************************************************
void STA_Util_configProfiler(uint32_t base)
{
    //
    // Clear overflow flag.
    //
    CPUTimer_clearOverflowFlag(base);

    //
    // Initialize timer period.
    //
    CPUTimer_setPeriod(base, 0xFFFFFFFFU);

    //
    // Initialize pre-scaler.
    //
    CPUTimer_setPreScaler(base, 0U);

    //
    // Reload timer counter.
    //
    CPUTimer_reloadTimerCounter(base);

    //
    // Set the timer to stop after next decrement on emulation halt.
    //
    CPUTimer_setEmulationMode(base, CPUTIMER_EMULATIONMODE_RUNFREE);
}
#endif

//
// End of File
//
