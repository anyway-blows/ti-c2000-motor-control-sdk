//#############################################################################
//
// FILE:  stl_hwbist.c
//
// TITLE: Diagnostic Library HWBIST software module source
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
// Includes
//
#include "stl_hwbist.h"
#include "stl_util.h"
#include "sysctl.h"
#include "interrupt.h"

//
// Defines
//
#define STL_HWBIST_NMI_CPU1_HWBISTERR   0x0010U

//
// Externs
//
extern void STL_HWBIST_runMicroTest(void);

//*****************************************************************************
//
// STL_HWBIST_NMIISR(void)
//
//*****************************************************************************
__interrupt void STL_HWBIST_errorNMIISR(void)
{
    //
    // Report global error.
    //
    STL_Util_setErrorFlag(STL_UTIL_HWBIST_NMI_INT);

    //
    // Clear the NMI CPU1 HWBIST Error flag and NMIINT flag if it is
    // the only flag left.
    //
    SysCtl_clearNMIStatus(STL_HWBIST_NMI_CPU1_HWBISTERR);
}

//*****************************************************************************
//
// STL_HWBIST_runFull(const STL_HWBIST_Error errorType)
//
//*****************************************************************************
#ifndef __cplusplus
#pragma CODE_SECTION(STL_HWBIST_runFull, ".TI.ramfunc");
#else
#pragma CODE_SECTION(".TI.ramfunc");
#endif
uint16_t STL_HWBIST_runFull(const STL_HWBIST_Error errorType)
{
    uint16_t microResult = 0U, bistDone = 0U, microCount = 0U;
	uint16_t microLimit90 = 750U;
    uint32_t prevNMIVector;

    //
    // Initialize the HWBIST for 90% LOS test.
    //
    STL_HWBIST_init(STL_HWBIST_90_LOS);

    //
    // Inject an error or no error.
    //
    STL_HWBIST_injectError(errorType);

    //
    // Enable vector fetching from PIE block.
    //
    HWREGH(PIECTRL_BASE + PIE_O_CTRL) |= PIE_CTRL_ENPIE;

    //
    // Save previous NMI vector.
    //
    prevNMIVector = HWREG(PIEVECTTABLE_BASE + PIEVECTTABLE_O_NMI);

    //
    // Register NMI ISR.
    //
    Interrupt_register(INT_NMI, STL_HWBIST_errorNMIISR);

    //
    // Execute HWBIST for 90% LOS coverage.
    //
    while((bistDone == 0U) && (microCount < microLimit90))
    {
        //
        // Increment micro-run counter.
        //
        microCount++;

        //
        // Call HWBIST assembly self test micro-run.
        //
        STL_HWBIST_runMicroTest();

        EDIS;

        //
        // Get the micro-run status.
        //
        microResult = HWREGH(HWBIST_BASE + HWBIST_O_CSTGSTAT);

        if(STL_HWBIST_BIST_DONE == (microResult & STL_HWBIST_BIST_DONE))
        {
            //
            // HWBIST is done.
            //
            bistDone = 1U;
        }
    }

    //
    // Check if there where any errors and return if so.
    //
    if(bistDone == 0U)
    {
        //
        // HWBIST failed. It did not complete in the correct number of
        // micro-runs.
        //
        // Report global error for over-run error.
        //
        STL_Util_setErrorFlag(STL_UTIL_HWBIST_OVERRUN);
    }
    else if(STL_HWBIST_NMI == (microResult & STL_HWBIST_NMI))
    {
        //
        // Report global error.
        //
        STL_Util_setErrorFlag(STL_UTIL_HWBIST_NMI_TEST);
    }
    else if((microResult & STL_HWBIST_BIST_FAIL) == STL_HWBIST_BIST_FAIL)
    {
        //
        // Report global error.
        //
        STL_Util_setErrorFlag(STL_UTIL_HWBIST_FAIL);
    }
    else
    {
        //
        // No error found.
        //
    }

    //
    // Restore previous NMI vector.
    //
    EALLOW;
    HWREG(PIEVECTTABLE_BASE + PIEVECTTABLE_O_NMI) = prevNMIVector;
    EDIS;

    //
    // Return the result.
    //
    return(microResult);
}

//*****************************************************************************
//
// STL_HWBIST_runMicro(void)
//
//*****************************************************************************
#ifndef __cplusplus
#pragma CODE_SECTION(STL_HWBIST_runMicro, ".TI.ramfunc");
#else
#pragma CODE_SECTION(".TI.ramfunc");
#endif
uint16_t STL_HWBIST_runMicro(void)
{
    uint16_t microResult;
    uint32_t prevNMIVector;

    //
    // Save previous NMI vector.
    //
    prevNMIVector = HWREG(PIEVECTTABLE_BASE + PIEVECTTABLE_O_NMI);

    //
    // Enable vector fetching from PIE block
    //
    HWREGH(PIECTRL_BASE + PIE_O_CTRL) |= PIE_CTRL_ENPIE;

    //
    // Register NMI ISR.
    //
    EALLOW;
    HWREG(PIEVECTTABLE_BASE + PIEVECTTABLE_O_NMI) =
                                         (uint32_t)STL_HWBIST_errorNMIISR;
    EDIS;

    //
    // Call HWBIST assembly self test micro-run.
    //
    STL_HWBIST_runMicroTest();

    //
    // Restore previous NMI vector.
    //
    EALLOW;
    HWREG(PIEVECTTABLE_BASE + PIEVECTTABLE_O_NMI) = prevNMIVector;
    EDIS;

    //
    // Get the micro-run status.
    //
    microResult = HWREGH(HWBIST_BASE + HWBIST_O_CSTGSTAT);

    if(STL_HWBIST_NMI == (microResult & STL_HWBIST_NMI))
    {
        //
        // Report global error.
        //
        STL_Util_setErrorFlag(STL_UTIL_HWBIST_NMI_TEST);
    }
    else if((microResult & STL_HWBIST_BIST_FAIL) == STL_HWBIST_BIST_FAIL)
    {
        //
        // Report global error.
        //
        STL_Util_setErrorFlag(STL_UTIL_HWBIST_FAIL);
    }
    else
    {
        //
        // No error found.
        //
    }

    //
    // Return the result.
    //
    return(microResult);
}

//*****************************************************************************
//
// STL_HWBIST_restoreContext(void)
//
//*****************************************************************************
#pragma RETAIN(STL_HWBIST_restoreContext);
#ifndef __cplusplus
#pragma CODE_SECTION(STL_HWBIST_restoreContext, "hwbist");
#else
#pragma CODE_SECTION("hwbist");
#endif
void STL_HWBIST_restoreContext(void)
{
    //
    // Initialize the stack pointer.
    //
    STL_HWBIST_REF_STACK;
    STL_HWBIST_MOV_SP_STACK;

    //
    // Select C28x object mode.
    //
    STL_HWBIST_C28OBJ;

    //
    // Select C27x/C28x addressing.
    //
    STL_HWBIST_C28ADDR;

    //
    // Set blocks M0/M1 for C28x mode.
    //
    STL_HWBIST_C28MAP;

    //
    // Always use stack addressing mode.
    //
    STL_HWBIST_CLRC_PAGE0;

    //
    // Initialize DP to point to the low 64K.
    //
    STL_HWBIST_MOVW_DP_0;
    STL_HWBIST_CLRC_OVM;
    STL_HWBIST_SPM_0;

    EALLOW;

    //
    // Branch to reset handler and context restore.
    //
    STL_HWBIST_REF_HANDLE_RESET_FXN;
    STL_HWBIST_LCR_HANDLE_RESET_FXN;
}

//*****************************************************************************
//
// STL_HWBIST_init(const STL_HWBIST_Coverage coverage)
//
//*****************************************************************************
#ifndef __cplusplus
#pragma CODE_SECTION(STL_HWBIST_init, ".TI.ramfunc");
#else
#pragma CODE_SECTION(".TI.ramfunc");
#endif
void STL_HWBIST_init(const STL_HWBIST_Coverage coverage)
{
    EALLOW;

    //
    // Soft reset the HWBIST controller to put it into a known state.
    //
    HWREG(HWBIST_BASE + HWBIST_O_CSTCGCR5) = 0x80000000UL;

    //
    // Clear the HWBIST soft reset bit.
    //
    HWREG(HWBIST_BASE + HWBIST_O_CSTCGCR5) = 0x0000UL;

    //
    // Load target coverage metric.
    //
    HWREG(HWBIST_BASE + HWBIST_O_CSTCGCR6) = (uint32_t)coverage;

    //
    // Restart HWBIST.
    //
    HWREG(HWBIST_BASE + HWBIST_O_CSTCGCR5) = 0x000AUL;

    //
    // Load a micro run cycle count.
    //
    HWREG(HWBIST_BASE + HWBIST_O_CSTCGCR1) = 48UL;

    //
    // Run launch-on-shift at clock divide of 1.
    //
    HWREG(HWBIST_BASE + HWBIST_O_CSTCGCR7) = 15UL;

    //
    // Support load for this test.
    //
    HWREG(HWBIST_BASE + HWBIST_O_CSTCGCR8) = 50UL;
    HWREG(HWBIST_BASE + HWBIST_O_CSTCSADDR) = 6008UL << 16;

    //
    // Set pattern count.
    //
    HWREG(HWBIST_BASE + HWBIST_O_CSTCPCNT) = 750UL;

    //
    // Configuration is done.
    //
    HWREG(HWBIST_BASE + HWBIST_O_CSTCCONFIG) = 0x0005UL;

    //
    // Set return address.
    //
    HWREG(HWBIST_BASE + HWBIST_O_CSTCRET) = 0x0000UL;
    NOP;
    NOP;

    EDIS;
}

//
// End of File
//
