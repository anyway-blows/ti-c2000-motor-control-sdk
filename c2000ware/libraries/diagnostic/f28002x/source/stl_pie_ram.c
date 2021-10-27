//###########################################################################
//
// FILE:  stl_pie_ram.c
//
// TITLE: Diagnostic Library PIE RAM software module source
//
//###########################################################################
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
//###########################################################################

//
// Includes
//
#include "stl_pie_ram.h"
#include "stl_util.h"
#include "interrupt.h"

//
// Globals.
//
volatile static uint16_t STL_PIE_RAM_handlerFlag;

//*****************************************************************************
//
// void STL_PIE_RAM_handler(void)
//
//*****************************************************************************
void STL_PIE_RAM_handler(void)
{
    //
    // Set the global flag.
    //
    STL_PIE_RAM_handlerFlag = 1;
}

//*****************************************************************************
//
// STL_PIE_RAM_configHandler(void)
//
//*****************************************************************************
void STL_PIE_RAM_configHandler(const void *handlerPtr)
{
    //
    // Configure the function as the PIE RAM error handler.
    //
    EALLOW;
    HWREG(CPUSYS_BASE + SYSCTL_O_PIEVERRADDR) = (uint32_t)handlerPtr;
    EDIS;
}

//*****************************************************************************
//
// STL_PIE_RAM_injectFault(uint16_t entry)
//
//*****************************************************************************
void STL_PIE_RAM_injectFault(uint16_t entry)
{
    //
    // Assert the entry is within the limits of the PIE vector table and
    // a multiple of 2 to ensure it is 32-bit aligned.
    //
    ASSERT(entry >= STL_PIE_RAM_MIN_INDEX);
    ASSERT(entry <= STL_PIE_RAM_MAX_INDEX);
    ASSERT((entry & 0x0001) == 0);

    //
    // Inject an error by bit-flipping the entry specified in the redundant
    // PIE RAM vector table.
    //
    EALLOW;
    HWREG(STL_PIE_RAM_REDUNDANT_PIE_ADDRESS + entry) =
          ~HWREG(PIEVECTTABLE_BASE + entry);
    EDIS;
}

//*****************************************************************************
//
// STL_PIE_RAM_retoreVector(uint16_t entry)
//
//*****************************************************************************
void STL_PIE_RAM_restoreVector(uint16_t entry)
{
    //
    // Rewrite the vector table entry. This will update the redundant table
    // as well.
    //
    EALLOW;
    HWREG(PIEVECTTABLE_BASE + entry) = HWREG(PIEVECTTABLE_BASE + entry);
    EDIS;
}

//*****************************************************************************
//
// STL_PIE_RAM_retoreTable(uint32_t *pieTableSourcePtr)
//
//*****************************************************************************
void STL_PIE_RAM_restoreTable(const uint32_t *pieTableSourcePtr)
{
    int16_t index;

    EALLOW;

    //
    // Restore the PIE vector table.
    //
    for(index = STL_PIE_RAM_MIN_INDEX; index < STL_PIE_RAM_MAX_INDEX;
        index = index + 2)
    {
        HWREG((uint32_t)PIEVECTTABLE_BASE + (uint32_t)index) =
                                    pieTableSourcePtr[((uint16_t)index >> 1)];
    }
    EDIS;
}

//*****************************************************************************
//
// STL_PIE_RAM_testRAM(void)
//
//*****************************************************************************
uint16_t STL_PIE_RAM_testRAM(void)
{

    int16_t index;
    uint16_t status;

    status = STL_PIE_RAM_PASS;

    //
    // Loop over the whole PIE RAM table.
    //
    for(index = STL_PIE_RAM_MIN_INDEX; index < STL_PIE_RAM_MAX_INDEX;
        index = index + 2)
    {
        //
        // Compare redundant table entry to the main table entry
        //
        if(HWREG((uint32_t)PIEVECTTABLE_BASE + (uint32_t)index) !=
           HWREG((uint32_t)STL_PIE_RAM_REDUNDANT_PIE_ADDRESS + (uint32_t)index))
        {
            //
            // Set the return status to the index. This will provide some
            // additional error information.
            //
            status = (uint16_t)index + PIEVECTTABLE_BASE;

            //
            // Report global error.
            //
            STL_Util_setErrorFlag(STL_UTIL_PIE_RAM_MISMATCH);
        }
    }

    return(status);
}

//*****************************************************************************
//
// STL_PIE_RAM_testHandler(void)
//
//*****************************************************************************
uint16_t STL_PIE_RAM_testHandler(uint32_t interruptNumber)
{

    uint16_t intGroup, groupMask, testStatus;
    uint32_t prevHandler;

    //
    // Assert the interrupt number is a PIE interrupt.
    // This test will only work for PIE interrupts.
    //
    ASSERT((uint16_t)(interruptNumber >> 16U) >= 0x20U &&
           (uint16_t)(interruptNumber >> 16U) <= 0xDFU);

    //
    // Get the PIE RAM vector entry.
    //
    uint16_t pieEntry = (((interruptNumber & STL_PIE_RAM_VECT_ID_M) >>
                          STL_PIE_RAM_VECT_ID_S) * 2U);

    //
    // Configure the default handler for the STL.
    //
    prevHandler = HWREG(CPUSYS_BASE + SYSCTL_O_PIEVERRADDR);
    STL_PIE_RAM_configHandler(&STL_PIE_RAM_handler);

    //
    // Inject a fault at the entry.
    //
    STL_PIE_RAM_injectFault(pieEntry);

    //
    // Enable and force the interruptNumber. This should trigger the mismatch
    // handler. If the handler is not serviced, then this test will FAIL.
    //

    //
    // Determine the PIE interrupt group.
    //
    intGroup = ((uint16_t)(interruptNumber & STL_PIE_RAM_TABLE_COL_M) >>
                STL_PIE_RAM_TABLE_COL_S) - 1U;
    groupMask = 1U << intGroup;

    //
    // Enable the PIE interrupt.
    //
    HWREGH(PIECTRL_BASE + PIE_O_IER1 + (intGroup * 2U)) |=
        1U << ((uint16_t)(interruptNumber & STL_PIE_RAM_TABLE_ROW_M) - 1U);

    //
    // Enable PIE Group Interrupt.
    //
    IER |= groupMask;

    //
    // Reset the global handler flag.
    //
    STL_PIE_RAM_handlerFlag = 0U;

    //
    // Force the PIE interrupt.
    //
    HWREGH(PIECTRL_BASE + PIE_O_IFR1 + (intGroup * 2U)) |=
        1U << ((uint16_t)(interruptNumber & STL_PIE_RAM_TABLE_ROW_M) - 1U);

    //
    // Short delay to ensure the exception was handled.
    //
    SysCtl_delay(5);

    //
    // Check that the PIE error interrupt occurred as expected.
    //
    if(STL_PIE_RAM_handlerFlag == 1U)
    {
        testStatus = STL_PIE_RAM_PASS;
    }
    else
    {
        testStatus = STL_PIE_RAM_FAIL_HANDLER;

        //
        // Report global error.
        //
        STL_Util_setErrorFlag(STL_UTIL_PIE_RAM_INT);
    }

    //
    // Restore the interruptNumber in PIE RAM redundant table.
    //
    STL_PIE_RAM_restoreVector(pieEntry);

    //
    // Restore the PIEVERRADDR.
    //
    STL_PIE_RAM_configHandler((void *)prevHandler);

    //
    // Acknowledge the PIE group. This must occur or else the PIE group
    // will not be able to service further interrupts.
    //

    //
    // This can also be accomplished in the error handler by looking at the
    // PIECTRL register and determining which group the PIE RAM mismatch
    // occurred for during a vector fetch.
    //
    Interrupt_clearACKGroup(groupMask);
    return(testStatus);
}

//
// End of File
//
