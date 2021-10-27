//###########################################################################
//
// FILE:  stl_can_ram.c
//
// TITLE: Diagnostic Library CAN message RAM test software module source
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
#include "stl_can_ram.h"
#include "stl_util.h"
#include "inc/hw_types.h"

//*****************************************************************************
//
// STL_CAN_RAM_testRAM(const uint32_t canBase,
//                     const STL_March_Pattern pattern,
//                     const uint16_t startAddress,
//                     const uint16_t endAddress,
//                     const uint32_t copyAddress)
//
//*****************************************************************************
void STL_CAN_RAM_testRAM(const uint32_t canBase,
                         const STL_March_Pattern pattern,
                         const uint32_t startAddress,
                         const uint32_t endAddress,
                         const uint32_t copyAddress)
{
    uint32_t length;

    ASSERT(CAN_isBaseValid(canBase));
    ASSERT(endAddress >= startAddress);
    ASSERT((canBase + STL_CAN_RAM_ADDR_OFFSET) <= startAddress);
    ASSERT((canBase + STL_CAN_RAM_MAX_ADDR_OFFSET) >= endAddress);

    //
    // The length needs to be divided by 2 since the test function expects a
    // length in terms of 32-bit words.
    //
    if(endAddress >= startAddress)
    {
        length = (endAddress - startAddress) / 2U;
    }
    else
    {
        length = 0U;
    }

    //
    // Make sure parity checking is turned on.
    //
    HWREGH(canBase + CAN_O_CTL) &= ~CAN_CTL_PMD_M;

    //
    // Turn on message RAM direct access mode.
    //
    CAN_enableTestMode(canBase, 0U);
    CAN_enableMemoryAccessMode(canBase);

    //
    // Run the March test.
    //
    if(STL_CAN_RAM_NO_COPY == copyAddress)
    {
        STL_March_testRAM(pattern, startAddress, length);
    }
    else
    {
        STL_March_testRAMCopy(pattern, startAddress, length, copyAddress);
    }

    //
    // Disable the test modes.
    //
    CAN_disableMemoryAccessMode(canBase);
    CAN_disableTestMode(canBase);
}

//*****************************************************************************
//
// STL_CAN_RAM_injectError(const uint32_t canBase, const uint32_t address,
//                         const uint32_t xorMask);
//
//*****************************************************************************
void STL_CAN_RAM_injectError(const uint32_t canBase, const uint32_t address,
                             const uint32_t xorMask)
{
    ASSERT(CAN_isBaseValid(canBase));
    ASSERT((address >= (canBase + STL_CAN_RAM_ADDR_OFFSET)) &&
           (address <= (canBase + STL_CAN_RAM_MAX_ADDR_OFFSET)));

    //
    // Turn off parity, so we can flip a bit without updating it.
    //
    HWREGH(canBase + CAN_O_CTL) =
       (HWREGH(canBase + CAN_O_CTL) & ~CAN_CTL_PMD_M) | STL_CAN_RAM_PARITY_OFF;

    //
    // Turn on message RAM direct access mode.
    //
    CAN_enableTestMode(canBase, 0U);
    CAN_enableMemoryAccessMode(canBase);

    //
    // Flip a bit.
    //
    HWREG_BP(address) ^= xorMask;

    //
    // Turn parity back on.
    //
    HWREGH(canBase + CAN_O_CTL) &= ~CAN_CTL_PMD_M;

    //
    // Disable the test modes.
    //
    CAN_disableMemoryAccessMode(canBase);
    CAN_disableTestMode(canBase);
}

//*****************************************************************************
//
// STL_CAN_RAM_checkErrorStatus(const uint32_t canBase)
//
//*****************************************************************************
uint16_t STL_CAN_RAM_checkErrorStatus(const uint32_t canBase)
{
    uint16_t status;

    if(CAN_getStatus(canBase) & CAN_STATUS_PERR)
    {
        status = STL_CAN_RAM_ERROR;

        //
        // Set global error flag.
        //
        STL_Util_setErrorFlag(STL_UTIL_CAN_RAM_PARITY);
    }
    else
    {
        status = STL_CAN_RAM_PASS;
    }

    return(status);
}

//
// End of File
//
