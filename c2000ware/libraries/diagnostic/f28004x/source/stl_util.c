//###########################################################################
//
// FILE:  stl_util.c
//
// TITLE: Diagnostic Library Utility software module source
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
#include "stl_util.h"

//
// Globals
//
#pragma DATA_ALIGN (STL_Util_globalErrorFlags, 4);
static volatile uint32_t STL_Util_globalErrorFlags = 0UL;

//*****************************************************************************
//
//  STL_Util_setErrorFlag(const uint32_t errorFlag)
//
//*****************************************************************************
void STL_Util_setErrorFlag(const STL_Util_ErrorFlag errorFlag)
{
    //
    // Set the global flag associated with errorFlag
    //
    STL_Util_globalErrorFlags |= (uint32_t)errorFlag;
}

//*****************************************************************************
//
//  STL_Util_getErrorFlag(void)
//
//*****************************************************************************
uint32_t STL_Util_getErrorFlag(void)
{
    //
    // Return the global flag status
    //
    return((uint32_t)STL_Util_globalErrorFlags);
}

//*****************************************************************************
//
//  STL_Util_clearErrorFlag(const uint32_t errorFlag)
//
//*****************************************************************************
void STL_Util_clearErrorFlag(const STL_Util_ErrorFlag errorFlag)
{
    //
    // Clear the global flag associated with errorFlag
    //
    STL_Util_globalErrorFlags &= ~(uint32_t)errorFlag;
}

//
// End of File
//
