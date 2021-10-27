//#############################################################################
//
// FILE:  sta_util.h
//
// TITLE: Self Test Application Utility header
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

#ifndef STA_UTIL_H
#define STA_UTIL_H

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//
// Included Files
//
#include <stdbool.h>
#include <stdint.h>
#include "stl_util.h"
#include "cputimer.h"

//*****************************************************************************
//
//! \addtogroup sta_util STA_Util API Functions
//!
//! This module contains utility functions for this test application. This
//! primarily includes the functions used to profile the cycle counts of the
//! SDL functions.
//!
//! The code for this module is contained in <tt>sta_util.c</tt>, with
//! <tt>sta_util.h</tt> containing the API declarations.
//!
//! @{
//
//*****************************************************************************

//
// Defines
//

//
// Enable HWBIST test
//
// Due to RAM size constraints and the need to locate the HWBIST reset handler at
// the RAM boot entry point, only the Flash build is supported for HWBIST.
//
#ifdef _FLASH
//! Used to enable or disable HWBIST micro-run test.
//! A value of 0 will disable the HWBIST micro-run test.
//! A value of not 0 will enable the HWBIST micro-run test.
#define STA_UTIL_HWBIST_MICRO       0
//! Used to enable or disable a complete HWBIST test.
//! A value of 0 will disable the complete HWBIST test.
//! A value of not 0 will enable the complete HWBIST test.
#define STA_UTIL_HWBIST_FULL        0
#endif

//
// Enable profiling
//
//! Used to enable or disable CPUTimer cycle profiling of STA tests.
//! A value of 0 will disable cycle profiling.
//! A value of not 0 will enable cycle profiling.
#define STA_UTIL_PROFILE            1

#if STA_UTIL_PROFILE
//*****************************************************************************
//
//! \brief Configures the CPUTimer specified by the parameter \b base for
//! profiling cycle counts.
//!
//! \param base is the base address of a valid CPUTimer.
//!
//! \return None.
//
//*****************************************************************************
extern void STA_Util_configProfiler(uint32_t base);

//*****************************************************************************
//
//! \brief Starts the CPUTimer used for profiling cycles.
//!
//! \param base is the base address of a valid CPUTimer.
//!
//! \return None.
//
//*****************************************************************************
static inline void STA_Util_startProfiler(uint32_t base)
{
    //
    // Set TSS bit of register TCR
    //
    HWREGH(base + CPUTIMER_O_TCR) |= CPUTIMER_TCR_TSS;

    //
    // Reload the timer counter.
    //
    HWREGH(base + CPUTIMER_O_TCR) |= CPUTIMER_TCR_TRB;

    //
    // Clear TSS bit of register TCR
    //
    HWREGH(base + CPUTIMER_O_TCR) &= ~CPUTIMER_TCR_TSS;
}

//*****************************************************************************
//
//! \brief Stops the CPUTimer used for profiling cycles.
//!
//! \param base is the base address of a valid CPUTimer.
//!
//! \return Returns the number of cycles elapsed since STA_Util_startProfiler()
//! was called.
//
//*****************************************************************************
static inline uint32_t STA_Util_stopProfiler(uint32_t base)
{
    //
    // Set TSS bit of register TCR
    //
    HWREGH(base + CPUTIMER_O_TCR) |= CPUTIMER_TCR_TSS;

    //
    // Return the cycle count
    //
    return(0xFFFFFFFFU - HWREG(base + CPUTIMER_O_TIM));
}
#endif

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus

#endif

#endif // STA_UTIL_H

//
// End of File
//
