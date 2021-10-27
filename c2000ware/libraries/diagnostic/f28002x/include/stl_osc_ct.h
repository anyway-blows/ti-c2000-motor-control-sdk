//#############################################################################
//
// FILE:  stl_osc_ct.h
//
// TITLE: Diagnostic Library Oscillator CPU Timer software module header
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

#ifndef STL_OSC_CT_H
#define STL_OSC_CT_H

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
// Includes
//
#include <stdbool.h>
#include <stdint.h>
#include "cpu.h"
#include "cputimer.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"

//*****************************************************************************
//
//! \addtogroup stl_osc_ct Oscillator CPU Timer API Functions
//!
//! The code for this module is contained in <tt>source/stl_osc_ct.c</tt>, with
//! <tt>include/stl_osc_ct.h</tt> containing the API declarations for use by
//! applications.
//!
//! # Error Injection #
//!
//! Here are techniques to inject error:
//!
//! \b STL_OSC_CT_startTest and STL_OSC_CT_stopTest()
//! - Set maxCount and minCount to values so that the Timer 2 counter will be
//!   out of range.
//! - Set the prescaler so that the Timer 2 counter will not be in the
//!   minCount/maxCount range.
//!
//! @{
//
//*****************************************************************************

//
// Defines
//
#define STL_OSC_CT_FAIL             1U
#define STL_OSC_CT_PASS             0U

#define STL_OSC_CT_PERIOD           0xFFFFFFFFU

//! \brief Defines the OSC Timer 2 test object
//!
typedef struct
{
    uint32_t              minCount;     //!< Lower bound count
    uint32_t              maxCount;     //!< Upper bound count
    CPUTimer_ClockSource  clockSource;  //!< Clock source for Timer2
    CPUTimer_Prescaler    prescaler;    //!< Prescaler for selected clock source
} STL_OSC_CT_Obj;

//! \brief Defines the OSC Timer 2 test handle
//!
typedef STL_OSC_CT_Obj * STL_OSC_CT_Handle;

//*****************************************************************************
//
//! \brief Starts CPU Timer 2 to test the oscillator source.
//!
//! \param oscTimer2Handle is a pointer to the Oscillator Timer 2 object.
//!
//! This function disables CPU Timer 2 interrupts, configures the CPU Timer 2
//! to use the specified clock source and prescaler, starts the timer, and then
//! returns.
//!
//! The user should configure the PLL and CPU Timer with independent clock
//! sources.
//!
//! The user must preserve and restore the registers modified by this function.
//! The following CPU Timer 2 and System Control registers are modified by the
//! function.
//!  - TCR
//!  - PRD
//!  - TPRH
//!  - TPR
//!  - TMR2CLKCTL
//!
//! \note When \b CPUTIMER_CLOCK_SOURCE_SYS is selected as the \b clockSource,
//! the \b prescaler is bypassed.
//!
//! \return None.
//!
//
//*****************************************************************************
extern void
STL_OSC_CT_startTest(const STL_OSC_CT_Handle oscTimer2Handle);

//*****************************************************************************
//
//! \brief Stops CPU Timer 2 and checks the elapsed time.
//!
//! \param oscTimer2Handle is a pointer to the Oscillator Timer 2 object.
//!
//! This function stops CPU Timer 2 and then compares the number of ticks that
//! have elapsed with the min and max boundaries. This function is intended to
//! be used in combination with STL_OSC_CT_startTest() to perform a
//! periodic test of the oscillator source.
//!
//! \return If the elapsed number of ticks is not within the min and max
//! boundaries, the function returns \b STL_OSC_CT_FAIL. Otherwise, it
//! returns \b STL_OSC_CT_PASS.
//!
//
//*****************************************************************************
extern uint16_t
STL_OSC_CT_stopTest(const STL_OSC_CT_Handle oscTimer2Handle);

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
}
#endif

#endif  //  STL_OSC_CT_H

//
// End of File
//
