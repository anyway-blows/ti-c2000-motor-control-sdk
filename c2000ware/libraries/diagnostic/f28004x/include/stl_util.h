//#############################################################################
//
// FILE:  stl_util.h
//
// TITLE: Diagnostic Library Utility software module header
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

#ifndef STL_UTIL_H
#define STL_UTIL_H

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
#include "sysctl.h"

//*****************************************************************************
//
//! \addtogroup stl_util Utilities API Functions
//!
//! The code for this module is contained in <tt>source/stl_util.c</tt> and
//! <tt>source/stl_util_s.asm</tt>, with <tt>include/stl_util.h</tt> containing
//! the API declarations for use by applications.
//!
//! @{
//
//*****************************************************************************

//
// Defines
//
#define STL_UTIL_CPU_RATE   10.00f   // for a 100MHz CPU clock speed (SYSCLKOUT)
//#define STL_UTIL_CPU_RATE   11.111f   // for a 90MHz CPU clock speed (SYSCLKOUT)
//#define STL_UTIL_CPU_RATE   12.500f   // for a 80MHz CPU clock speed (SYSCLKOUT)
//#define STL_UTIL_CPU_RATE   16.667f   // for a 60MHz CPU clock speed (SYSCLKOUT)
//#define STL_UTIL_CPU_RATE   20.000f   // for a 50MHz CPU clock speed (SYSCLKOUT)
//#define STL_UTIL_CPU_RATE   25.000f   // for a 40MHz CPU clock speed (SYSCLKOUT)
//#define STL_UTIL_CPU_RATE   33.333f   // for a 30MHz CPU clock speed (SYSCLKOUT)
//#define STL_UTIL_CPU_RATE   41.667f   // for a 24MHz CPU clock speed (SYSCLKOUT)
//#define STL_UTIL_CPU_RATE   50.000f   // for a 20MHz CPU clock speed (SYSCLKOUT)
//#define STL_UTIL_CPU_RATE   66.667f   // for a 15MHz CPU clock speed (SYSCLKOUT)
//#define STL_UTIL_CPU_RATE  100.000f   // for a 10MHz CPU clock speed (SYSCLKOUT)

//
// Typedefs
//

//! Values to be passed to STL_Util_setErrorFlag() and
//! STL_Util_clearErrorFlag(). These correspond to different errors in the STL.
//
typedef enum
{
    //! CRC check
    STL_UTIL_CRC                        = 0x00000001UL,
    //! CPU register test
    STL_UTIL_CPU_REG                    = 0x00000002UL,
    //! FPU register test
    STL_UTIL_FPU_REG                    = 0x00000004UL,
    //! VCU register test
    STL_UTIL_VCU_REG                    = 0x00000008UL,
    //! PIE RAM
    STL_UTIL_PIE_RAM_MISMATCH           = 0x00000010UL,
    //! PIE RAM handler failed to execute
    STL_UTIL_PIE_RAM_INT                = 0x00000020UL,
    //! OSC timer 2
    STL_UTIL_OSC_TIMER2                 = 0x00000040UL,
    //! MEP out of range
    STL_UTIL_OSC_HR_MEP_RANGE           = 0x00000080UL,
    //! SFO calibration error
    STL_UTIL_OSC_HR_SFO                 = 0x00000100UL,
    //! SFO delay error
    STL_UTIL_OSC_HR_DELAY               = 0x00000200UL,
    //! March RAM test
    STL_UTIL_MARCH                      = 0x00000400UL,
    //! March test for CAN message RAM
    STL_UTIL_CAN_RAM_PARITY             = 0x00000800UL
} STL_Util_ErrorFlag;

//*****************************************************************************
//
//! Delay for a specified number of microseconds
//!
//! \param microseconds is the number of microseconds to delay.
//!
//! This function calls SysCtl_delay() to achieve a delay in microseconds. The
//! function will convert the desired delay in microseconds to the count value
//! expected by the function. \e microseconds is the number of microseconds to
//! delay.
//!
//! \note If this function does not get inlined (for instance, if the optimizer
//! is turned off) the delay will be made less accurate by the overhead of the
//! additional function call.
//!
//! \return None.
//*****************************************************************************
#ifndef __cplusplus
#pragma CODE_SECTION(STL_Util_delayUS, ".TI.ramfunc")
#else
#pragma CODE_SECTION(".TI.ramfunc");
#endif
static inline void STL_Util_delayUS(uint32_t microseconds)
{
    uint32_t delay = ((((float32_t)microseconds * 1000.0f) /
                       (float32_t)STL_UTIL_CPU_RATE) - 9.0f) / 5.0f;

    SysCtl_delay(delay);
}

//*****************************************************************************
//
//! Sets a global error flag.
//!
//! \param errorFlag is a STL_Util_ErrorFlag that will be set in
//! globalErrorFlags.
//!
//! This function sets an error flag in the global globalErrorFlags.
//!
//! \return None.
//
//*****************************************************************************
extern void STL_Util_setErrorFlag(const STL_Util_ErrorFlag errorFlag);

//*****************************************************************************
//
//! Gets the error flag status of globalErrorFlags.
//!
//! This function returns the global globalErrorFlags.
//!
//! \return Returns the global globalErrorFlags.
//
//*****************************************************************************
extern uint32_t STL_Util_getErrorFlag(void);

//*****************************************************************************
//
//! Clears a flag of globalErrorFlags.
//!
//! \param errorFlag is a STL_Util_ErrorFlag that will be cleared in
//! globalErrorFlags.
//!
//! This function clears a flag of globalErrorFlags.
//!
//! \return None.
//
//*****************************************************************************
extern void STL_Util_clearErrorFlag(const STL_Util_ErrorFlag errorFlag);

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

#endif  // STL_UTIL_H

//
// End of File
//

