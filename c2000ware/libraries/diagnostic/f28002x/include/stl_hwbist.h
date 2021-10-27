//#############################################################################
//
// FILE:  stl_hwbist.h
//
// TITLE: Diagnostic Library HWBIST software module header
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

#ifndef STL_HWBIST_H
#define STL_HWBIST_H

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
#include "stl_util.h"
#include "cpu.h"
#include "debug.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"

//*****************************************************************************
//
//! \addtogroup stl_hwbist HWBIST API Functions
//!
//! The code for this module is contained in <tt>source/stl_hwbist.c</tt> and
//! <tt>source/stl_hwbist_s.asm</tt>, with <tt>include/stl_hwbist.h</tt>
//! containing the API declarations for use by applications.
//!
//! # Error Injection #
//!
//! Here are techniques to inject error:
//!
//! STL_HWBIST_runFull()
//! - Call STL_HWBIST_injectError() with a parameter of STL_HWBIST_Error type.
//!
//! @{
//
//*****************************************************************************

//
// Defines
//
#define STL_HWBIST_BIST_DONE            0x0001U
#define STL_HWBIST_MACRO_DONE           0x0002U
#define STL_HWBIST_NMI                  0x0004U
#define STL_HWBIST_BIST_FAIL            0x0008U
#define STL_HWBIST_INT_COMP_FAIL        0x0010U
#define STL_HWBIST_TO_FAIL              0x0020U
#define STL_HWBIST_OVERRUN_FAIL         0x0040U

#define HWBIST_O_CSTCGCR0               0x0U
#define HWBIST_O_CSTCGCR1               0x4U
#define HWBIST_O_CSTCGCR2               0x8U
#define HWBIST_O_CSTCGCR3               0xCU
#define HWBIST_O_CSTCGCR4               0x10U
#define HWBIST_O_CSTCGCR5               0x14U
#define HWBIST_O_CSTCGCR6               0x18U
#define HWBIST_O_CSTCGCR7               0x1CU
#define HWBIST_O_CSTCGCR8               0x20U
#define HWBIST_O_CSTCPCNT               0x24U
#define HWBIST_O_CSTCCONFIG             0x28U
#define HWBIST_O_CSTCSADDR              0x2CU
#define HWBIST_O_CSTCTEST               0x30U
#define HWBIST_O_CSTCRET                0x34U
#define HWBIST_O_CSTCCRD                0x38U
#define HWBIST_O_CSTGSTAT               0x40U
#define HWBIST_O_CSTCCPCR               0x48U
#define HWBIST_O_CSTCCADDR              0x4CU
#define HWBIST_O_CSTCSEM                0xA0U
#define HWBIST_CSTCSEM_SEMAPHORE        0x03U

#define PIEVECTTABLE_O_NMI              ((INT_NMI >> 16) * 2U)

//
// Macros to isolate inline assembly
//
#define STL_HWBIST_REF_STACK            __asm(" .ref __stack")
#define STL_HWBIST_MOV_SP_STACK         __asm(" MOV SP, #__stack")
#define STL_HWBIST_C28OBJ               __asm(" C28OBJ")
#define STL_HWBIST_C28ADDR              __asm(" C28ADDR")
#define STL_HWBIST_C28MAP               __asm(" C28MAP")
#define STL_HWBIST_CLRC_PAGE0           __asm(" CLRC PAGE0")
#define STL_HWBIST_MOVW_DP_0            __asm(" MOVW DP,#0")
#define STL_HWBIST_CLRC_OVM             __asm(" CLRC OVM")
#define STL_HWBIST_SPM_0                __asm(" SPM 0")
#define STL_HWBIST_REF_HANDLE_RESET_FXN __asm(" .ref STL_HWBIST_handleReset")
#define STL_HWBIST_LCR_HANDLE_RESET_FXN __asm(" LCR STL_HWBIST_handleReset")

//
// Typedefs
//

//
//! Values that must be used as parameter to STL_HWBIST_injectError() and
//! STL_HWBIST_runFull() in order to specify the type of error to inject
//! before executing HWBIST.
//
typedef enum
{
    STL_HWBIST_NO_ERROR      = 0x0000,    //!< No error
    STL_HWBIST_TIMEOUT       = 0x000A,    //!< Time-out error
    STL_HWBIST_FINAL_COMPARE = 0x00A0,    //!< Final MISR compare error
    STL_HWBIST_NMI_TRAP      = 0x0A00,    //!< NMI trap error
    STL_HWBIST_LOGIC_FAULT   = 0x2000     //!< Logic error
} STL_HWBIST_Error;

//
//! Values that must be used as a parameter to STL_HWBIST_init() in order to
//! initialize the HWBIST engine before a micro-run for a specific target
//! coverage. Note that not all devices support multiple coverage levels.
//
typedef enum
{
    STL_HWBIST_90_LOS        = 0x0001    //!< 90% launch-on-shift
} STL_HWBIST_Coverage;

//
// Prototypes
//
__interrupt void STL_HWBIST_errorNMIISR(void);

//*****************************************************************************
//
//! \brief Performs a hardware built-in self-test of the CPU under test.
//!
//! \param errorType is an enumerated type \b STL_HWBIST_Error which specifies
//! the type of error to inject before executing a full run of HWBIST test.
//!
//! This function initializes the HWBIST engine and then injects the
//! \b errorType. It also registers the STL_HWBIST_NMIISR as the NMI vector.
//! It then performs a full hardware built-in self-test achieving the 90%
//! launch-on-shift coverage after 750 micro-runs. If there is a failure
//! in the HWBIST, then a global error flag will be set and the return value
//! will specify a failure. Additionally, if the coverage is not achieved
//! in the expected number of micro-runs, then the test will fail due to an
//! overrun. Before returning, the function will restore the previous NMI
//! vector.
//!
//! \note There is a corner case where a spurious CPU Timer 1 or CPU Timer 2
//! interrupt may be triggered when HWBIST completes. This test contains a
//! workaround which clears the TIE bits of Timer 1 and 2 before starting
//! HWBIST interrupt logging and restores them after HWBIST runs.
//!
//! \return If the HWBIST full run test passes with no errors within the
//! expected number of micro-runs, then this function returns the status of the
//! HWBIST and the value will be a bitwise OR of \b STL_HWBIST_BIST_DONE, and
//! \b STL_HWBIST_MACRO_DONE. If the test fails, then the status of the HWBIST
//! and the return value of the function will have contain a bitwise OR of some
//! combination of the following values: \b STL_HWBIST_NMI,
//! \b STL_HWBIST_BIST_FAIL, STL_HWBIST_INT_COMP_FAIL, \b STL_HWBIST_TO_FAIL,
//! and \b STL_HWBIST_OVERRUN_FAIL.
//
//*****************************************************************************
extern uint16_t STL_HWBIST_runFull(const STL_HWBIST_Error errorType);

//*****************************************************************************
//
//! \brief Performs a  micro-run of the hardware built-in self-test of the
//! CPU under test.
//!
//! This function expects the HWBIST engine to already be initialized with
//! STL_HWBIST_init() before it is called. This function performs a HWBIST
//! micro-run and returns its status. Before returning, the function will also
//! restore the previous NMI vector.
//!
//! In order to achieve 90% coverage, the user needs to initialize the HWBIST
//! controller for 90% launch-on-shift coverage and then execute 750
//! micro-runs. The HWBIST will complete in 750 micro-runs if there are no
//! detected faults.
//!
//! \note This function performs a HWBIST micro-run and is designed to be used
//! as a periodic self-test or PEST.
//!
//! \note There is a corner case where a spurious CPU Timer 1 or CPU Timer 2
//! interrupt may be triggered when HWBIST completes. This test contains a
//! workaround which clears the TIE bits of Timer 1 and 2 before starting
//! HWBIST interrupt logging and restores them after HWBIST runs.
//!
//! \return If the HWBIST micro-run test passes with no errors, then this
//! function returns the status of the HWBIST and the value will be either
//! \b STL_HWBIST_MACRO_DONE or a bitwise OR of \b STL_HWBIST_BIST_DONE, and
//! \b STL_HWBIST_MACRO_DONE. If the test fails, then the status of the HWBIST
//! and the return value of the function will have contain a bitwise OR of some
//! combination of the following values: \b STL_HWBIST_NMI,
//! \b STL_HWBIST_BIST_FAIL, STL_HWBIST_INT_COMP_FAIL, and
//! \b STL_HWBIST_TO_FAIL.
//
//*****************************************************************************
extern uint16_t STL_HWBIST_runMicro(void);

//*****************************************************************************
//
//! \brief Begins the context restore after a CPU reset after a HWBIST
//! micro-run.
//!
//! This function should not be called by the user, but must be placed at the
//! beginning of RAMM0, memory address 0x0000. After a HWBIST micro-run
//! completes, the CPU will reset and begin executing instructions from 0x0000.
//! The user is responsible for placing this function at memory address 0x0000.
//! This can be done using the linker command file and program sections.
//!
//! \return None.
//
//*****************************************************************************
extern void STL_HWBIST_restoreContext(void);

//*****************************************************************************
//
//! \brief Initializes the HWBIST engine for operation.
//!
//! \param coverage is an enumerated type \b STL_HWBIST_Coverage which specifies
//! the coverage to achieve.
//!
//! This function initializes the HWBIST engine for the specified level of
//! coverage. This function is intended to be used with STL_HWBIST_runMicro().
//! This function should be called once to initialize the HWBIST and not called
//! again until the HWBIST is done which is indicated by a return value of
//! \b STL_HWBIST_BIST_DONE as this function resets the HWBIST engine.
//!
//! \return None.
//
//*****************************************************************************
extern void STL_HWBIST_init(const STL_HWBIST_Coverage coverage);

//*****************************************************************************
//
//! \brief Injects an error into the HWBIST engine for operation.
//!
//! \param errorType is an enumerated type \b STL_HWBIST_Error which specifies
//! the error to inject.
//!
//! This function injects an error into the HWBIST using the HWBIST registers.
//!
//! \note This function should be called after STL_HWBIST_init() because
//! STL_HWBIST_init() resets the HWBIST engine and initializes the test.
//!
//! \return None.
//
//*****************************************************************************
#ifndef __cplusplus
#pragma CODE_SECTION(STL_HWBIST_injectError, ".TI.ramfunc");
#else
#pragma CODE_SECTION(".TI.ramfunc");
#endif
static inline
void STL_HWBIST_injectError(const STL_HWBIST_Error errorType)
{
    EALLOW;

    //
    // Inject an error in the HWBIST.
    //
    HWREG(HWBIST_BASE + HWBIST_O_CSTCTEST) = (uint32_t)errorType;
    NOP;
    NOP;

    EDIS;
}

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

#endif // STL_HWBIST_H

//
// End of File
//
