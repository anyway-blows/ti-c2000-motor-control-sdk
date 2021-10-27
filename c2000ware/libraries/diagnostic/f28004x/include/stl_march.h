//#############################################################################
//
// FILE: stl_march.h
//
// TITLE: Diagnostic Library March13N software module header
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

#ifndef STL_MARCH_H
#define STL_MARCH_H

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
#include "memcfg.h"

//*****************************************************************************
//
//!
//! \addtogroup stl_march March13N Test API Functions
//!
//! The code for this module is contained in <tt>source/stl_march.c</tt> and
//! <tt>source/stl_march_s.asm</tt>, with <tt>include/stl_march.h</tt>
//! containing the API declarations for use by applications.
//!
//! # Error Injection #
//!
//! Here are techniques to inject error:
//!
//! \b STL_March_testRAM(), \b STL_March_testRAMCopy()
//! - Call STL_March_injectError() with testMode and xorMask values that cause
//!   errors.
//!
//! @{
//
//*****************************************************************************

//
// Defines
//
#define STL_MARCH_PASS                0U
#define STL_MARCH_CORR_ERROR          1U
#define STL_MARCH_UNC_ERROR           2U
#define STL_MARCH_BOTH_ERROR          3U

//
// Typedefs
//

//
//! Values that must be used to pass to determine the test pattern for
//! STL_March_testRAMCopy() and STL_March_testRAM()
//!
typedef enum
{
    STL_MARCH_PATTERN_ONE    = 0x96966969U,   //!< Test Pattern One
    STL_MARCH_PATTERN_TWO    = 0x0000FFFEU,   //!< Test Pattern Two
    STL_MARCH_PATTERN_THREE  = 0x2AAA5555U,   //!< Test Pattern Three
    STL_MARCH_PATTERN_FOUR   = 0xCC3723CCU    //!< Test Pattern Four
} STL_March_Pattern;

//
//! \brief Defines the March memory test inject error object.
//!
typedef struct
{
    uint32_t            address;            //!< Address (32-bit aligned)
    uint32_t            ramSection;         //!< RAM section identifier
    uint32_t            xorMask;            //!< Mask to flip bits in test mode
    MemCfg_TestMode     testMode;           //!< Mode in which to inject error
} STL_March_InjectErrorObj;

//
//! Defines the RAM error logic test handle
//!
typedef STL_March_InjectErrorObj * STL_March_InjectErrorHandle;

//
// Prototypes
//

//*****************************************************************************
//
//! \brief Performs a March13N non-destructive memory test on the specified
//! RAM memory.
//!
//! \param pattern is the test pattern to use.
//! \param startAddress is the address to start the memory test.
//! \param length is the number of 32-bit words of the memory test minus 1.
//! \param copyAddress is the address to copy the original contents of the
//! memory under test. It will be used to restore the original memory at the
//! end of the March13N memory test.
//!
//! This function performs a March13N memory test on RAM specified by
//! \b startAddress and \b length. This function performs a non-destructive
//! memory test. This means that it will begin by copying the original contents
//! of the memory to \b copyAddress, perform the memory test, and then copy
//! the original contents back to the memory under test. The test patterns
//! along with the March13N memory test algorithm provided, test memory for
//! stuck-at-faults as well as boundary cases including worst case timings
//! tailored for the C2000 RAM bank architecture.
//!
//! This test is implemented to be able to perform a memory test on any section
//! of RAM including the stack.
//!
//! \note If this code is running from RAM, be careful not to perform this
//! memory test on itself, meaning do not perform the March13N memory test on
//! the March13N program code in RAM. This will likely lead to an ITRAP. In
//! order to test the program code for this March13N algorithm, the user can
//! create a copy of this function in RAM or flash and run the memory test code
//! from the copy.
//!
//! \note This function disables global CPU interrupts (DINT) and then
//! re-enables them after the test has completed.
//!
//! \note \b length is the number of 32-bits words to test minus 1. For
//! example, in order to test 8 32-bit words, length is 7.
//!
//! \return None.
//
//*****************************************************************************
extern void STL_March_testRAMCopy(const STL_March_Pattern pattern,
                                  const uint32_t startAddress,
                                  const uint32_t length,
                                  const uint32_t copyAddress);

//*****************************************************************************
//
//! \brief Performs a March13N destructive memory test on the specified RAM
//! memory.
//!
//! \param pattern is the test pattern to use.
//! \param startAddress is the address to start the memory test.
//! \param length is the number of 32-bit words of the memory test minus 1.
//!
//! This function performs a March13N memory test on RAM specified by
//! \b startAddress and \b length. This test performs a destructive memory
//! test, meaning the original contents will be lost by this test. The test
//! patterns along with the March13N memory test algorithm provided, test
//! memory for stuck-at-faults as well as boundary cases including worst case
//! timings tailored for the C2000 RAM bank architecture.
//!
//! \note If this code is running from RAM, be careful not to perform this
//! memory test on itself, meaning do not perform the March13N memory test on
//! the March13N program code in RAM. This will likely lead to an ITRAP. In
//! order to test the program code for this March13N algorithm, the user can
//! create a copy of this function in RAM or flash and run the memory test code
//! from the copy.
//!
//! \note This function disables global CPU interrupts (DINT) and then
//! re-enables them after the test has completed.
//!
//! \note \b length is the number of 32-bits words to test minus 1. For
//! example, in order to test 8 32-bit words, length is 7.
//!
//! \return None.
//
//*****************************************************************************
extern void STL_March_testRAM(const STL_March_Pattern pattern,
                              const uint32_t startAddress,
                              const uint32_t length);

//*****************************************************************************
//
//! \brief Injects an error into a RAM memory address.
//!
//! \param errorHandle the inject error handle specifying where and what type
//! of error to inject into RAM.
//!
//! This function injects an error at a specific memory \b address in a
//! specific \b ramSection. \b testMode specifies whether the error will be
//! injected in the data or ECC/parity bits. \b xorMask specifies which bit
//! or bits to flip in order to corrupt either the data or ECC/parity bits.
//!
//! \return None.
//
//*****************************************************************************
extern void STL_March_injectError(const STL_March_InjectErrorHandle
                                  errorHandle);

//*****************************************************************************
//
//! \brief Returns the status of the Memory Error Registers for RAM.
//!
//! This function checks if there are any correctable or uncorrectable errors
//! indicated by the Memory Error Registers and returns the status.
//!
//! \return If the Memory Error Registers indicate a correctable error
//! and/or an uncorrectable error in RAM, then the function returns
//! \b STL_MARCH_CORR_ERROR, \b STL_MARCH_UNC_ERROR, or \b STL_MARCH_BOTH_ERROR.
//! Otherwise, the function returns \b STL_MARCH_PASS.
//
//*****************************************************************************
extern uint16_t STL_March_checkErrorStatus(void);

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

#endif  //  STL_MARCH_H

//
// End of File
//
