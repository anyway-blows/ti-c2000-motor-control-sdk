//#############################################################################
//
// FILE: stl_can_ram.h
//
// TITLE: Diagnostic Library CAN message RAM test software module header
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

#ifndef STL_CAN_RAM_H
#define STL_CAN_RAM_H

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
#include "stl_crc.h"
#include "stl_march.h"
#include "can.h"

//*****************************************************************************
//
//!
//! \addtogroup stl_can_ram CAN Message RAM Test API Functions
//!
//! The code for this module is contained in <tt>source/stl_can_ram.c</tt> and
//! <tt>source/stl_can_ram_s.asm</tt>, with <tt>include/stl_can_ram.h</tt>
//! containing the API declarations for use by applications.
//!
//! # Error Injection #
//!
//! Here are techniques to inject error:
//!
//! \b STL_CAN_RAM_testRAM(), STL_CAN_RAM_checkErrorStatus()
//! - Call STL_CAN_RAM_injectError() with an \e xorMask value that will
//!   cause errors.
//!
//! @{
//
//*****************************************************************************

//
// Defines
//
#define STL_CAN_RAM_PASS            0U
#define STL_CAN_RAM_ERROR           1U

//
// Values for address calculation and checking
//
#define STL_CAN_RAM_ADDR_OFFSET     0x1000U // Offset from CAN base address to
                                            // message RAM start
#define STL_CAN_RAM_MAX_ADDR_OFFSET 0x13F1U // Offset from CAN base address to
                                            // message RAM end (in RDA mode)

//
// Key to turn off parity checking
//
#define STL_CAN_RAM_PARITY_OFF      (5U << CAN_CTL_PMD_S)

//
//! Value to indicate to STL_CAN_RAM_testRAM() through the \b copyAddress
//! parameter that the test should run a destructive test on the region of CAN
//! message RAM without saving and restoring its contents.
//
#define STL_CAN_RAM_NO_COPY         0xFFFFFFFFU

//
// Prototypes
//
//*****************************************************************************
//
//! \brief Performs a March13N memory test on the specified range of CAN
//! message RAM objects.
//!
//! \param canBase is the base address of the CAN controller.
//! \param pattern is the test pattern to use.
//! \param startAddress is the starting address (inclusive) of the CAN message
//! RAM range to test.
//! \param endAddress is the end address (inclusive) of the CAN message RAM
//! range to test.
//! \param copyAddress is the address to copy the original contents of the
//! memory under test. It will be used to restore the original memory at the
//! end of the March13N memory test. If no save and restore is required, use
//! a value of \b STL_CAN_RAM_NO_COPY.
//!
//! This function performs a March13N memory test on the range of CAN message
//! RAM objects specified by \b canBase, \b startAddress and \b endAddress.
//! The test can save and restore the original contents of the message RAM
//! by passing an address to a back up buffer through the \b copyAddress
//! parameter. The test will copy the original contents of the memory to
//! \b copyAddress, perform the memory test, and then copy the original
//! contents back to the memory under test. To skip the save and restore, use
//! a value of \b STL_CAN_RAM_NO_COPY for the \b copyAddress and a destructive
//! test will be performed instead.
//!
//! The test patterns and the March13N memory test algorithm provided test
//! the memory for stuck-at-faults. The parity bits will show if any were
//! detected. Use STL_CAN_RAM_checkErrorStatus() to read the parity status.
//!
//! \note Note to take care calculating the size of memory needed for
//! \b copyAddress. The March13N function is not specific to the CAN message
//! RAM and does not take the byte addressability of the memory into account.
//! Use a buffer twice the size of the actual message RAM region you are
//! testing.
//!
//! \note The STL_March functions called by this test disable global CPU
//! interrupts (DINT) and then re-enable them after the test has completed.
//!
//! \return None.
//
//*****************************************************************************
extern void STL_CAN_RAM_testRAM(const uint32_t canBase,
                                const STL_March_Pattern pattern,
                                const uint32_t startAddress,
                                const uint32_t endAddress,
                                const uint32_t copyAddress);

//*****************************************************************************
//
//! \brief Injects an error into a CAN message RAM address.
//!
//! \param canBase is the base address of the CAN controller.
//! \param address is the address of the word in the message RAM where the
//! error will be injected.
//! \param xorMask mask of the bit to flip in \b address.
//!
//! This function injects an error at a specific memory \b address. \b xorMask
//! specifies which bit to flip in order to corrupt the data or parity bits.
//!
//! \return None.
//
//*****************************************************************************
extern void STL_CAN_RAM_injectError(const uint32_t canBase,
                                    const uint32_t address,
                                    const uint32_t xorMask);

//*****************************************************************************
//
//! \brief Returns the status of the CAN message RAM parity error detected bit.
//!
//! \param canBase is the base address of the CAN controller.
//!
//! This function checks if any parity errors have been detected by the CAN
//! message RAM parity logic.
//!
//! \return If the CAN error status register indicates a parity error the
//! function returns \b STL_CAN_RAM_ERROR. Otherwise, the function returns
//! \b STL_CAN_RAM_PASS.
//!
//! \note Several bits in the CAN error status register (CAN_ES) are cleared
//! on a CPU read of the register which this function performs. If the
//! potential clearing of these bits (see device Technical Reference Manual for
//! details) is undesired behavior for your application, use different means of
//! reading the status.
//
//*****************************************************************************
extern uint16_t STL_CAN_RAM_checkErrorStatus(const uint32_t canBase);

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

#endif  //  STL_CAN_RAM_H

//
// End of File
//
