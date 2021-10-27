//#############################################################################
//
// FILE:  stl_crc.h
//
// TITLE: Diagnostic Library CRC software module header
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

#ifndef STL_CRC_H
#define STL_CRC_H

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
#include "inc/hw_types.h"
#include "stl_util.h"

//*****************************************************************************
//
//! \addtogroup stl_crc CRC API Functions
//!
//! The code for this module is contained in <tt>source/stl_crc.c</tt>
//! and <tt>source/stl_crc_s.asm</tt> (F2837x only), with
//! <tt>include/stl_crc.h</tt> containing the API declarations for use by
//! applications.
//!
//! @{
//
//*****************************************************************************

//
// Defines
//
#define STL_CRC_INIT_CRC        0x0UL          //!< Initial CRC Register Value
#define STL_CRC_PASS            0x0U
#define STL_CRC_FAIL            0x1U

//
// Typedefs
//
//! \brief Parity enumeration
//!
//! The parity is used by the CRC algorithm to determine whether to begin
//! calculations from the low byte (EVEN) or from the high byte (ODD) of the
//! first word (16-bit) in the message. \n
//! For example, if your message had 10 bytes and started at the address 0x8000
//! but the first byte was at the high byte position of the first 16-bit word,
//! the user would call the CRC function with odd parity i.e.
//! STL_CRC_PARITY_ODD \n
//! Address: HI  LO \n
//! 0x8000 : B0  XX \n
//! 0x8001 : B2  B1 \n
//! 0x8002 : B4  B3 \n
//! 0x8003 : B6  B5 \n
//! 0x8004 : B8  B7 \n
//! 0x8005 : XX  B9 \n
//! However, if the first byte was at the low byte position of the first 16-bit
//! word, the user would call the CRC function with even parity i.e.
//! STL_CRC_PARITY_EVEN \n
//! Address: HI  LO \n
//! 0x8000 : B1  B0 \n
//! 0x8001 : B3  B2 \n
//! 0x8002 : B5  B4 \n
//! 0x8003 : B7  B6 \n
//! 0x8004 : B9  B8 \n
//
typedef enum
{
    STL_CRC_PARITY_EVEN = 0U,   //!< Even parity, CRC starts at the low byte of
                                //!< the first word (16-bit)
    STL_CRC_PARITY_ODD  = 1U    //!< Odd parity, CRC starts at the high byte of
                                //!< the first word (16-bit)
} STL_CRC_Parity;

//! \brief CRC structure
//!
typedef struct
{
    uint32_t          seedValue;    //!< Initial value of the CRC calculation
    uint16_t          numBytes;     //!< Number of bytes in the message buffer
    STL_CRC_Parity    parity;       //!< Start the CRC from the low byte
                                    //!< or high byte
    uint32_t          crcResult;    //!< The calculated CRC
    void *            msgBuffer;    //!< Pointer to the message buffer
} STL_CRC_Obj;

//! \brief Handle to the CRC structure
//!
typedef STL_CRC_Obj *STL_CRC_Handle;

//
// Prototypes
//

//*****************************************************************************
//
//! \brief Workaround to the silicon issue of first VCU calculation on power up
//! being erroneous
//!
//! Due to the internal power-up state of the VCU module, it is possible that
//! the first CRC result will be incorrect. This condition applies to the
//! first result from each of the eight CRC instructions. This rare condition
//! can only occur after a power-on reset, but will not necessarily occur on
//! every power on. A warm reset will not cause this condition to reappear.
//!
//! Workaround(s): The application can reset the internal VCU CRC logic by
//! performing a CRC calculation of a single byte in the initialization
//! routine. This routine only needs to perform one CRC calculation and can use
//! any of the CRC instructions.
//!
//! \return None.
//
//*****************************************************************************
extern void STL_CRC_reset(void);

//*****************************************************************************
//
//! \brief Runs the 32-bit CRC routine using polynomial 0x04c11db7
//!
//! \param crcHandle handle to the CRC object
//!
//! Calculates the 32-bit CRC using polynomial 0x04c11db7 on the VCU or VCRC.
//! Depending on the parity chosen the CRC begins at either
//! the low byte (STL_CRC_PARITY_EVEN) or the high byte (STL_CRC_PARITY_ODD)
//! of the first word (16-bit).
//!
//! \note The size of the message (bytes) is limited to 65535 bytes.
//!
//! \return None.
//
//*****************************************************************************
extern void STL_CRC_calculate(const STL_CRC_Handle crcHandle);

//*****************************************************************************
//
//! \brief Runs the 32-bit CRC routine using polynomial 0x04c11db7
//!
//! \param crcHandle handle to the CRC object
//!
//! Calculates the 32-bit CRC using polynomial 0x04c11db7 on the VCU or VCRC.
//! This algorithm performs a CRC32 only on the low bytes (LSB) of each
//! 16-bit word. The input \b parity has no effect. This function works on
//! unpacked data.
//!
//! \note The size of the message (bytes) is limited to 65535 bytes.
//!
//! \return None.
//
//*****************************************************************************
extern void STL_CRC_calculateLowBytes(const STL_CRC_Handle crcHandle);

//*****************************************************************************
//
//! \brief Calculates a CRC-32 value for specific memory range and compares it
//! with the goldenCRC value.
//!
//! \param startAddress - start address of CRC calculation.
//! \param endAddress - end address of CRC calculation, inclusive.
//! \param goldenCRC - golden CRC value.
//!
//! This function performs a test of the memory range by calculating the
//! CRC-32 value for the input memory range and comparing it with the golden
//! CRC value.
//!
//! \note This function could be used with many memory types including
//! Flash and Boot ROM.
//!
//! \return If the calculated CRC matches the golden CRC, then the function
//! returns \b STL_CRC_PASS. Otherwise, it returns \b STL_CRC_FAIL.
//
//*****************************************************************************
extern uint16_t STL_CRC_checkCRC(const uint32_t startAddress,
                                 const uint32_t endAddress,
                                 const uint32_t goldenCRC);

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

#endif // STL_CRC_H

//
// End of File
//
