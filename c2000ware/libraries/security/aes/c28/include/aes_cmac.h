//###########################################################################
//
// FILE:   aes_cmac.h
//
// TITLE:  AES 256 bit CMAC mode header file
//
//###########################################################################
// $TI Release: C2000 AES Software v1.00.00.00 $
// $Release Date: Fri Feb 12 19:23:22 IST 2021 $
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

#ifndef AES_CMAC_H
#define AES_CMAC_H

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

//*****************************************************************************
//
//! \addtogroup aes_cmac_api AES CMAC API
//! @{
//
//*****************************************************************************
#include <stdint.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_dcsm.h"
#include "aes_ecb.h"

//*****************************************************************************
//
//! Perform 256-bit AES CMAC Authentication
//!
//! \param cmacKey is the 256-bit CMAC key
//! \param startAddress is the memory address the CMAC will start from
//! \param endAddress is the memory address the CMAC will end on
//! \param tagAddress is the memory address where the golden CMAC tag is
//!        stored
//!
//! This function performs 256-Bit AES Cipher-based Message Authentication
//! (CMAC) on the memory range provided. The calculated CMAC tag is then
//! compared to the golden CMAC tag at the specified memory location.
//!
//! - \b Note: Address range must align to 128-bit boundary. This implementation
//!            won't apply padding.
//! - \b Note: The golden CMAC tag can be within or outside the CMAC memory
//!            region. If it is within the memory region, the algorithm will
//!            read that memory as all ones.
//!
//! \return
//! - \b 0xFFFFFFFF = Calculated CMAC tag doesn't match golden CMAC tag
//! - \b 0xA5A5A5A5 = Memory range provided isn't aligned to 128-bit
//!                   boundary or length is zero
//! - \b 0x00000000 = Calculated CMAC tag matched golden tag
//
//*****************************************************************************
extern uint32_t AES256_performCMAC(uint16_t *cmacKey, uint32_t startAddress,
                                   uint32_t endAddress, uint32_t tagAddress);

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif //AES_CMAC_H
