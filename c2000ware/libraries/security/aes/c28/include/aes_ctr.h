//###########################################################################
//
// FILE:   aes_ctr.h
//
// TITLE:  AES 256 bit CTR mode header file
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

#ifndef AES_CTR_H
#define AES_CTR_H

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
//! \addtogroup aes_ctr_api AES CTR API
//! @{
//
//*****************************************************************************
#include <stdint.h>
#include "aes_ecb.h"

//*****************************************************************************
//
//! Perform 256-bit AES CTR Encryption/Decryption
//!
//! \param key is the 256-bit AES key
//! \param text is the input plaintext or ciphertext and once operation is
//!        complete, this parameter contains the output plaintext or ciphertext
//! \param length is the length of the input data in blocks (block size is
//!        128-bits)
//! \param nonceCounter is the required number-used-once 128-bit counter
//!        value
//!
//! This function performs 256-bit AES Counter (CTR) mode encryption or
//! decryption on the requested number of 128-bit input blocks.
//!
//! \return None.
//
//*****************************************************************************
extern void AES256_performCTR(uint16_t *key, uint16_t *text, uint16_t length,
                              uint16_t *nonceCounter);

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

#endif //AES_CTR_H
