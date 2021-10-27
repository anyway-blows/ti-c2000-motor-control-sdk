//###########################################################################
//
// FILE:   aes_ecb.h
//
// TITLE:  AES 256 bit ECB mode header file
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

#ifndef AES_ECB_H
#define AES_ECB_H

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
//! \addtogroup aes_ecb_api AES ECB API
//! @{
//
//*****************************************************************************
#include <stdint.h>

//*****************************************************************************
//
//! MACRO for Base ECB implementation
//
//*****************************************************************************

#ifdef AES_BASE_ECB
#define AES256_encrypt(key, text) AES256_performECB(AES_OPMODE_ENCRYPT, key, text)
#define AES256_decrypt(key, text) AES256_performECB(AES_OPMODE_DECRYPT, key, text)
#endif

//*****************************************************************************
//
//! Modes that can be used in the encryption/decryption functions
//
//*****************************************************************************
typedef enum
{
     AES_OPMODE_ENCRYPT,   //!< Encryption mode
     AES_OPMODE_DECRYPT    //!< Decryption mode
} AES_OperationMode;

//*****************************************************************************
//
//! Encrypts and decrypts a 128 bit message with a 256 bit key in ECB mode.
//!
//! \param mode is the AES operation mode of encryption or decryption
//! \param key the 256-bit AES key
//! \param text is the input plaintext or ciphertext (length of 128-bits) and
//!  once operation is complete, this parameter contains the output plaintext
//!  or ciphertext
//!
//! This function performs 256-bit Electronic Codebook (ECB) mode encryption
//! or decryption on set of 128-bit plaintext/ciphertext
//!
//! \return None.
//
//*****************************************************************************
extern void AES256_performECB(AES_OperationMode mode, uint16_t *key,
                              uint16_t *text);

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

#endif //AES_ECB_H
