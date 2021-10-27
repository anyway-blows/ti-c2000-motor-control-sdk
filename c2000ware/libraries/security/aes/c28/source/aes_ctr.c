//###########################################################################
//
// FILE:   aes_ctr.c
//
// TITLE:  Implementation of the AES-256 CTR mode.
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

#include "aes_ctr.h"

//*****************************************************************************
//
// CTR_splitText - Split text into 128 bit blocks based on blockNumber
//
//*****************************************************************************
void CTR_splitText(uint16_t *text, uint16_t *output, uint16_t blockNumber)
{
    int16_t i;
    
    for(i = 0; i < 16; i++)
    {
        output[i] = text[(16 * blockNumber) + i];
    }
}

//*****************************************************************************
//
// CTR_perform128bitXOR - Perform XOR operation on two 16 byte arrays
//
//*****************************************************************************
void CTR_perform128bitXOR(uint16_t *input1, uint16_t *input2, uint16_t *out)
{
    int16_t i;

    for(i = 0; i < 16; i++)
    {
        out[i] = input1[i] ^ input2[i]; 
    }
}

//*****************************************************************************
//
// CTR_move128bitArray - Move input array into output array
//
//*****************************************************************************
void CTR_move128bitArray(uint16_t *input, uint16_t *output)
{
    int16_t i;

    for(i = 0; i < 16; i++)
    {
        output[i] = input[i];
    }
}

//*****************************************************************************
//
// CTR_move256bitArray - Move input array into output array
//
//*****************************************************************************
void CTR_move256bitArray(uint16_t *input, uint16_t *output)
{
    int16_t i;

    for(i = 0; i < 32; i++)
    {
        output[i] = input[i];
    }
}

//*****************************************************************************
//
// CTR_incrementCounter - Increment nonceCounter for next round 
//
//*****************************************************************************
void CTR_incrementCounter(uint16_t *input)
{
    int16_t i = 0;

    input[15] = input[15] + 1;

    //
    // If input[15] overflows, carry the overflow into more significant bytes
    // until there is no more overflow. If the input is 0xFFFFFFFFFFFFFFFF,
    // it will become 0x0000000000000000 when incremented.
    //
    for(i = 15; i >= 0; i--)
    {
        if(((input[i] & 0x100) >> 8) == 1)
        {
            if(i != 0)
            {
                input[i - 1] = input[i - 1] + 1;
            }
            input[i] = 0x00;
        }
    }
}

//*****************************************************************************
//
// CTR_mergeText - Merge final plaintext/ciphertext into text array 
//
//*****************************************************************************
void CTR_mergeText(uint16_t *text, uint16_t *input, uint16_t blockNumber)
{
    int16_t i;

    for(i = 0; i < 16; i++)
    {
        text[(16 * blockNumber) + i]  = input[i];
    }
}

//*****************************************************************************
//
// AES256_performCTR
//
//*****************************************************************************
void AES256_performCTR(uint16_t *key, uint16_t *text, uint16_t length, 
                       uint16_t *nonceCounter)
{
    //
    // Initialize local variables
    //
    uint16_t roundKey[32];
    uint16_t i;
    uint16_t roundText[16];
    uint16_t encryptionInput[16];
    
    //
    // main loop
    // Encryption/Decryption order of operations: get block of text, encrypt
    //                                               nonceCounter, XOR with text,
    //                                              update text
    // 
    //
    for(i = 0; i < length; i++)
    {    
        //
        // Get 128 bit block of plaintext/ciphertext from array passed in
        //
        CTR_splitText(text, roundText, i);
        
        //
        // Reset key for encryption
        //
        CTR_move256bitArray(key, roundKey);
        
        //
        // Set encryption input for round equal to round counter
        //
        CTR_move128bitArray(nonceCounter, encryptionInput);

        //
        // AES ECB encryption
        //
        AES256_encrypt(roundKey, encryptionInput);
        
        //
        // XOR ciphertext with new plaintext/ciphertext to get final 
        // ciphertext/plaintext
        //
        CTR_perform128bitXOR(encryptionInput, roundText, roundText);
        
        // 
        // Increment counter for next round
        //
        CTR_incrementCounter(nonceCounter);
        
        //
        // Move the final plaintext/ciphertext into text
        //
        CTR_mergeText(text, roundText , i);
    }
}                           
//
// End of File
//
