//###########################################################################
//
// FILE:   aes_cmac.c
//
// TITLE:  Implementation of the AES-256 CMAC mode.
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

#include "aes_cmac.h"

//*****************************************************************************
//
// CMAC_perform128bitXOR - Perform XOR operation on two 16 byte arrays
//
//*****************************************************************************
void CMAC_perform128bitXOR(uint16_t *input1, uint16_t *input2, uint16_t *out)
{
    int16_t i;

    for(i = 0U; i < 16U; i++)
    {
        out[i] = input1[i] ^ input2[i];
    }
}

//*****************************************************************************
//
// CMAC_performOneBitLeftshift - Perform left shift on 16 byte array
//
//*****************************************************************************
void CMAC_performOneBitLeftshift(uint16_t *input, uint16_t *output)
{
    int16_t i;
    uint16_t overflow = 0U;

    for(i = 15; i >= 0; i--)
    {
        output[i] = (input[i] << 1U) & 0xFFU;
        output[i] |= overflow;
        overflow = (input[i] & 0x80U)?1U:0U;
    }
}

//*****************************************************************************
//
// CMAC_readMemoryBlock - Reads 128-bits (8 words) and extracts each byte into
//                        a 16-byte array
//
//*****************************************************************************
void CMAC_readMemoryBlock(uint32_t address, uint16_t *output)
{
    uint16_t i;
    uint16_t outputIndex = 0U;
    uint32_t currentAddress = address;

    for(i = 0U; i < 8U; i+= 2)
    {
        currentAddress = address + i;

        output[outputIndex] = (HWREGH(currentAddress) & 0xFF00U) >> 8U;
        output[outputIndex + 1U] = (HWREGH(currentAddress) & 0xFFU);
        output[outputIndex + 2U] = (HWREGH(currentAddress + 0x1UL) & 0xFF00U)
                                   >> 8U;
        output[outputIndex + 3U] = (HWREGH(currentAddress + 0x1UL) & 0xFFU);
        outputIndex+=4U;
    }
}

//*****************************************************************************
//
// CMAC_readMemoryBlockWithTagMask - Reads 128-bits (8 words) and extracts each
//                                   byte into a 16-byte array. If the memory
//                                   range containing the golden tag is read,
//                                   will mask with 0xFs
//
//*****************************************************************************
void CMAC_readMemoryBlockWithTagMask(uint32_t address, uint16_t *output,
                                     uint32_t tagAddress, uint16_t *tagMasked)
{
    uint16_t i;
    int16_t j;
    uint16_t outputIndex = 0U;
    uint32_t currentAddress = address;

    for(i = 0U; i < 8U; i+= 2)
    {
        for(j = 1; j >= 0; j--)
        {
            currentAddress = address + i + j;

            //
            // Once golden tag is masked, reduce compilation checks
            //
            if(*tagMasked == 0x1U)
            {
                output[outputIndex] = (HWREGH(currentAddress) & 0xFF00U) >> 8U;
                output[outputIndex + 1U] = (HWREGH(currentAddress) & 0xFFU);
            }

            //
            // Check if current address being read is an address in memory
            // containing the golden tag, if so, mask with 0xFs. Once golden
            // tag is masked, no longer perform this check.
            //
            else
            {
                if((currentAddress >= tagAddress) &&
                   (currentAddress < (tagAddress + 0x8U)))
                {
                    output[outputIndex] = 0xFFU;
                    output[outputIndex + 1] = 0xFFU;
                }
                else if(currentAddress >= (tagAddress + 0x8U))
                {
                    *tagMasked = 0x1U;
                    output[outputIndex] = (HWREGH(currentAddress) & 0xFF00U)
                                          >> 8U;
                    output[outputIndex + 1U] = (HWREGH(currentAddress) & 0xFFU);
                }
                else
                {
                    output[outputIndex] = (HWREGH(currentAddress) & 0xFF00U)
                                          >> 8U;
                    output[outputIndex + 1U] = (HWREGH(currentAddress) & 0xFFU);
                }
            }
            outputIndex+=2U;
        }
    }
}

//*****************************************************************************
//
// CMAC_readGoldentag - Read the golden CMAC tag (128-bit) from
//                      location in memory (first address contains tag MSB)
//
//*****************************************************************************
void CMAC_readGoldentag(uint32_t address, uint16_t *output)
{
    uint16_t i;

    for(i = 0U; i < 8U; i+= 2)
    {
        output[i] = HWREGH(address + i);
        output[i + 1] = HWREGH(address + i + 1UL);
    }
}

//*****************************************************************************
//
// CMAC_convertToWordArray - Convert 16 byte array to 8 word array
//
//*****************************************************************************
void CMAC_convertToWordArray(uint16_t *input, uint16_t *output)
{
    uint16_t i;
    uint16_t inputIndex = 0U;

    for(i=0U; i < 8U; i++)
    {
        output[i] = ((input[inputIndex] << 8U) | input[inputIndex + 1U]);

        inputIndex+=2U;
    }
}

//*****************************************************************************
//
// CMAC_memset - Copy of compiler memset to quickly clear arrays
//
//*****************************************************************************
void CMAC_memset(void *mem, int16_t ch, uint32_t length)
{
    char *m = (char *)mem;

    while(length--)
    {
        *m++ = ch;
    }
}

//*****************************************************************************
//
// CMAC_move128bitArray - Move input array into output array
//
//*****************************************************************************
void CMAC_move128bitArray(uint16_t *input, uint16_t *output)
{
    int16_t i;

    for(i = 0; i < 16; i++)
    {
        output[i] = input[i];
    }
}

//*****************************************************************************
//
// CMAC_move256bitArray - Move input array into output array
//
//*****************************************************************************
void CMAC_move256bitArray(uint16_t *input, uint16_t *output)
{
    int16_t i;

    for(i = 0; i < 32; i++)
    {
        output[i] = input[i];
    }
}

//*****************************************************************************
//
// AES256_performCMAC
//
//*****************************************************************************
uint32_t AES256_performCMAC(uint16_t *cmacKey, uint32_t startAddress,
                            uint32_t endAddress, uint32_t tagAddress)
{

    //
    // Setup initial local variable set
    //
    uint32_t status = 0xFFFFFFFFUL;
    uint16_t tagWithinMessage = 0U;
    uint32_t length = endAddress - startAddress;

    //
    // Verify the length aligns to 128-bit (8 word) boundary and that length
    // isn't equal to zero
    //
    if(((length % (uint32_t)8U) != 0U) || (length == 0U))
    {
        //
        // Clear Stack
        //
        startAddress = 0U;
        endAddress = 0U;
        tagAddress = 0U;
        length = 0U;

        //
        // Set status indicated boundary or length issue
        //
        status = 0xA5A5A5A5UL;

        return(status);
    }

    //
    // Determine if golden tag is within the message range to be read
    //
    if((tagAddress >= startAddress) && (tagAddress < endAddress))
    {
        tagWithinMessage = 1U;
    }

    //
    // Setup local variables for calculation
    //
    int16_t i = 0;
    uint32_t currentAddress = startAddress;
    uint32_t numberOfBlocks = 0U;
    uint16_t tagMasked = 0U;
    uint16_t k0[16];
    uint16_t k1Temp[16];
    uint16_t k1[16];
    uint16_t CMAC_TAG[16];
    uint16_t CMAC_LAST_MESSAGE[16];
    uint16_t CMAC_TAG_FINAL[8];
    uint16_t CMAC_GOLDEN_TAG[8];
    uint16_t CMAC_256BIT_KEY[32];
    uint16_t CMAC_AES_CONSTANT[16];
    uint16_t CMAC_ZERO_MESSAGE[16];
    uint16_t currentMessage[16];

    //
    // Set arrays to zero
    //
    CMAC_memset(CMAC_TAG, 0, sizeof(CMAC_TAG));
    CMAC_memset(CMAC_LAST_MESSAGE, 0, sizeof(CMAC_LAST_MESSAGE));
    CMAC_memset(CMAC_TAG_FINAL, 0, sizeof(CMAC_TAG_FINAL));
    CMAC_memset(CMAC_GOLDEN_TAG, 0, sizeof(CMAC_GOLDEN_TAG));
    CMAC_memset(CMAC_256BIT_KEY, 0, sizeof(CMAC_256BIT_KEY));
    CMAC_memset(CMAC_AES_CONSTANT, 0, sizeof(CMAC_AES_CONSTANT));
    CMAC_memset(CMAC_ZERO_MESSAGE, 0, sizeof(CMAC_ZERO_MESSAGE));
    CMAC_memset(currentMessage, 0, sizeof(currentMessage));

    //
    // Configure AES Constant
    //
    CMAC_AES_CONSTANT[15] = 0x87U;

    //
    // Calculate number of blocks
    // Note: Blocks are 128bits(8 words) in size
    //
    numberOfBlocks = length / 8U;

    //
    // Read CMAC key from parameter and pass into CMAC_256BIT_KEY
    //
    // CMAC_256BIT_KEY[0] contains MSB, CMAC_256BIT_KEY[31] contains LSB
    //
    CMAC_move256bitArray(cmacKey, CMAC_256BIT_KEY);

    //
    // Begin CMAC Algorithm
    //

    //
    // Calculate k0 with encryption of 0
    //
    AES256_encrypt(CMAC_256BIT_KEY, CMAC_ZERO_MESSAGE);

    //
    // Move ciphertext into k0 for future use
    //
    CMAC_move128bitArray(CMAC_ZERO_MESSAGE, k0);

    //
    // Reset Key data
    //
    CMAC_move256bitArray(cmacKey, CMAC_256BIT_KEY);

    //
    // Generate k1
    // Note: Not generating k2 since requiring memory range to be aligned to
    // 128-bit blocks. No padding will be required or performed.
    //
    if((k0[0] & 0x80U) == 0U)
    {
        //
        // Leftshift k0 by 1 to calculate k1
        //
        CMAC_performOneBitLeftshift(k0, k1);
    }
    else
    {
        //
        // Leftshift k0 by 1 to calculate temporary k1. Then XOR temporary k1
        // by the AES-256bit constant to calculate k1.
        //
        CMAC_performOneBitLeftshift(k0, k1Temp);
        CMAC_perform128bitXOR(k1Temp, CMAC_AES_CONSTANT, k1);
    }

    //
    // Clear temporary key data
    //
    CMAC_memset(k0, 0, sizeof(k0));
    CMAC_memset(k1Temp, 0, sizeof(k1Temp));

    //
    // Perform message block cipher encryption on all blocks except last one
    //
    for(i = 0; i < (numberOfBlocks - 1U); i++)
    {
        //
        // Read 1 block (128-bits) from memory at current address
        //
        if((tagWithinMessage == 1U) && (tagMasked == 0U))
        {
            //
            // If golden tag is within message, use function to mask
            // value to all Fs
            //
            CMAC_readMemoryBlockWithTagMask(currentAddress, currentMessage,
                                            tagAddress, &tagMasked);
        }
        else
        {
            CMAC_readMemoryBlock(currentAddress, currentMessage);
        }

        //
        // XOR the current tag with the current message and place in
        // CMAC_TAG
        //
        CMAC_perform128bitXOR(CMAC_TAG,currentMessage,CMAC_TAG);

        //
        // Perform AES ECB encryption on CMAC_TAG
        //
        AES256_encrypt(CMAC_256BIT_KEY, CMAC_TAG);

        //
        // Reset key
        //
        CMAC_move256bitArray(cmacKey, CMAC_256BIT_KEY);

        //
        // Update address to point to next block
        //
        currentAddress += 0x8U;
    }

    //
    // Read last message block and XOR message with k1
    //
    if(tagWithinMessage == 1U)
    {
        //
        // If golden tag is within message, use function to mask
        // value to all Fs
        //
        CMAC_readMemoryBlockWithTagMask(currentAddress, currentMessage,
                                        tagAddress, &tagMasked);
    }
    else
    {
        CMAC_readMemoryBlock(currentAddress, currentMessage);
    }
    CMAC_perform128bitXOR(k1, currentMessage, CMAC_LAST_MESSAGE);

    //
    // Clear K1
    //
    CMAC_memset(k1, 0, sizeof(k1));

    //
    // Generate final tag - XOR CMAC_TAG and CMAC_LAST_MESSAGE.
    //                      Encrypt CMAC_TAG.
    //
    CMAC_perform128bitXOR(CMAC_TAG, CMAC_LAST_MESSAGE, CMAC_TAG);
    AES256_encrypt(CMAC_256BIT_KEY, CMAC_TAG);

    //
    // Convert calculated CMAC tag from byte to word storage
    //
    CMAC_convertToWordArray(CMAC_TAG, CMAC_TAG_FINAL);

    //
    // Clear CMAC tag byte array
    //
    CMAC_memset(CMAC_TAG, 0, sizeof(CMAC_TAG));

    //
    // Read golden CMAC tag from memory
    //
    CMAC_readGoldentag(tagAddress, CMAC_GOLDEN_TAG);

    //
    // Compare CMAC calculated tag to golden tag
    //
    for(i = 0U; i < 8U; i++)
    {
        if(CMAC_TAG_FINAL[i] != CMAC_GOLDEN_TAG[i])
        {
            status = 0xFFFFFFFFUL;
            break;
        }
        else
        {
            status = 0x0U;
        }
    }

    //
    // Finish Clearing Stack
    //
    CMAC_memset(CMAC_TAG_FINAL, 0, sizeof(CMAC_TAG_FINAL));
    CMAC_memset(CMAC_GOLDEN_TAG, 0, sizeof(CMAC_GOLDEN_TAG));
    CMAC_memset(currentMessage, 0, sizeof(currentMessage));
    CMAC_memset(CMAC_LAST_MESSAGE, 0, sizeof(CMAC_LAST_MESSAGE));
    CMAC_memset(CMAC_AES_CONSTANT, 0, sizeof(CMAC_AES_CONSTANT));
    CMAC_memset(CMAC_ZERO_MESSAGE, 0, sizeof(CMAC_ZERO_MESSAGE));
    startAddress = 0U;
    endAddress = 0U;
    tagAddress = 0U;
    length = 0U;
    currentAddress = 0U;
    numberOfBlocks = 0U;
    tagMasked = 0U;

    //
    // Return pass or fail status
    //
    return(status);
}

//
// End of File
//
