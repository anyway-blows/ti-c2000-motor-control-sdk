//###########################################################################
//
// FILE:   aes_ecb.c
//
// TITLE:  Implementation of the AES-256 ECB mode as defined by the FIPS PUB
//         197:the official AES standard
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

#include "aes_ecb.h"

//*****************************************************************************
//
// Substitution box used in the encryption steps.
//
//*****************************************************************************
const unsigned char sbox[256] =   {
//0     1    2      3     4    5     6     7      8    9     A      B    C     D     E     F
0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76, //0
0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0, //1
0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15, //2
0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75, //3
0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84, //4
0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf, //5
0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8, //6
0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2, //7
0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73, //8
0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb, //9
0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79, //A
0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08, //B
0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a, //C
0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e, //D
0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf, //E
0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16 }; //F

//*****************************************************************************
//
// Reverse substitution box used in the decryption steps.
//
//*****************************************************************************
const unsigned char rsbox[256] =
{ 0x52, 0x09, 0x6a, 0xd5, 0x30, 0x36, 0xa5, 0x38, 0xbf, 0x40, 0xa3, 0x9e, 0x81, 0xf3, 0xd7, 0xfb
, 0x7c, 0xe3, 0x39, 0x82, 0x9b, 0x2f, 0xff, 0x87, 0x34, 0x8e, 0x43, 0x44, 0xc4, 0xde, 0xe9, 0xcb
, 0x54, 0x7b, 0x94, 0x32, 0xa6, 0xc2, 0x23, 0x3d, 0xee, 0x4c, 0x95, 0x0b, 0x42, 0xfa, 0xc3, 0x4e
, 0x08, 0x2e, 0xa1, 0x66, 0x28, 0xd9, 0x24, 0xb2, 0x76, 0x5b, 0xa2, 0x49, 0x6d, 0x8b, 0xd1, 0x25
, 0x72, 0xf8, 0xf6, 0x64, 0x86, 0x68, 0x98, 0x16, 0xd4, 0xa4, 0x5c, 0xcc, 0x5d, 0x65, 0xb6, 0x92
, 0x6c, 0x70, 0x48, 0x50, 0xfd, 0xed, 0xb9, 0xda, 0x5e, 0x15, 0x46, 0x57, 0xa7, 0x8d, 0x9d, 0x84
, 0x90, 0xd8, 0xab, 0x00, 0x8c, 0xbc, 0xd3, 0x0a, 0xf7, 0xe4, 0x58, 0x05, 0xb8, 0xb3, 0x45, 0x06
, 0xd0, 0x2c, 0x1e, 0x8f, 0xca, 0x3f, 0x0f, 0x02, 0xc1, 0xaf, 0xbd, 0x03, 0x01, 0x13, 0x8a, 0x6b
, 0x3a, 0x91, 0x11, 0x41, 0x4f, 0x67, 0xdc, 0xea, 0x97, 0xf2, 0xcf, 0xce, 0xf0, 0xb4, 0xe6, 0x73
, 0x96, 0xac, 0x74, 0x22, 0xe7, 0xad, 0x35, 0x85, 0xe2, 0xf9, 0x37, 0xe8, 0x1c, 0x75, 0xdf, 0x6e
, 0x47, 0xf1, 0x1a, 0x71, 0x1d, 0x29, 0xc5, 0x89, 0x6f, 0xb7, 0x62, 0x0e, 0xaa, 0x18, 0xbe, 0x1b
, 0xfc, 0x56, 0x3e, 0x4b, 0xc6, 0xd2, 0x79, 0x20, 0x9a, 0xdb, 0xc0, 0xfe, 0x78, 0xcd, 0x5a, 0xf4
, 0x1f, 0xdd, 0xa8, 0x33, 0x88, 0x07, 0xc7, 0x31, 0xb1, 0x12, 0x10, 0x59, 0x27, 0x80, 0xec, 0x5f
, 0x60, 0x51, 0x7f, 0xa9, 0x19, 0xb5, 0x4a, 0x0d, 0x2d, 0xe5, 0x7a, 0x9f, 0x93, 0xc9, 0x9c, 0xef
, 0xa0, 0xe0, 0x3b, 0x4d, 0xae, 0x2a, 0xf5, 0xb0, 0xc8, 0xeb, 0xbb, 0x3c, 0x83, 0x53, 0x99, 0x61
, 0x17, 0x2b, 0x04, 0x7e, 0xba, 0x77, 0xd6, 0x26, 0xe1, 0x69, 0x14, 0x63, 0x55, 0x21, 0x0c, 0x7d };

//*****************************************************************************
//
// Round constants needed for the key schedule.
//
//*****************************************************************************
const unsigned char Rcon[7] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40};

//*****************************************************************************
//
// performGaloisMultiple
//
//*****************************************************************************
uint16_t performGaloisMultiple(uint16_t value)
{
    int16_t signedValue;

    //
    // Cast to signed value
    //
    signedValue = (int16_t) value;

    //
    // If MSB is 1, then this will signed extend and fill the temp variable
    // with 1's
    //
    signedValue = signedValue >> 7;

    //
    // Ensures that the temp variable is actually a -1 if the MSB is 1
    //
    if(signedValue == 0x01)
    {
        signedValue = 0xFF;
    }

    //
    // AND with the reduction variable
    //
    signedValue = signedValue & 0x1B;

    //
    // Finally shift and reduce the value
    // Added mask to account for C28x
    //
    return(((value << 1) & 0xFF) ^ signedValue);
}
//*****************************************************************************
//
// AES256_performECB
//
//*****************************************************************************
void AES256_performECB(AES_OperationMode mode, uint16_t *key,  uint16_t *text)
{
    uint16_t buf1, buf2, buf3, buf4, round, i;
    uint16_t previousKey[14][16];
    uint16_t rconEncIndex = 0;
    uint16_t rconDecIndex = 0;

    //
    // In case of decryption, the key schedule is precomputed and the first
    // addRoundKey step is performed
    //
    if(mode == AES_OPMODE_DECRYPT)
    {
        //
        // Compute the last key of encryption before starting the decryption
        //
        for(round = 0; round < 14; round++)
        {
            //
            // For round 0, the keys in the upper half of the key (indices
            // 16-31) are moved to the lower half of the array (indices 0-15).
            // The key used in round 0 is also saved so it can be used to
            // compute the key for round 2.
            //
            if(round < 1)
            {
                for(i = 0; i < 16; i++)
                {
                    previousKey[round][i] = key[i];
                    key[i] = key[i+16];
                }
            }

            //
            // expanded key schedule
            //
            else
            {
                //
                // The current key is saved in the previousKey array
                // These values are used 1 round after they are written in
                //
                for(i = 0; i < 16; i++)
                {
                    previousKey[round][i] = key[i];
                }

                //
                // All odd numbered rounds rotate the last 4 bytes of the
                // current key and these are used to index into the sbox. These
                // values are XORed with the key values from 1 round ago, and
                // are also XORed with the Rcon value. For keys 1-3, the Rcon
                // value is 0x00 so this was not shown.
                //
                if((round & 0x01) == 1)
                {
                    key[0] = sbox[key[13]] ^ previousKey[round-1][0] ^
                             Rcon[rconDecIndex];
                    key[1] = sbox[key[14]] ^ previousKey[round-1][1];
                    key[2] = sbox[key[15]] ^ previousKey[round-1][2];
                    key[3] = sbox[key[12]] ^ previousKey[round-1][3];
                    rconDecIndex++;
                }

                //
                // All even numbered rounds use the last 4 bytes of the current
                // key and are used to index into the sbox. These values are
                // XORed with the key values from 1 round ago.
                //
                else
                {
                    key[0] = sbox[key[12]] ^ previousKey[round-1][0];
                    key[1] = sbox[key[13]] ^ previousKey[round-1][1];
                    key[2] = sbox[key[14]] ^ previousKey[round-1][2];
                    key[3] = sbox[key[15]] ^ previousKey[round-1][3];
                }

                //
                // Keys 4-15 are computed by indexing into previousKey (the key
                // from 1 round ago), and XORing those values with the value
                // 4 indices prior in the current key.
                //
                for(i = 4; i < 16; i++)
                {
                    key[i] = previousKey[round-1][i] ^ key[i-4];
                }
            }
        }

        //
        // first Addroundkey
        //
        for(i = 0; i < 16; i++)
        {
            text[i] = text[i] ^ key[i];
        }
    }

    //
    // main loop - runs the 14 rounds of encryption/decryption
    //
    // Encryption order of operations: addRoundKey, subBytes, shiftRows,
    //                                 mixColumns, key schedule
    //
    // Decryption order of operations: key schedule, shiftRows, mixColumns,
    //                                 inverse shiftRows, addRoundKey
    //
    for(round = 0; round < 14; round++)
    {
        if(mode == AES_OPMODE_DECRYPT)
        {

            //
            // Get the appropriate key needed for the round from the key
            // schedule
            //
            for(i = 0; i < 16; i++)
            {
                key[i] = previousKey[13-round][i];
            }
        }
        else
        {
            //
            // addRoundKey and subBytes steps
            //
            for(i = 0; i < 16; i++)
            {
                text[i]=sbox[text[i] ^ key[i]];
            }

        //
        // shiftRows step -- same for Encryption and Decryption
        //
        buf1 = text[1];
        text[1] = text[5];
        text[5] = text[9];
        text[9] = text[13];
        text[13] = buf1;

        buf1 = text[2];
        buf2 = text[6];
        text[2] = text[10];
        text[6] = text[14];
        text[10] = buf1;
        text[14] = buf2;

        buf1 = text[15];
        text[15] = text[11];
        text[11] = text[7];
        text[7] = text[3];
        text[3] = buf1;
        }

        //
        // mixColumns step, done in every round except the last for encryption,
        // and every round except the first for decryption
        //
        if((round > 0 && mode == AES_OPMODE_DECRYPT) ||
           (round < 13 && mode == AES_OPMODE_ENCRYPT))
        {
            //
            // 4 keys are operated on at once, so there are 4 rounds to account
            // for the 16 keys
            //
            for(i = 0; i < 4; i++)
            {
                buf4 = (i << 2);

                //
                // Precompute the values needed for decryption
                //
                if(mode == AES_OPMODE_DECRYPT)
                {
                    buf1 = performGaloisMultiple(
                            performGaloisMultiple(
                             text[buf4] ^ text[buf4+2]));
                    buf2 = performGaloisMultiple(
                            performGaloisMultiple(
                             text[buf4+1] ^ text[buf4+3]));
                    text[buf4] ^= buf1;
                    text[buf4+1] ^= buf2;
                    text[buf4+2] ^= buf1;
                    text[buf4+3] ^= buf2;
                }

                //
                // for encryption and decryption
                //
                buf1 = text[buf4] ^ text[buf4+1] ^ text[buf4+2] ^ text[buf4+3];
                buf2 = text[buf4];

                buf3 = text[buf4] ^ text[buf4+1];
                buf3 = performGaloisMultiple(buf3);
                text[buf4] = text[buf4] ^ buf3 ^ buf1;

                buf3 = text[buf4+1] ^ text[buf4+2];
                buf3 = performGaloisMultiple(buf3);
                text[buf4+1] = text[buf4+1] ^ buf3 ^ buf1;

                buf3 = text[buf4+2] ^ text[buf4+3];
                buf3 = performGaloisMultiple(buf3);
                text[buf4+2] = text[buf4+2] ^ buf3 ^ buf1;

                buf3 = text[buf4+3] ^ buf2;
                buf3 = performGaloisMultiple(buf3);
                text[buf4+3] = text[buf4+3] ^ buf3 ^ buf1;
            }
        }

        //
        // Inverse shift rows for decryption to index into the rsbox, Rows 1-3
        // are shifted and Row 0 is left as is
        //
        if(mode == AES_OPMODE_DECRYPT)
        {
            //
            // Row 1
            //
            buf1 = text[13];
            text[13] = text[9];
            text[9] = text[5];
            text[5] = text[1];
            text[1] = buf1;

            //
            // Row 2
            //
            buf1 = text[10];
            buf2 = text[14];
            text[10] = text[2];
            text[14] = text[6];
            text[2] = buf1;
            text[6] = buf2;

            //
            // Row 3
            //
            buf1 = text[3];
            text[3] = text[7];
            text[7] = text[11];
            text[11] = text[15];
            text[15] = buf1;

            //
            // Index into rsbox to recover an earlier text value
            //
            for(i = 0; i < 16; i++)
            {
                text[i] = rsbox[text[i]] ^ key[i];
            }
        }
        else
        {

            //
            // For round 0, the keys in the upper half of the key (indices
            // 16-31) are moved to the lower half of the array (indices 0-15).
            // the key used in round 0 is also saved so it can be used to
            // compute the key for round 2.
            //
            if(round < 1)
            {
                for(i = 0; i < 16; i++)
                {
                    previousKey[round][i] = key[i];
                    key[i] = key[i+16];
                }
            }

            //
            // expanded key schedule
            //
            else
            {
                //
                // The current key is saved in the previousKey array.
                // These values are used 2 rounds after they are written in.
                //
                for(i = 0; i < 16; i++)
                {
                    previousKey[round][i] = key[i];
                }

                //
                // All odd numbered rounds rotate the last 4 bytes of the
                // current key and these are used to index into the sbox.
                // These values are XORed with the key values from 1 round ago,
                // and are also XORed with the Rcon value.
                // For keys 1-3, the Rcon value is 0x00 so this was not shown.
                //
                if((round & 0x01) == 1)
                {
                    key[0] = sbox[key[13]] ^ previousKey[round-1][0] ^
                             Rcon[rconEncIndex];
                    key[1] = sbox[key[14]] ^ previousKey[round-1][1];
                    key[2] = sbox[key[15]] ^ previousKey[round-1][2];
                    key[3] = sbox[key[12]] ^ previousKey[round-1][3];
                    rconEncIndex++;
                }

                //
                // All even numbered rounds use the last 4 bytes of the current
                // key and are used to index into the sbox. These values are
                // XORed with the key values from 1 round ago.
                //
                else
                {
                    key[0] = sbox[key[12]] ^ previousKey[round-1][0];
                    key[1] = sbox[key[13]] ^ previousKey[round-1][1];
                    key[2] = sbox[key[14]] ^ previousKey[round-1][2];
                    key[3] = sbox[key[15]] ^ previousKey[round-1][3];
                }

                //
                // Keys 4-15 are computed by indexing into previousKey (the key
                // from 1 round ago), and XORing those values with the value
                // 4 indexes prior in the current key.
                //
                for(i = 4; i < 16; i++)
                {
                    key[i] = previousKey[round-1][i] ^ key[i-4];
                }

                //
                // addRoundKey for the last round
                //
                if (round == 13)
                {
                    for (i = 0; i < 16; i++)
                    {
                        text[i] = text[i] ^ key[i];
                    }
                }
            }
        }
    }
}
//
// End of File
//
