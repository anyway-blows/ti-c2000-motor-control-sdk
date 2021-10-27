//#############################################################################
//
// FILE: spifsi.c
//
// TITLE: SPI to FSI communication
//
//! \addtogroup spifsi_communication
//
//
//#############################################################################
// $TI Release: SPIFSI Utility Driver 3.04.00.00 $
// $Release Date: 2020 $
// $Copyright:
// Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
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

//
// Includes
//
#include "spifsi.h"

//*****************************************************************************
//
// SPIFSI_readFrame(uint32_t base, SPIFSI_FrameInfo *frameInfo)
// SPIFSI_DID_R_04
//
//*****************************************************************************
SPIFSI_Error SPIFSI_readFrame(uint32_t base, SPIFSI_FrameInfo *frameInfo)
{
    //
    // Frame sections.
    //
    uint16_t sof = 0U;
    uint16_t frameType = 0U;
    uint16_t userData = 0U;
    uint16_t crc8 = 0U;
    uint16_t frameTag = 0U;
    uint16_t eof = 0U;
    uint16_t dataLength = 0U;

    //
    // Temp variables.
    //
    uint16_t word = 0U;
    uint16_t userDataCRC8 = 0U;
    uint16_t finalCRC8 = 0U;
    uint16_t i;

    //
    // Error flag.
    //
    uint16_t readError = SPIFSI_PASS;

    //
    // Read first word (16 bits) and extract sof and frameType.
    //
    readError = SPI_read16Bits(base, &word);

    //
    // Return readError if error occurs from reading.
    //
    if(readError) {
        return(SPIFSI_READ_ERROR);
    }

    //
    // Extract sof and frameType, then populate frameType.
    //
    sof = (word & SPIFSI_FIRST_BIT_MASK) >> SPIFSI_SHIFT_12;
    frameType = (word & SPIFSI_SECOND_BIT_MASK) >> SPIFSI_SHIFT_8;
    frameInfo->frameType = (SPIFSI_FrameType) frameType;

    //
    // Verify sof.
    // SPIFSI_DID_R_05
    //
    if(sof != SPIFSI_START_OF_FRAME)
    {
        return(SPIFSI_SOF_ERROR);
    }

    //
    // Check frameType.
    //
    switch(frameType)
    {
        case SPIFSI_FRAME_TYPE_NWORD_DATA:
            //
            // If NWORD frame type, given paramenter, nLength,
            // is the length of data.
            //

            //
            // Verify nLength
            // SPIFSI_DID_R_06
            //
            if((frameInfo->nLength < SPIFSI_MIN_DATA_WORDS) ||
               (frameInfo->nLength > SPIFSI_MAX_DATA_WORDS))
            {
                return(SPIFSI_N_RANGE_ERROR);
            }

            dataLength = frameInfo->nLength;
            break;
        case SPIFSI_FRAME_TYPE_1WORD_DATA:

            //
            // Initialize dataLength.
            //
            dataLength = 1U;
            break;
        case SPIFSI_FRAME_TYPE_2WORD_DATA:
            dataLength = 2U;
            break;
        case SPIFSI_FRAME_TYPE_4WORD_DATA:
            dataLength = 4U;
            break;
        case SPIFSI_FRAME_TYPE_6WORD_DATA:
            dataLength = 6U;
            break;
        case SPIFSI_FRAME_TYPE_ERROR:
        case SPIFSI_FRAME_TYPE_PING:

            //
            // In case of ERROR or PING, check eof, then populate tag.
            // SPIFSI_DID_R_07
            //
            eof = (word & SPIFSI_FOURTH_BIT_MASK);
            if(eof != SPIFSI_END_OF_FRAME)
            {
                return(SPIFSI_EOF_ERROR);
            }

            //
            // Different structure of word for ERROR and PING.
            // SPIFSI_DID_R_08
            //
            frameInfo->frameTag = (SPIFSI_FrameTag)
                    (word & SPIFSI_THIRD_BIT_MASK) >> SPIFSI_SHIFT_4;

            return(SPIFSI_NO_ERROR);
        default:

            //
            // Return frame type error.
            // SPIFSI_DID_R_09
            //
            return(SPIFSI_F_TYPE_ERROR);
    }

    //
    // Extract userData and populate.
    // SPIFSI_DID_R_11
    //
    userData = (word & SPIFSI_LAST_TWO_BITS_MASK);
    frameInfo->userData = userData;

    //
    // Read as many words as data length determined by frameType and nLength.
    // SPIFSI_DID_R_14
    //
    for(i = 0U; i < dataLength; i++)
    {
        //
        // Read and check for error.
        //
        readError = SPI_read16Bits(base, (frameInfo->data+i));
        if(readError)
        {
            return(SPIFSI_READ_ERROR);
        }
    }

    //
    // Read the last word and extract crc, frameTag, and eof.
    // SPIFSI_DID_R_15
    //
    readError = SPI_read16Bits(base, &word);
    if(readError)
    {
        return(SPIFSI_READ_ERROR);
    }
    crc8 = (word & SPIFSI_FIRST_TWO_BITS_MASK) >> SPIFSI_SHIFT_8;
    frameInfo->crc8 = crc8;
    frameTag = (word & SPIFSI_THIRD_BIT_MASK) >> SPIFSI_SHIFT_4;
    frameInfo->frameTag = (SPIFSI_FrameTag) frameTag;
    eof = (word & SPIFSI_FOURTH_BIT_MASK);

    //
    // Verify eof.
    // SPIFSI_DID_R_13
    //
    if(eof != SPIFSI_END_OF_FRAME)
    {
        return(SPIFSI_EOF_ERROR);
    }

    //
    // Verify CRC8. First, crc8 check on user data.
    // SPIFSI_DID_R_16
    //
    userDataCRC8 = (uint16_t) SPIFSI_calcCRC8(SPIFSI_CRC8_SEED_0,
                                              &(frameInfo->userData),
                                              SPIFSI_CRC8_DEFAULT_PARITY,
                                              1U);

    //
    // Second, crc8 check on user data + dataLength * 2
    // because number of byte (1 word = 2 bytes).
    // SPIFSI_DID_R_17
    //
    finalCRC8 = (uint16_t) SPIFSI_calcCRC8(userDataCRC8,
                                           (frameInfo->data),
                                           SPIFSI_CRC8_DEFAULT_PARITY,
                                           dataLength * 2U);

    //
    // Compare crc8
    // SPIFSI_DID_R_18
    //
    if(crc8 != (finalCRC8 & SPIFSI_LAST_TWO_BITS_MASK))
    {
        return(SPIFSI_CRC_ERROR);
    }


    //
    // No error.
    // SPIFSI_DID_R_20
    //
    return(SPIFSI_NO_ERROR);
}

//*****************************************************************************
//
// SPIFSI_writeFrame(uint32_t base, SPIFSI_frameInfo *frameInfo)
// SPIFSI_DID_W_04
//
//*****************************************************************************
SPIFSI_Error SPIFSI_writeFrame(uint32_t base, SPIFSI_FrameInfo *frameInfo)
{
    //
    // Initialize temp variables.
    //
    uint16_t dataLength = 0U;
    uint16_t word = 0U;
    uint16_t i;
    uint16_t userDataCRC8 = 0U;
    uint16_t finalCRC8 = 0U;

    //
    // Error flag.
    //
    uint16_t writeError = SPIFSI_PASS;

    //
    // Build a first part of the first ward.
    // SPIFSI_DID_W_06
    //
    word |= SPIFSI_START_OF_FRAME << SPIFSI_SHIFT_12;
    word |= (frameInfo->frameType & SPIFSI_FOURTH_BIT_MASK) << SPIFSI_SHIFT_8;

    //
    // Check frameType.
    //
    switch(frameInfo->frameType)
    {
        case SPIFSI_FRAME_TYPE_NWORD_DATA:

            //
            // Verify nLength
            // SPIFSI_DID_W_05
            //
            if((frameInfo->nLength < SPIFSI_MIN_DATA_WORDS) ||
               (frameInfo->nLength > SPIFSI_MAX_DATA_WORDS))
            {
                return(SPIFSI_N_RANGE_ERROR);
            }

            //
            // If NWORD frame type, given parameter, nLength, is
            // the length of data.
            //
            dataLength = frameInfo->nLength;
            break;
        case SPIFSI_FRAME_TYPE_1WORD_DATA:

            //
            // Save dataLength
            //
            dataLength = 1U;
            break;
        case SPIFSI_FRAME_TYPE_2WORD_DATA:
            dataLength = 2U;
            break;
        case SPIFSI_FRAME_TYPE_4WORD_DATA:
            dataLength = 4U;
            break;
        case SPIFSI_FRAME_TYPE_6WORD_DATA:
            dataLength = 6U;
            break;
        case SPIFSI_FRAME_TYPE_ERROR:
        case SPIFSI_FRAME_TYPE_PING:

            //
            // For ERROR and PING, build a 16-bit word consisting of sof, frameType,
            // frameTag, and eof.
            // SPIFSI_DID_W_08
            //
            word |= (frameInfo->frameTag & SPIFSI_FOURTH_BIT_MASK) << SPIFSI_SHIFT_4;
            word |= SPIFSI_END_OF_FRAME;

            //
            // Send the word and check for error.
            // SPIFSI_DID_W_08
            //
            writeError = SPI_write16Bits(base, &word);
            if(writeError)
            {
                return(SPIFSI_WRITE_ERROR);
            }

            return(SPIFSI_NO_ERROR);
        default:

            //
            // Return frame type error.
            // SPIFSI_DID_W_07
            //
            return(SPIFSI_F_TYPE_ERROR);
    }

    //
    // Fill in the rest of the first word.
    // SPIFSI_DID_W_09
    //
    word |= (frameInfo->userData & SPIFSI_LAST_TWO_BITS_MASK);

    //
    // Send/write the first word (sof, frameType, and userData)
    // SPIFSI_DID_W_09
    //
    writeError = SPI_write16Bits(base, &word);
    if(writeError)
    {
        return(SPIFSI_WRITE_ERROR);
    }

    //
    // Send/write data.
    // SPIFSI_DID_W_10
    //
    for(i = 0U; i < dataLength; i++)
    {
        writeError = SPI_write16Bits(base, (frameInfo->data + i));
        if(writeError)
        {
            return(SPIFSI_WRITE_ERROR);
        }
    }

    //
    // Crc8 generation. first, on user data.
    // SPIFSI_DID_W_11
    //
    userDataCRC8 = (uint16_t) SPIFSI_calcCRC8(SPIFSI_CRC8_SEED_0,
                                              &(frameInfo->userData),
                                              SPIFSI_CRC8_DEFAULT_PARITY,
                                              1U);

    //
    // Second, crc8 check on user data + dataLength * 2
    // because number of byte (1 word = 2 bytes).
    // SPIFSI_DID_W_12
    //
    finalCRC8 = (uint16_t) SPIFSI_calcCRC8(userDataCRC8,
                                           (frameInfo->data),
                                           SPIFSI_CRC8_DEFAULT_PARITY,
                                           dataLength * 2U);

    //
    // Mask finalCRC8.
    //
    finalCRC8 &= SPIFSI_LAST_TWO_BITS_MASK;

    //
    // Populate crc8 field of frameInfo.
    //
    frameInfo->crc8 = finalCRC8;

    //
    // Construct the last word consisted of crc, tag, and eof.
    // SPIFSI_DID_W_13
    //
    word = 0U;
    word |= finalCRC8 << SPIFSI_SHIFT_8;
    word |= (frameInfo->frameTag & SPIFSI_FOURTH_BIT_MASK) << SPIFSI_SHIFT_4;
    word |= SPIFSI_END_OF_FRAME;

    //
    // Write/send the last word then check for write error.
    // SPIFSI_DID_W_13
    //
    writeError = SPI_write16Bits(base, &word);
    if(writeError)
    {
        return(SPIFSI_WRITE_ERROR);
    }

    //
    // Return no error.
    // SPIFSI_DID_W_14
    //
    return(SPIFSI_NO_ERROR);
}

//
// End of File
//
