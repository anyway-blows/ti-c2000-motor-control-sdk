//#############################################################################
//
// FILE:   crc.c
//*****************************************************************************
// function definitions
//*****************************************************************************
//*****************************************************************************
//
//! Generates CRC tables for a specified polynomial
//!
//! \param nBits -      Number of bits of the given polynomial
//! \param polynomial - Polynomial used for CRC calculations
//! \param pTable -     Pointer to the table where the CRC table values
//!                     calculated
//!                     will be stored.
//!
//! This function generates table of 256 entries for a given
//!                     CRC polynomial (polynomial)
//! with specified number of bits (nBits)
//! Generated tables are stored at the address specified by pTable.
//!
//! \return None.
//
//*****************************************************************************
//#############################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:27 CDT 2021 $
// $Copyright:
// Copyright (C) 2017-2020 Texas Instruments Incorporated
//
//     http://www.ti.com/ ALL RIGHTS RESERVED
// $
//#############################################################################
#include "PM_tformat_include.h"
#include "PM_tformat_internal_include.h"

void PM_tformat_generateCRCTable(uint16_t nBits, uint16_t polynomial,          \
        uint16_t *pTable)
{
    uint16_t i, j;
    uint16_t accum;

    polynomial <<= (8 - nBits);
    for(i = 0; i < 256 ; i++)
    {
        accum  = i;
        for( j = 0; j < 8; j++)
        {
            if(accum & 0x80)
            {

                //
                // If leading bit is 1, shift accum to left, mask off unwanted
                // MSbs and xor the rest with poly
                //
                accum = ((accum << 1) & 0xFF) ^ polynomial;
            }
            else
            {

                //
                // If leading bit is 0, shift accum to left, mask off unwanted MSbs
                //
                accum = ((accum << 1) & 0xFF);
            }
        }
        pTable[i] = accum;
    }
}


//! \internal

uint16_t PM_tformat_getCRC(uint16_t inputCRCaccum, uint16_t nBitsData,
        uint16_t nBitsPoly, uint16_t * msg, uint16_t *crcTable, uint16_t rxLen)
{
    uint16_t i;
    uint16_t j;
    uint16_t index;
    uint16_t crcAccum, crcAccumLow = 0U;
    int *pdata;
    uint16_t nDanglingBits;

    index = rxLen - 1; // Start from the end

    //
    // n_danglingBits = nBitsData % 8
    //
    nDanglingBits = nBitsData & 0x7;

    //
    // Special case, cannot xor the initial value
    // fully into the leading byte
    //
    crcAccum = 0xFF;
    if(nDanglingBits != 0)
    {
        if(nDanglingBits < nBitsPoly)
        {

            //
            // Shift RIGHT, so the initial value lines up with the
            // leading bit in the stream
            //
            crcAccum = inputCRCaccum >> (nBitsPoly - nDanglingBits);

            //
            // Shift  left, so the leftover bits in the initial value lines up
            // with the start of the next byte
            //
            crcAccumLow = (inputCRCaccum
                            << (8 - (nBitsPoly - nDanglingBits)) & 0xFF);
        }
        else //nDanglingBits >= nBitsPoly
        {

            //
            // Shift left, so the initial value lines up with the
            // leading bit in the stream
            //
            crcAccum = inputCRCaccum << (nDanglingBits - nBitsPoly);
        }
    }

    pdata = (int *)msg;

    //
    // Do the first two bytes, special case
    //
    i = crcAccum  ^ (__byte(pdata, index--));
    crcAccum = crcTable[i] ^ crcAccumLow;
    i = crcAccum ^ (__byte(pdata, index--));
    crcAccum = crcTable[i];

    //
    // Do the CRC for the bytes
    //
    for (j = 0; j < rxLen - 2; j++, index--)
    {
        i = crcAccum  ^ (__byte(pdata, index));
        crcAccum = crcTable[i];
    }

    return((uint16_t)(crcAccum >> (8 - nBitsPoly)));
}


//
// End of file
//
