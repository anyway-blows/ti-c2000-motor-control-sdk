//#############################################################################
//
// FILE: spifsi_spi.h
//
// TITLE: SPI to FSI communication user-specific header file
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

#ifndef SPIFSI_SPI_H_
#define SPIFSI_SPI_H_

//
// Include vcu crc.
//
#ifdef VCU_CRC
#include "vcu0_crc.h"
#endif

#include <stdint.h>
#include <stdbool.h>
#include "driverlib.h"

//*****************************************************************************
//
//! \brief reads 16-bit word.
//!
//! \param base is base address of SPI.
//! \param data16 is a pointer to 16-bit word.
//!
//! This function reads 16-bit word from SPI and assigns the word to *data16.
//! If any error occurs during reading, this function returns one of
//! \b SPIFSI_error enum type, \b SPIFSI_NO_ERROR otherwise.
//!
//! \note The actual function call of reading does not need to be blockingFIFO
//! call, it can be switched out to any SPI 16-bit read.
//!
//! \return If there is an error while reading, it returns one of the listed
//! \b SPIFSI_error enum type, \b SPIFSI_READ_ERROR inexample. Otherwise, it
//! returns \b SPIFSI_NO_ERROR.
//
//*****************************************************************************
static inline uint16_t SPI_write16Bits(uint32_t base, uint16_t *data16)
{
    SPI_writeDataBlockingFIFO(base, *data16);

    //
    //return SPIFSI_WRITE_ERROR;
    //
    return(0U);
}

//*****************************************************************************
//
//! \brief writes 16-bit word.
//!
//! \param base is base address of SPI.
//! \param data16 is a word to be written.
//!
//! This function writes 16-bit word, data16, to SPI. If any error occurs
//! during reading, this function returns one of \b SPIFSI_error enum type,
//! \b SPIFSI_NO_ERROR otherwise.
//!
//! \note The actual function call of writing does not need to be blockingFIFO
//! call, it can be switched out to any SPI 16-bit write.
//!
//! \return If there is an error while writing, it returns one of the listed
//! \b SPIFSI_error enum type, \b SPIFSI_WRITE_ERROR inexample. Otherwise, it
//! returns \b SPIFSI_NO_ERROR.
//
//*****************************************************************************
static inline uint16_t SPI_read16Bits(uint32_t base, uint16_t *data16)
{
    *data16 = SPI_readDataBlockingFIFO(base);

    //
    //return SPIFSI_READ_ERROR;
    //
    return(0U);
}

//*****************************************************************************
//
//! \brief returns calculated 8-bit CRC
//!
//! \param input_crc8_accum The seed value for the CRC, in the event of a
//!                         multi-part message, the result of the previous crc8
//!                         can be used as the initial value for the current
//!                         segment crc8 calculation until the final crc is derived.
//! \param msg Address of the message buffer
//! \param parity Parity of the first message word. The parity determines whether
//!               the CRC begins at the low byte (CRC_parity_even) or at the high
//!               byte (CRC_parity_odd) of the first word determines whether the
//!               CRC begins at the low byte (EVEN) or at the high byte (ODD).
//! \param rxLen Length of the message in bytes
//!
//! \return CRC result
//
//*****************************************************************************
extern uint16_t SPIFSI_calcCRC8(uint32_t input_crc8_accum, uint16_t *msg, uint16_t parity, uint16_t rxLen);

//
// Non-VCU CRC
//
#ifndef VCU_CRC

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
    CRC_PARITY_EVEN = 0U,   //!< Even parity, CRC starts at the low byte of
                                //!< the first word (16-bit)
    CRC_PARITY_ODD  = 1U    //!< Odd parity, CRC starts at the high byte of
                                //!< the first word (16-bit)
} CRC_Parity;

//! \brief CRC structure
//!
typedef struct
{
    uint32_t          seedValue;    //!< Initial value of the CRC calculation
    uint16_t          numBytes;     //!< Number of bytes in the message buffer
    CRC_Parity        parity;       //!< Start the CRC from the low byte
                                    //!< or high byte
    uint32_t          crcResult;    //!< The calculated CRC
    void *            msgBuffer;    //!< Pointer to the message buffer
} CRC_Obj;

//! \brief Handle to the CRC structure
//!
typedef CRC_Obj *CRC_Handle;

//
// CRC8 table
//

//! \brief crc8 table.
//!
static const uint16_t crc8Table[] = {
0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3,
};

//*****************************************************************************
//
// CRC_calculate(CRC_Handle hndCRC)
//
//*****************************************************************************
static inline void CRC_calculate(CRC_Handle hndCRC)
{
    uint32_t i;
    uint16_t tableIndex;
    uint16_t accumulator   = hndCRC->seedValue;
    uint16_t parity        = (uint16_t)hndCRC->parity;
    uint16_t *pInputVector = (uint16_t *)hndCRC->msgBuffer;
    uint16_t *pCrcTable    = (uint16_t *)crc8Table;

    // The assumption is the message bytes are packed into 16-bit words
    // and the calculation starts from from either the high or low byte
    // The memory arrangement is as follows
    // Address|__LB__|__HB__|
    // 0x0000 |__D0__|__D1__|
    // 0x0001 |__D2__|__D3__|
    // 0x0002 |__D4__|__D5__|
    // 0x0003 |__D6__|__D7__|
    // 0x0004 |__D8__|__D9__|
    // ...

    for(i = 0; i < hndCRC->numBytes; i++, parity++){
        // __byte selects either the low(0) or high(1) byte in a word
        // the initial selection provided by the enumeration parity
        tableIndex = accumulator ^ __byte((int *)pInputVector, parity);
        accumulator = pCrcTable[tableIndex];
    }

    // Save the CRC result
    hndCRC->crcResult = (uint32_t)(accumulator & 0x00FF);
}
#endif /* VCU_CRC */
#endif /* SPIFSI_SPI_H_ */

//
// End of File
//

