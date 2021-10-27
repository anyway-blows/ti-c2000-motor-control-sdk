//#############################################################################
//
// FILE: spifsi.h
//
// TITLE: SPI to FSI communication header file
//
//! \addtogroup spifsi_communication
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

#ifndef SPIFSI_H
#define SPIFSI_H

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
#include "spifsi_map.h"

//*****************************************************************************
//
//!
//! The code for this module is contained in <tt>source/spifsi.c</tt>, with
//! <tt>source/spifsi.h</tt> containing the API declarations for use by
//! applications.
//!
//! @{
//
//*****************************************************************************

//
// Defines
//
#define SPIFSI_PASS                 0x0000U

//! \brief sof and eof.
//!
#define SPIFSI_START_OF_FRAME       0x0009U
#define SPIFSI_END_OF_FRAME         0x0006U

//! \brief maximum number of words
#define SPIFSI_MIN_DATA_WORDS       0x0001U
#define SPIFSI_MAX_DATA_WORDS       0x0010U

//! \brief mask bits and shift bits
//!
#define SPIFSI_FIRST_BIT_MASK       0xF000U
#define SPIFSI_SECOND_BIT_MASK      0x0F00U
#define SPIFSI_THIRD_BIT_MASK       0x00F0U
#define SPIFSI_FOURTH_BIT_MASK      0x000FU
#define SPIFSI_FIRST_TWO_BITS_MASK  0xFF00U
#define SPIFSI_LAST_TWO_BITS_MASK   0x00FFU
#define SPIFSI_SHIFT_4              0x0004U
#define SPIFSI_SHIFT_8              0x0008U
#define SPIFSI_SHIFT_12             0x000CU

//! \brief CRC8 default function arguments
//!
#define SPIFSI_CRC8_SEED_0          0x00000000UL
#define SPIFSI_CRC8_DEFAULT_PARITY  0x0000U

//! \brief Various SPIFSI frame types
//!
//! \details Three frame types exist-
//!          - \b Ping: Used for checking line integrity, can be sent by
//!                     software or automatically by hardware.
//!          - \b Error: Used typically during error conditions or when one
//!                      side wants to signal the other side for attention.
//!          - \b Data: Two subtypes exist based on data-length-
//!                     a) \b Fixed (1/2/4/6 words)
//!                     b) \b Nwords Software programs number of data words
//!
//!  \note 4 bit code for frame types- 0x1, 0x2 and 0x8 to 0xE are reserved
//!
typedef enum
{
    SPIFSI_FRAME_TYPE_PING       = 0x0000U,
    SPIFSI_FRAME_TYPE_ERROR      = 0x000FU,
    SPIFSI_FRAME_TYPE_1WORD_DATA = 0x0004U,
    SPIFSI_FRAME_TYPE_2WORD_DATA = 0x0005U,
    SPIFSI_FRAME_TYPE_4WORD_DATA = 0x0006U,
    SPIFSI_FRAME_TYPE_6WORD_DATA = 0x0007U,
    SPIFSI_FRAME_TYPE_NWORD_DATA = 0x0003U
} SPIFSI_FrameType;

//! \brief Possible values of a SPIFSI frame tag
//!
//! \details 4 bit field inside SPIFSI frame is available to set
//! tag value(0-15)
//!
typedef enum
{
    SPIFSI_FRAME_TAG0  = 0x0000U,
    SPIFSI_FRAME_TAG1  = 0x0001U,
    SPIFSI_FRAME_TAG2  = 0x0002U,
    SPIFSI_FRAME_TAG3  = 0x0003U,
    SPIFSI_FRAME_TAG4  = 0x0004U,
    SPIFSI_FRAME_TAG5  = 0x0005U,
    SPIFSI_FRAME_TAG6  = 0x0006U,
    SPIFSI_FRAME_TAG7  = 0x0007U,
    SPIFSI_FRAME_TAG8  = 0x0008U,
    SPIFSI_FRAME_TAG9  = 0x0009U,
    SPIFSI_FRAME_TAG10 = 0x000AU,
    SPIFSI_FRAME_TAG11 = 0x000BU,
    SPIFSI_FRAME_TAG12 = 0x000CU,
    SPIFSI_FRAME_TAG13 = 0x000DU,
    SPIFSI_FRAME_TAG14 = 0x000EU,
    SPIFSI_FRAME_TAG15 = 0x000FU
} SPIFSI_FrameTag;

//! \brief FSI frame structure in SPI compatibility mode.
//!
typedef struct
{
    SPIFSI_FrameType    frameType;
    //!< readFrame: output. writeFrame: input.

    uint16_t            userData;
    //!< readFrame: output. writeFrame: input.

    uint16_t            *data;
    //!< readFrame: input(address)/output(data filled). writeFrame: input(address and data filled).

    uint16_t            nLength;
    //!< readFrame: input. writeFrame: input. (required only when type is N_WORD).

    uint16_t            crc8;
    //!< readFrame: output. writeFrame: output.

    SPIFSI_FrameTag     frameTag;
    //!< readFrame: output. writeFrame: input.
}SPIFSI_FrameInfo;

//! \brief ERROR enum type returned by SPIFSI_readFrame() and SPIFSI_writeFrame().
//!
typedef enum
{
    SPIFSI_NO_ERROR         = 0x0000U,
    SPIFSI_SOF_ERROR        = 0x0001U,
    SPIFSI_F_TYPE_ERROR     = 0x0002U,
    SPIFSI_CRC_ERROR        = 0x0003U,
    SPIFSI_EOF_ERROR        = 0x0004U,
    SPIFSI_N_RANGE_ERROR    = 0x0005U,
    SPIFSI_READ_ERROR       = 0x0006U,
    SPIFSI_WRITE_ERROR      = 0x0007U
} SPIFSI_Error;

//*****************************************************************************
//
//! \brief Reads/Receives frame from FSI.
//!
//! \param base is base address of SPI
//! \param frameInfo is a pointer to the \b SPIFSI_FrameInfo object.
//!
//! First, this function reads-in the first word which contains frameType.
//! If frameType is \b SPIFSI_FRAME_TYPE_PING or ERROR, it populates frameType
//! and frameTag fields of frameInfo then returns. Else, it reads-in data and
//! calculates crc8. If crc8 is valid, it populates frameType, userData,
//! data, crc8, and frameTag of frameInfo, then returns.
//!
//! \note The user must populate nLength field if frameType is N_WORD and
//! initialize the *data field prior to function call.
//!
//! \return If there is an error within the received frame, it returns one of
//! the listed \b SPIFSI_Error enum. Otherwise, it returns \b SPIFSI_NO_ERROR.
//
//*****************************************************************************
extern SPIFSI_Error SPIFSI_readFrame(uint32_t base,
                                     SPIFSI_FrameInfo *frameInfo);

//*****************************************************************************
//
//! \brief Writes/sends frame to FSI.
//!
//! \param base is base address of SPI
//! \param frameInfo is a pointer to the \b SPIFSI_FrameInfo object.
//!
//! First, if a frameType, a field of frameInfo parameter, is
//! \b SPIFSI_FRAME_TYPE_PING or ERROR, this function sends a word consisting of
//! sof, frameType, frameTag, and eof to FSI. Else, it calculates crc8 and if
//! valid, it then sends a frame consisting of sof, frameType, userData, data,
//! crc, frameTag, and eof to FSI.
//!
//! \note The user must populate every field of frameInfo except crc8 prior to
//! function call.
//!
//! \return If there is an error within the received frame, it returns one of
//! the listed \b SPIFSI_Error enum. Otherwise, it returns \b SPIFSI_NO_ERROR.
//
//*****************************************************************************
extern SPIFSI_Error SPIFSI_writeFrame(uint32_t base,
                                      SPIFSI_FrameInfo *frameInfo);

//*****************************************************************************
//
//! \brief reads 16-bit word.
//!
//! \param base is base address of SPI.
//! \param data16 is a pointer to 16-bit word.
//!
//! This function reads 16-bit word from SPI and assigns the word to *data16.
//! If any error occurs during reading, this function returns non-zero 16-bit
//! int. Otherwise, it returns zero 16-bit int.
//!
//! \note The user must define the function body per device.
//!
//! \return If there is an error while reading, it returns non-zero 16-bit int.
//! Otherwise, it returns zero 16-bit int.
//
//*****************************************************************************
extern uint16_t SPI_read16Bits(uint32_t base, uint16_t *data16);

//*****************************************************************************
//
//! \brief writes 16-bit word.
//!
//! \param base is base address of SPI.
//! \param data16 is a word to be written.
//!
//! This function writes 16-bit word, data16, to SPI. If any error occurs during
//! reading, this function returns non-zero 16-bit int. Otherwise, it returns
//! zero 16-bit int.
//!
//! \note The user must define the function body per device.
//!
//! \return If there is an error while reading, it returns non-zero 16-bit int.
//! Otherwise, it returns zero 16-bit int.
//
//*****************************************************************************
extern uint16_t SPI_write16Bits(uint32_t base, uint16_t *data16);

//*****************************************************************************
//
//! \brief VCU(ASM)- function to get the 8-bit CRC
//!
//! Calculate the 8-bit CRC of a message buffer by using the VCU instructions,
//! VCRC8L_1 and VCRC8H_1
//!
//! \param inputCRC8Accum The seed value for the CRC, in the event of a
//!  multi-part message, the result of the previous crc8 can be used as
//!  the initial value for the current segment crc8 calculation
//!  until the final crc is derived.
//! \param msg Address of the message buffer
//! \param parity Parity of the first message word. The parity determines whether
//!               the CRC begins at the low byte (even parity, 0U) or at the high
//!               byte (odd parity, 1U) of the first word
//! determines whether the CRC begins at the low byte (EVEN) or at the high byte (ODD).
//! \param rxLen Length of the message in bytes
//! \return CRC result
//
//*****************************************************************************
extern uint16_t SPIFSI_calcCRC8(uint32_t inputCRC8Accum, uint16_t *msg,
                                uint16_t parity, uint16_t rxLen);

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

#endif

#endif // SPIFSI_H

//
// End of File
//
