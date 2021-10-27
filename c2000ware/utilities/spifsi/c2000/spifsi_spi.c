//#############################################################################
//
// FILE: spifsi_spi.c
//
// TITLE: SPI to FSI communication user-specific file
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

#include "spifsi_spi.h"


//*****************************************************************************
//
// SPIFSI_calcCrc8 - calculate CRC8 by either using VCU or not.
//
//*****************************************************************************
#ifdef VCU_CRC
uint16_t SPIFSI_calcCRC8(uint32_t input_crc8_accum, uint16_t *msg, uint16_t parity, uint16_t rxLen) {
    return((uint16_t) getCRC8_vcu(input_crc8_accum, msg, (CRC_parity_e)parity, rxLen));
}
#else
uint16_t SPIFSI_calcCRC8(uint32_t input_crc8_accum, uint16_t *msg, uint16_t parity, uint16_t rxLen) {
    //
    // Build CRC struct.
    //
    CRC_Obj crcObj;
    crcObj.seedValue = input_crc8_accum;
    crcObj.numBytes = rxLen;
    crcObj.parity = (CRC_Parity) parity;
    crcObj.msgBuffer = msg;

    //
    // Point to CRC struct.
    //
    CRC_Handle hndCRC = &crcObj;

    //
    // Calculate.
    //
    CRC_calculate(hndCRC);

    return(hndCRC->crcResult);
}
#endif

//
// End of File
//


