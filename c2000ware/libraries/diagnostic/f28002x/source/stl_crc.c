//###########################################################################
//
// FILE:  stl_crc.c
//
// TITLE: Diagnostic Library CRC software module source
//
//###########################################################################
// $TI Release: C2000 Diagnostic Library v2.01.00 $
// $Release Date: Fri Feb 12 19:23:23 IST 2021 $
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

//
// Includes
//
#include "stl_crc.h"

//*****************************************************************************
//
// STL_CRC_checkCRC(const uint32_t startAddress, const uint32_t endAddress,
//                  const uint32_t goldenCRC)
//
//*****************************************************************************
uint16_t STL_CRC_checkCRC(const uint32_t startAddress,
                          const uint32_t endAddress,
                          const uint32_t goldenCRC)
{
    uint16_t testStatus;
    STL_CRC_Obj crcObj;

    //
    // Calculate number of bytes to calculate a CRC on, inclusive of the end
    // address. Remember words are two bytes.
    //
    uint16_t numBytes = (endAddress - startAddress + 1U) << 1;

    //
    // Calculate the CRC of the received data using the VCU.
    //
    // Step 1: Initialize the CRC object.
    //
    crcObj.seedValue    = STL_CRC_INIT_CRC;
    crcObj.numBytes     = numBytes;
    crcObj.parity       = STL_CRC_PARITY_EVEN;
    crcObj.crcResult    = 0U;
    crcObj.msgBuffer    = (void *)startAddress;

    //
    // Step 2: Run the 32-bit CRC on the received data.
    //
    STL_CRC_reset();

    STL_CRC_calculate(&crcObj);

    //
    // Compare calculated CRC and golden CRC.
    //
    if(goldenCRC != crcObj.crcResult)
    {
        //
        // Report global error.
        //
        STL_Util_setErrorFlag(STL_UTIL_CRC);
        testStatus = STL_CRC_FAIL;
    }
    else
    {
        testStatus = STL_CRC_PASS;
    }

    return(testStatus);
}

//
// End of File
//
