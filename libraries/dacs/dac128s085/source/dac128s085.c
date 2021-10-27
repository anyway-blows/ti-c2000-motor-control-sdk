//#############################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:26 CDT 2021 $
// $Copyright:
// Copyright (C) 2017-2019 Texas Instruments Incorporated - http://www.ti.com/
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

//! \file   \libraries\dacs\dac128s085\source\dac128s085.c
//! \brief  Contains the various functions related to the dacs128s085 object
//!

// **************************************************************************
// the includes

#include <math.h>

// **************************************************************************
// drivers
#include "dac128s085.h"


// **************************************************************************
// modules

// **************************************************************************
// platforms

// **************************************************************************
// the defines

// **************************************************************************
// the globals

// **************************************************************************
// the function prototypes


DAC128S_Handle DAC128S_init(void *pMemory)
{
    DAC128S_Handle handle;
    DAC128S_Obj *obj;

    // assign the handle
    handle = (DAC128S_Handle)pMemory;
    obj = (DAC128S_Obj *)handle;

#if defined(DAC128S_SPIA)
    // assign the SPI handle
    obj->spiHandle = SPIA_BASE;
#elif defined(DAC128S_SPIB)
    // assign the SPI handle
    obj->spiHandle = SPIB_BASE;
#endif

    obj->channelNum = DAC128S_CHN_NUM_EN;

    return(handle);
} // end of DAC128S_init() function


void DAC128S_setupSPI(DAC128S_Handle handle)
{
    DAC128S_Obj *obj = (DAC128S_Obj *)handle;

    // Must put SPI into reset before configuring it
    SPI_disableModule(obj->spiHandle);

    // SPI configuration. Use a 10.0MHz SPICLK and 16-bit word size, 50MHz LSPCLK
    SPI_setConfig(obj->spiHandle, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA0,
                  SPI_MODE_MASTER, 10000000, 16);

    SPI_disableLoopback(obj->spiHandle);

    SPI_setEmulationMode(obj->spiHandle, SPI_EMULATION_FREE_RUN);

    SPI_enableFIFO(obj->spiHandle);
    SPI_setTxFifoTransmitDelay(obj->spiHandle, 0x04);

    SPI_clearInterruptStatus(obj->spiHandle, SPI_INT_TXFF);

    // Configuration complete. Enable the module.
    SPI_enableModule(obj->spiHandle);

    return;
}  // end of HAL_setupSPI() function


void DAC128S_writeCommand(DAC128S_Handle handle)
{
    DAC128S_Obj *obj = (DAC128S_Obj *)handle;
    uint16_t ctrlWord;

    // reset the Rx fifo pointer to zero
    SPI_resetRxFIFO(obj->spiHandle);
    SPI_enableFIFO(obj->spiHandle);
    SPI_clearInterruptStatus(obj->spiHandle, SPI_INT_TXFF);

    // build the control word
    ctrlWord = (uint16_t)DAC128S_buildCtrlWord(DAC128S_MODE_WTM, 0, 0);

    // write the command
    SPI_writeDataNonBlocking(obj->spiHandle, ctrlWord);

    return;
}  // end of DAC128S_writeData() function


void DAC128S_writeData(DAC128S_Handle handle)
{
    DAC128S_Obj *obj = (DAC128S_Obj *)handle;
    uint16_t cnt;
    uint16_t ctrlWord;

    float32_t dacData;

    // reset the Rx fifo pointer to zero
    SPI_resetRxFIFO(obj->spiHandle);
    SPI_enableFIFO(obj->spiHandle);
    SPI_clearInterruptStatus(obj->spiHandle, SPI_INT_TXFF);

    for(cnt = 0; cnt < obj->channelNum; cnt++)
    {
        dacData = (*obj->ptrData[cnt]);
        obj->dacData[cnt] = (int16_t)(dacData * obj->gain[cnt]) + obj->offset[cnt];

        // build the control word
        ctrlWord = (uint16_t)DAC128S_buildCtrlWord(DAC128S_MODE_WRD, cnt, obj->dacData[cnt]);

        // write the command
        SPI_writeDataNonBlocking(obj->spiHandle, ctrlWord);
    }

    return;
}  // end of DAC128S_writeData() function


// end of file
