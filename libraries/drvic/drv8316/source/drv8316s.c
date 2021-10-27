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

//! \file   libraries/drvic/drv8316/source/drv8316s.c
//! \brief  Contains the various functions related to the DRV8316 object
//!

// **************************************************************************
// the includes

#include <math.h>

// **************************************************************************
// drivers
#include "drv8316s.h"

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

DRV8316_Handle DRV8316_init(void *pMemory)
{
    DRV8316_Handle handle;

    // assign the handle
    handle = (DRV8316_Handle)pMemory;

    DRV8316_resetRxTimeout(handle);
    DRV8316_resetEnableTimeout(handle);

    return(handle);
} // end of DRV8316_init() function

void DRV8316_enable(DRV8316_Handle handle)
{
    DRV8316_Obj *obj = (DRV8316_Obj *)handle;
    volatile uint16_t enableWaitTimeOut;
    uint16_t n = 0;

    // Enable the DRV8316
    GPIO_writePin(obj->gpioNumber_EN, 1);
    GPIO_writePin(obj->gpioNumber_EN, 1);

    enableWaitTimeOut = 0;

    // Make sure the FAULT bit is not set during startup
    while(((DRV8316_readSPI(handle, DRV8316_ADDRESS_STATUS_0) &
            DRV8316_STAT00_FAULT_BITS) != 0) && (enableWaitTimeOut < 1000))
    {
        if(++enableWaitTimeOut > 999)
        {
            obj->enableTimeOut = true;
        }
    }

    // Wait for the DRV8316 to go through start up sequence
    for(n = 0; n < 0xffff; n++)
    {
        __asm(" NOP");
    }

    // Write 011b to this register to unlock all registers
    DRV8316_writeSPI(handle,  DRV8316_ADDRESS_CONTROL_1, 0x03);

    // Clear Fault, Slew rate is 200 V/μs
    DRV8316_writeSPI(handle,  DRV8316_ADDRESS_CONTROL_2, 0x19);

    return;
} // end of DRV8316_enable() function

void DRV8316_setSPIHandle(DRV8316_Handle handle, uint32_t spiHandle)
{
    DRV8316_Obj *obj = (DRV8316_Obj *)handle;

    // initialize the serial peripheral interface object
    obj->spiHandle = spiHandle;

    return;
} // end of DRV8316_setSPIHandle() function

void DRV8316_setGPIOCSNumber(DRV8316_Handle handle, uint32_t gpioNumber)
{
    DRV8316_Obj *obj = (DRV8316_Obj *)handle;

    // initialize the gpio interface object
    obj->gpioNumber_CS = gpioNumber;

    return;
} // end of DRV8316_setGPIOCSNumber() function

void DRV8316_setGPIOENNumber(DRV8316_Handle handle, uint32_t gpioNumber)
{
    DRV8316_Obj *obj = (DRV8316_Obj *)handle;

    // initialize the gpio interface object
    obj->gpioNumber_EN = gpioNumber;

    return;
} // end of DRV8316_setGPIOENNumber() function

void DRV8316_setupSPI(DRV8316_Handle handle,
                      DRV8316_VARS_t *drv8316Vars)
{
    DRV8316_Address_e drvRegAddr;
    uint16_t drvDataNew;

    // Set Default Values
    // Manual Read/Write
    drv8316Vars->manReadAddr  = 0;
    drv8316Vars->manReadData  = 0;
    drv8316Vars->manReadCmd = false;
    drv8316Vars->manWriteAddr = 0;
    drv8316Vars->manWriteData = 0;
    drv8316Vars->manWriteCmd = false;

    // Read/Write
    drv8316Vars->readCmd  = false;
    drv8316Vars->writeCmd = false;

    // Read registers for default values
    // Read Status Register 0
    drvRegAddr = DRV8316_ADDRESS_STATUS_0;
    drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
    drv8316Vars->statReg00.all = drvDataNew;

    // Read Status Register 1
    drvRegAddr = DRV8316_ADDRESS_STATUS_1;
    drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
    drv8316Vars->statReg01.all = drvDataNew;

    // Read Status Register 2
    drvRegAddr = DRV8316_ADDRESS_STATUS_2;
    drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
    drv8316Vars->statReg02.all = drvDataNew;

      // Read Control Register 1
    drvRegAddr = DRV8316_ADDRESS_CONTROL_1;
    drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
    drv8316Vars->ctrlReg01.all = drvDataNew;

    // Read Control Register 2
    drvRegAddr = DRV8316_ADDRESS_CONTROL_2;
    drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
    drv8316Vars->ctrlReg02.all = drvDataNew;

    // Read Control Register 3
    drvRegAddr = DRV8316_ADDRESS_CONTROL_3;
    drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
    drv8316Vars->ctrlReg02.all = drvDataNew;

    // Read Control Register 4
    drvRegAddr = DRV8316_ADDRESS_CONTROL_4;
    drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
    drv8316Vars->ctrlReg04.all = drvDataNew;

    // Read Control Register 5
    drvRegAddr = DRV8316_ADDRESS_CONTROL_5;
    drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
    drv8316Vars->ctrlReg05.all = drvDataNew;

    // Read Control Register 6
    drvRegAddr = DRV8316_ADDRESS_CONTROL_6;
    drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
    drv8316Vars->ctrlReg06.all = drvDataNew;

    // Read Control Register 10
    drvRegAddr = DRV8316_ADDRESS_CONTROL_10;
    drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
    drv8316Vars->ctrlReg10.all = drvDataNew;

    return;
} // end of DRV8316_setupSPI() function

uint16_t DRV8316_readSPI(DRV8316_Handle handle,
                         const DRV8316_Address_e regAddr)
{
    DRV8316_Obj *obj = (DRV8316_Obj *)handle;
    uint16_t ctrlWord;
    uint16_t n;
    const uint16_t data = 0;
    volatile uint16_t readWord;
    volatile uint16_t WaitTimeOut = 0;

    volatile SPI_RxFIFOLevel RxFifoCnt = SPI_FIFO_RXEMPTY;

    // build the control word
    ctrlWord = (uint16_t)DRV8316_buildCtrlWord(DRV8316_CTRLMODE_READ, regAddr, data);

#ifdef DRV_CS_GPIO
    GPIO_writePin(obj->gpioNumber_CS, 0);
    GPIO_writePin(obj->gpioNumber_CS, 0);
#endif  // DRV_CS_GPIO

    // wait for registers to update
    for(n = 0; n < 0x08; n++)
    {
        __asm(" NOP");
    }

    // reset the Rx fifo pointer to zero
    SPI_resetRxFIFO(obj->spiHandle);
    SPI_enableFIFO(obj->spiHandle);

    // wait for registers to update
    for(n = 0; n < 0x20; n++)
    {
        __asm(" NOP");
    }

    // write the command
    SPI_writeDataBlockingNonFIFO(obj->spiHandle, ctrlWord);

    // wait for two words to populate the RX fifo, or a wait timeout will occur
    while(RxFifoCnt < SPI_FIFO_RX1)
    {
        RxFifoCnt = SPI_getRxFIFOStatus(obj->spiHandle);

        if(++WaitTimeOut > 0xfffe)
        {
            obj->rxTimeOut = true;
        }
    }

    WaitTimeOut = 0xffff;

    // wait for registers to update
    for(n = 0; n < 0x100; n++)
    {
        __asm(" NOP");
    }

#ifdef DRV_CS_GPIO
    GPIO_writePin(obj->gpioNumber_CS, 1);
    GPIO_writePin(obj->gpioNumber_CS, 1);
#endif  // DRV_CS_GPIO

    // Read the word
    readWord = SPI_readDataNonBlocking(obj->spiHandle);

    return(readWord & DRV8316_DATA_MASK);
} // end of DRV8316_readSPI() function


void DRV8316_writeSPI(DRV8316_Handle handle, const DRV8316_Address_e regAddr,
                      const uint16_t data)
{
    DRV8316_Obj *obj = (DRV8316_Obj *)handle;
    uint16_t ctrlWord;
    uint16_t n;

    // build the control word
    ctrlWord = (uint16_t)DRV8316_buildCtrlWord(DRV8316_CTRLMODE_WRITE, regAddr, data);

#ifdef DRV_CS_GPIO
    GPIO_writePin(obj->gpioNumber_CS, 0);
    GPIO_writePin(obj->gpioNumber_CS, 0);
#endif  // DRV_CS_GPIO

    // wait for GPIO
    for(n = 0; n < 0x08; n++)
    {
        __asm(" NOP");
    }

    // reset the Rx fifo pointer to zero
    SPI_resetRxFIFO(obj->spiHandle);
    SPI_enableFIFO(obj->spiHandle);

    // wait for registers to update
    for(n = 0; n < 0x40; n++)
    {
        __asm(" NOP");
    }

    // write the command
    SPI_writeDataBlockingNonFIFO(obj->spiHandle, ctrlWord);

    // wait for registers to update
    for(n = 0; n < 0x100; n++)
    {
        __asm(" NOP");
    }

#ifdef DRV_CS_GPIO
    GPIO_writePin(obj->gpioNumber_CS, 1);
    GPIO_writePin(obj->gpioNumber_CS, 1);
#endif  // DRV_CS_GPIO

    return;
}  // end of DRV8316_writeSPI() function


void DRV8316_writeData(DRV8316_Handle handle, DRV8316_VARS_t *drv8316Vars)
{
    DRV8316_Address_e drvRegAddr;
    uint16_t drvDataNew;

    if(drv8316Vars->writeCmd)
    {
        // Write Control Register 1
        drvRegAddr = DRV8316_ADDRESS_CONTROL_1;
        drvDataNew = drv8316Vars->ctrlReg01.all & DRV8316_DATA_MASK;
        DRV8316_writeSPI(handle, drvRegAddr, drvDataNew);

        // Write Control Register 2
        drvRegAddr = DRV8316_ADDRESS_CONTROL_2;
        drvDataNew = drv8316Vars->ctrlReg02.all & DRV8316_DATA_MASK;
        DRV8316_writeSPI(handle, drvRegAddr, drvDataNew);

        // Write Control Register 3
        drvRegAddr = DRV8316_ADDRESS_CONTROL_3;
        drvDataNew = drv8316Vars->ctrlReg03.all & DRV8316_DATA_MASK;
        DRV8316_writeSPI(handle, drvRegAddr, drvDataNew);

        // Write Control Register 4
        drvRegAddr = DRV8316_ADDRESS_CONTROL_4;
        drvDataNew = drv8316Vars->ctrlReg04.all & DRV8316_DATA_MASK;
        DRV8316_writeSPI(handle, drvRegAddr, drvDataNew);

        // Write Control Register 5
        drvRegAddr = DRV8316_ADDRESS_CONTROL_5;
        drvDataNew = drv8316Vars->ctrlReg05.all & DRV8316_DATA_MASK;
        DRV8316_writeSPI(handle, drvRegAddr, drvDataNew);

        // Write Control Register 6
        drvRegAddr = DRV8316_ADDRESS_CONTROL_6;
        drvDataNew = drv8316Vars->ctrlReg06.all & DRV8316_DATA_MASK;
        DRV8316_writeSPI(handle, drvRegAddr, drvDataNew);

        // Write Control Register 10
        drvRegAddr = DRV8316_ADDRESS_CONTROL_10;
        drvDataNew = drv8316Vars->ctrlReg10.all & DRV8316_DATA_MASK;
        DRV8316_writeSPI(handle, drvRegAddr, drvDataNew);

        drv8316Vars->writeCmd = false;
    }

    // Manual write to the DRV8316
    if(drv8316Vars->manWriteCmd)
    {
        // Custom Write
        drvRegAddr = (DRV8316_Address_e)(drv8316Vars->manWriteAddr << 11);
        drvDataNew = drv8316Vars->manWriteData;
        DRV8316_writeSPI(handle, drvRegAddr, drvDataNew);

        drv8316Vars->manWriteCmd = false;
    }

    return;
}  // end of DRV8316_writeData() function

void DRV8316_readData(DRV8316_Handle handle, DRV8316_VARS_t *drv8316Vars)
{
    DRV8316_Address_e drvRegAddr;
    uint16_t drvDataNew;

    if(drv8316Vars->readCmd)
    {
        // Read registers for default values
        // Read Status Register 0
        drvRegAddr = DRV8316_ADDRESS_STATUS_0;
        drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
        drv8316Vars->statReg00.all  = drvDataNew;

        // Read Status Register 1
        drvRegAddr = DRV8316_ADDRESS_STATUS_1;
        drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
        drv8316Vars->statReg01.all  = drvDataNew;

        // Read Status Register 2
        drvRegAddr = DRV8316_ADDRESS_STATUS_2;
        drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
        drv8316Vars->statReg02.all  = drvDataNew;

        // Read Control Register 1
        drvRegAddr = DRV8316_ADDRESS_CONTROL_1;
        drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
        drv8316Vars->ctrlReg01.all  = drvDataNew;

        // Read Control Register 2
        drvRegAddr = DRV8316_ADDRESS_CONTROL_2;
        drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
        drv8316Vars->ctrlReg02.all  = drvDataNew;

        // Read Control Register 3
        drvRegAddr = DRV8316_ADDRESS_CONTROL_3;
        drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
        drv8316Vars->ctrlReg03.all  = drvDataNew;

        // Read Control Register 4
        drvRegAddr = DRV8316_ADDRESS_CONTROL_4;
        drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
        drv8316Vars->ctrlReg04.all  = drvDataNew;

        // Read Control Register 5
        drvRegAddr = DRV8316_ADDRESS_CONTROL_5;
        drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
        drv8316Vars->ctrlReg05.all  = drvDataNew;

        // Read Control Register 6
        drvRegAddr = DRV8316_ADDRESS_CONTROL_6;
        drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
        drv8316Vars->ctrlReg06.all  = drvDataNew;

        // Read Control Register 10
        drvRegAddr = DRV8316_ADDRESS_CONTROL_10;
        drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
        drv8316Vars->ctrlReg10.all  = drvDataNew;

        drv8316Vars->readCmd = false;
    }

    // Manual read from the DRV8316
    if(drv8316Vars->manReadCmd)
    {
        // Custom Read
        drvRegAddr = (DRV8316_Address_e)(drv8316Vars->manReadAddr << 11);
        drvDataNew = DRV8316_readSPI(handle, drvRegAddr);
        drv8316Vars->manReadData = drvDataNew;

        drv8316Vars->manReadCmd = false;
    }

    return;
}  // end of DRV8316_readData() function

// end of file
