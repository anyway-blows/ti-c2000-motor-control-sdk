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

//! \file   libraries/drvic/drv8353/source/drv8353s.c
//! \brief  Contains the various functions related to the DRV8353 object
//!

// **************************************************************************
// the includes

#include <math.h>

// **************************************************************************
// drivers
#include "drv8353s.h"

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

DRV8353_Handle DRV8353_init(void *pMemory)
{
    DRV8353_Handle handle;

    // assign the handle
    handle = (DRV8353_Handle)pMemory;

    DRV8353_resetRxTimeout(handle);
    DRV8353_resetEnableTimeout(handle);

    return(handle);
} // end of DRV8353_init() function

void DRV8353_enable(DRV8353_Handle handle)
{
    DRV8353_Obj *obj = (DRV8353_Obj *)handle;
    volatile uint16_t enableWaitTimeOut;
    uint16_t n = 0;

    // Disable the DRV8353
    GPIO_writePin(obj->gpioNumber_EN, 0);

    // Wait for the DRV8353 to go through start up sequence
    for(n=0;n<0x3fff;n++)
    {
      asm(" NOP");
    }

    // Enable the DRV8353
    GPIO_writePin(obj->gpioNumber_EN, 1);

    enableWaitTimeOut = 0;

    // Make sure the FAULT bit is not set during startup
    while(((DRV8353_readSPI(handle, DRV8353_ADDRESS_STATUS_0) &
            DRV8353_STATUS00_FAULT_BITS) != 0) && (enableWaitTimeOut < 1000))
    {
        if(++enableWaitTimeOut > 999)
        {
            obj->enableTimeOut = true;
        }
    }

    // Wait for the DRV8353 to go through start up sequence
    for(n = 0; n < 0xffff; n++)
    {
        __asm(" NOP");
    }

    return;
} // end of DRV8353_enable() function


DRV8353_CTRL03_PeakSourCurHS_e DRV8353_getPeakSourCurHS(DRV8353_Handle handle)
{
    uint16_t data;

    // read data
    data = DRV8353_readSPI(handle, DRV8353_ADDRESS_CONTROL_3);

    // mask the bits
    data &= DRV8353_CTRL03_IDRIVEP_HS_BITS;

    return((DRV8353_CTRL03_PeakSourCurHS_e)data);
} // end of DRV8353_getPeakSourCurHS function

DRV8353_CTRL03_PeakSinkCurHS_e DRV8353_getPeakSinkCurHS(DRV8353_Handle handle)
{
    uint16_t data;

    // read data
    data = DRV8353_readSPI(handle, DRV8353_ADDRESS_CONTROL_3);

    // mask the bits
    data &= DRV8353_CTRL03_IDRIVEN_HS_BITS;

    return((DRV8353_CTRL03_PeakSinkCurHS_e)data);
} // end of DRV8353_getPeakSinkCurHS function

DRV8353_CTRL04_PeakTime_e DRV8353_getPeakSourTime(DRV8353_Handle handle)
{
    uint16_t data;

    // read data
    data = DRV8353_readSPI(handle, DRV8353_ADDRESS_CONTROL_4);

    // mask the bits
    data &= DRV8353_CTRL04_TDRIVE_BITS;

    return((DRV8353_CTRL04_PeakTime_e)data);
} // end of DRV8353_getPeakSourTime function

DRV8353_CTRL04_PeakSourCurLS_e DRV8353_getPeakSourCurLS(DRV8353_Handle handle)
{
    uint16_t data;

    // read data
    data = DRV8353_readSPI(handle, DRV8353_ADDRESS_CONTROL_4);

    // mask the bits
    data &= DRV8353_CTRL04_IDRIVEP_LS_BITS;

    return((DRV8353_CTRL04_PeakSourCurLS_e)data);
} // end of DRV8353_getPeakSourCurLS function

DRV8353_CTRL04_PeakSinkCurLS_e DRV8353_getPeakSinkCurLS(DRV8353_Handle handle)
{
    uint16_t data;

    // read data
    data = DRV8353_readSPI(handle, DRV8353_ADDRESS_CONTROL_4);

    // mask the bits
    data &= DRV8353_CTRL04_IDRIVEN_LS_BITS;

    return((DRV8353_CTRL04_PeakSinkCurLS_e)data);
} // end of DRV8353_getPeakSinkCurLS function


DRV8353_CTRL05_OcpDeg_e DRV8353_getVDSDeglitch(DRV8353_Handle handle)
{
    uint16_t data;

    // read data
    data = DRV8353_readSPI(handle, DRV8353_ADDRESS_CONTROL_5);

    // mask the bits
    data &= DRV8353_CTRL05_OCP_DEG_BITS;

    return((DRV8353_CTRL05_OcpDeg_e)data);
} // end of DRV8353_getVDSDeglitch function

DRV8353_CTRL05_DeadTime_e DRV8353_getDeadTime(DRV8353_Handle handle)
{
    uint16_t data;

    // read data
    data = DRV8353_readSPI(handle, DRV8353_ADDRESS_CONTROL_5);

    // mask the bits
    data &= DRV8353_CTRL05_DEAD_TIME_BITS;

    return((DRV8353_CTRL05_DeadTime_e)data);
} // end of DRV8353_getDeadTime function

DRV8353_CTRL02_PWMMode_e DRV8353_getPWMMode(DRV8353_Handle handle)
{
    uint16_t data;

    // read data
    data = DRV8353_readSPI(handle, DRV8353_ADDRESS_CONTROL_2);

    // mask the bits
    data &= DRV8353_CTRL02_PWM_MODE_BITS;

    return((DRV8353_CTRL02_PWMMode_e)data);
} // end of DRV8353_getPWMMode function

void DRV8353_setSPIHandle(DRV8353_Handle handle, uint32_t spiHandle)
{
    DRV8353_Obj *obj = (DRV8353_Obj *)handle;

    // initialize the serial peripheral interface object
    obj->spiHandle = spiHandle;

    return;
} // end of DRV8353_setSPIHandle() function

void DRV8353_setGPIOCSNumber(DRV8353_Handle handle, uint32_t gpioNumber)
{
    DRV8353_Obj *obj = (DRV8353_Obj *)handle;

    // initialize the gpio interface object
    obj->gpioNumber_CS = gpioNumber;

    return;
} // end of DRV8353_setGPIOCSNumber() function

void DRV8353_setGPIOENNumber(DRV8353_Handle handle, uint32_t gpioNumber)
{
    DRV8353_Obj *obj = (DRV8353_Obj *)handle;

    // initialize the gpio interface object
    obj->gpioNumber_EN = gpioNumber;

    return;
} // end of DRV8353_setGPIOENNumber() function

void DRV8353_setupSPI(DRV8353_Handle handle,
                      DRV8353_VARS_t *drv8353Vars)
{
    DRV8353_Address_e drvRegAddr;
    uint16_t drvDataNew;

    // Set Default Values
    // Manual Read/Write
    drv8353Vars->manReadAddr  = 0;
    drv8353Vars->manReadData  = 0;
    drv8353Vars->manReadCmd = false;
    drv8353Vars->manWriteAddr = 0;
    drv8353Vars->manWriteData = 0;
    drv8353Vars->manWriteCmd = false;

    // Read/Write
    drv8353Vars->readCmd  = false;
    drv8353Vars->writeCmd = false;

    // Read registers for default values
    // Read Status Register 0
    drvRegAddr = DRV8353_ADDRESS_STATUS_0;
    drvDataNew = DRV8353_readSPI(handle, drvRegAddr);
    drv8353Vars->statReg00.all = drvDataNew;

    // Read Status Register 1
    drvRegAddr = DRV8353_ADDRESS_STATUS_1;
    drvDataNew = DRV8353_readSPI(handle, drvRegAddr);
    drv8353Vars->statReg01.all = drvDataNew;

      // Read Control Register 2
    // all bit default value are 0, 6*PWM Mode
    drvRegAddr = DRV8353_ADDRESS_CONTROL_2;
    drvDataNew = DRV8353_readSPI(handle, drvRegAddr);
    drv8353Vars->ctrlReg02.all = drvDataNew;

    // Read Control Register 3
    // all bit default value are 1, IDRIVEP_HS=1000mA, IDRIVEN_HS = 2000mA
    drvRegAddr = DRV8353_ADDRESS_CONTROL_3;
    drvDataNew = DRV8353_readSPI(handle, drvRegAddr);
    drv8353Vars->ctrlReg03.all = drvDataNew;

    // Read Control Register 4
    // all bit default value are 1, TDRIVE=400ns, IDRIVEP_LS=1000mA, IDRIVEN_LS = 2000mA
    drvRegAddr = DRV8353_ADDRESS_CONTROL_4;
    drvDataNew = DRV8353_readSPI(handle, drvRegAddr);
    drv8353Vars->ctrlReg04.all = drvDataNew;

    // Read Control Register 5
    // DEAD_TIME=100ns, OCP_DEG=4us, VDS_LVL=0.75V
    drvRegAddr = DRV8353_ADDRESS_CONTROL_5;
    drvDataNew = DRV8353_readSPI(handle, drvRegAddr);
    drv8353Vars->ctrlReg05.all = drvDataNew;

    // Read Control Register 6
    // DEAD_TIME=100ns, OCP_DEG=4us, VDS_LVL=0.75V
    drvRegAddr = DRV8353_ADDRESS_CONTROL_6;
    drvDataNew = DRV8353_readSPI(handle, drvRegAddr);
    drv8353Vars->ctrlReg06.all = drvDataNew;

    // Read Control Register 7
    drvRegAddr = DRV8353_ADDRESS_CONTROL_7;
    drvDataNew = DRV8353_readSPI(handle, drvRegAddr);
    drv8353Vars->ctrlReg07.all = drvDataNew;

    return;
} // end of DRV8353_setupSPI() function

uint16_t DRV8353_readSPI(DRV8353_Handle handle,
                         const DRV8353_Address_e regAddr)
{
    DRV8353_Obj *obj = (DRV8353_Obj *)handle;
    uint16_t ctrlWord;
    uint16_t n;
    const uint16_t data = 0;
    volatile uint16_t readWord;
    volatile uint16_t WaitTimeOut = 0;

    volatile SPI_RxFIFOLevel RxFifoCnt = SPI_FIFO_RXEMPTY;

    // build the control word
    ctrlWord = (uint16_t)DRV8353_buildCtrlWord(DRV8353_CTRLMODE_READ, regAddr, data);

//    GPIO_writePin(obj->gpioNumber_CS, 0);
//    GPIO_writePin(obj->gpioNumber_CS, 0);

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

//    GPIO_writePin(obj->gpioNumber_CS, 1);
//    GPIO_writePin(obj->gpioNumber_CS, 1);

    // Read the word
    readWord = SPI_readDataNonBlocking(obj->spiHandle);

    return(readWord & DRV8353_DATA_MASK);
} // end of DRV8353_readSPI() function


void DRV8353_writeSPI(DRV8353_Handle handle, const DRV8353_Address_e regAddr,
                      const uint16_t data)
{
    DRV8353_Obj *obj = (DRV8353_Obj *)handle;
    uint16_t ctrlWord;
    uint16_t n;

    // build the control word
    ctrlWord = (uint16_t)DRV8353_buildCtrlWord(DRV8353_CTRLMODE_WRITE, regAddr, data);

//    GPIO_writePin(obj->gpioNumber_CS, 0);
//    GPIO_writePin(obj->gpioNumber_CS, 0);

    // wait for GPIO
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

    // wait for registers to update
    for(n = 0; n < 0x280; n++)
    {
        __asm(" NOP");
    }

//    GPIO_writePin(obj->gpioNumber_CS, 1);
//    GPIO_writePin(obj->gpioNumber_CS, 1);

    return;
}  // end of DRV8353_writeSPI() function


void DRV8353_writeData(DRV8353_Handle handle, DRV8353_VARS_t *drv8353Vars)
{
    DRV8353_Address_e drvRegAddr;
    uint16_t drvDataNew;

    if(drv8353Vars->writeCmd)
    {
        // Write Control Register 2
        drvRegAddr = DRV8353_ADDRESS_CONTROL_2;
        drvDataNew = drv8353Vars->ctrlReg02.all & DRV8353_DATA_MASK;
        DRV8353_writeSPI(handle, drvRegAddr, drvDataNew);

        // Write Control Register 3
        drvRegAddr = DRV8353_ADDRESS_CONTROL_3;
        drvDataNew = drv8353Vars->ctrlReg03.all & DRV8353_DATA_MASK;
        DRV8353_writeSPI(handle, drvRegAddr, drvDataNew);

        // Write Control Register 4
        drvRegAddr = DRV8353_ADDRESS_CONTROL_4;
        drvDataNew = drv8353Vars->ctrlReg04.all & DRV8353_DATA_MASK;
        DRV8353_writeSPI(handle, drvRegAddr, drvDataNew);

        // Write Control Register 5
        drvRegAddr = DRV8353_ADDRESS_CONTROL_5;
        drvDataNew = drv8353Vars->ctrlReg05.all & DRV8353_DATA_MASK;
        DRV8353_writeSPI(handle, drvRegAddr, drvDataNew);

        // Write Control Register 6
        drvRegAddr = DRV8353_ADDRESS_CONTROL_6;
        drvDataNew = drv8353Vars->ctrlReg06.all & DRV8353_DATA_MASK;
        DRV8353_writeSPI(handle, drvRegAddr, drvDataNew);

        // Write Control Register 7
        drvRegAddr = DRV8353_ADDRESS_CONTROL_7;
        drvDataNew = drv8353Vars->ctrlReg07.all & DRV8353_DATA_MASK;
        DRV8353_writeSPI(handle, drvRegAddr, drvDataNew);

        drv8353Vars->writeCmd = false;
    }

    // Manual write to the DRV8353
    if(drv8353Vars->manWriteCmd)
    {
        // Custom Write
        drvRegAddr = (DRV8353_Address_e)(drv8353Vars->manWriteAddr << 11);
        drvDataNew = drv8353Vars->manWriteData;
        DRV8353_writeSPI(handle, drvRegAddr, drvDataNew);

        drv8353Vars->manWriteCmd = false;
    }

    return;
}  // end of DRV8353_writeData() function

void DRV8353_readData(DRV8353_Handle handle, DRV8353_VARS_t *drv8353Vars)
{
    DRV8353_Address_e drvRegAddr;
    uint16_t drvDataNew;

    if(drv8353Vars->readCmd)
    {
        // Read registers for default values
        // Read Status Register 0
        drvRegAddr = DRV8353_ADDRESS_STATUS_0;
        drvDataNew = DRV8353_readSPI(handle, drvRegAddr);
        drv8353Vars->statReg00.all  = drvDataNew;

        // Read Status Register 1
        drvRegAddr = DRV8353_ADDRESS_STATUS_1;
        drvDataNew = DRV8353_readSPI(handle, drvRegAddr);
        drv8353Vars->statReg01.all  = drvDataNew;

        // Read Control Register 2
        drvRegAddr = DRV8353_ADDRESS_CONTROL_2;
        drvDataNew = DRV8353_readSPI(handle, drvRegAddr);
        drv8353Vars->ctrlReg02.all  = drvDataNew;

        // Read Control Register 3
        drvRegAddr = DRV8353_ADDRESS_CONTROL_3;
        drvDataNew = DRV8353_readSPI(handle, drvRegAddr);
        drv8353Vars->ctrlReg03.all  = drvDataNew;

        // Read Control Register 4
        drvRegAddr = DRV8353_ADDRESS_CONTROL_4;
        drvDataNew = DRV8353_readSPI(handle, drvRegAddr);
        drv8353Vars->ctrlReg04.all  = drvDataNew;

        // Read Control Register 5
        drvRegAddr = DRV8353_ADDRESS_CONTROL_5;
        drvDataNew = DRV8353_readSPI(handle, drvRegAddr);
        drv8353Vars->ctrlReg05.all  = drvDataNew;

        // Read Control Register 6
        drvRegAddr = DRV8353_ADDRESS_CONTROL_6;
        drvDataNew = DRV8353_readSPI(handle, drvRegAddr);
        drv8353Vars->ctrlReg06.all  = drvDataNew;

        drv8353Vars->readCmd = false;
    }

    // Manual read from the DRV8353
    if(drv8353Vars->manReadCmd)
    {
        // Custom Read
        drvRegAddr = (DRV8353_Address_e)(drv8353Vars->manReadAddr << 11);
        drvDataNew = DRV8353_readSPI(handle, drvRegAddr);
        drv8353Vars->manReadData = drvDataNew;

        drv8353Vars->manReadCmd = false;
    }

    return;
}  // end of DRV8353_readData() function

// end of file
