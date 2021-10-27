//#############################################################################
//
// FILE:   spi_ex5_external_loopback.c
//
// TITLE:  SPI Digital Loopback without using FIFOs and Interrupts
//
//! \addtogroup driver_example_list
//! <h1>SPI Digital External Loopback without FIFO Interrupts</h1>
//!
//! This program uses the external loopback between two SPI modules. Both
//! the SPI FIFOs and interrupts are not used in this example. SPIA is
//! configured as a slave and SPI B is configured as master. This example
//! demonstrates full duplex communication where both master and slave transmits
//! and receives data simultaneously.
//!
//!
//! \b External \b Connections \n
//! -GPIO40 and GPIO8  - SPISIMO
//! -GPIO41 and GPIO10 - SPISOMI
//! -GPIO22 and GPIO9  - SPICLK
//! -GPIO23 and GPIO11 - SPISTE
//!
//! \b Watch \b Variables \n
//!  - \b TxData_SPIA - Data send from SPIA (slave)
//!  - \b TxData_SPIB - Data send from SPIB (master)
//!  - \b RxData_SPIA - Data received by SPIA (slave)
//!  - \b RxData_SPIB - Data received by SPIB (master)
//!
//
//#############################################################################
// $TI Release: F28002x Support Library v3.04.00.00 $
// $Release Date: Fri Feb 12 18:58:34 IST 2021 $
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
// Included Files
//
#include "driverlib.h"
#include "device.h"

//
// Function Prototypes
//
void initSPIBMaster(void);
void initSPIASlave(void);
void configGPIOs(void);

//
// Main
//
void main(void)
{
    uint16_t i;

    uint16_t TxData_SPIA[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
    uint16_t RxData_SPIA[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    uint16_t TxData_SPIB[] = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F};
    uint16_t RxData_SPIB[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Disable pin locks and enable internal pullups.
    //
    Device_initGPIO();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // Configure GPIOs for external loopback.
    //
    configGPIOs();

    //
    // Set up SPI B as master, initializing it for FIFO mode
    //
    initSPIBMaster();

    //
    // Set up SPI A as slave, initializing it for FIFO mode
    //
    initSPIASlave();

    //
    // Loop forever. Suspend or place breakpoints to observe the buffers.
    //
    for(i = 0; i < 16; i++)
    {
        //
        // Set the TX buffer of slave SPI.
        //
        SPI_writeDataNonBlocking(SPIA_BASE, TxData_SPIA[i]);

        //
        // Set the the master TX buffer. This triggers the data trasnmission
        //
        SPI_writeDataNonBlocking(SPIB_BASE, TxData_SPIB[i]);

        //
        // Read the received data
        //
        RxData_SPIA[i] = SPI_readDataBlockingNonFIFO(SPIA_BASE);
        RxData_SPIB[i] = SPI_readDataBlockingNonFIFO(SPIB_BASE);

        //
        // Check the received data
        //
        if(RxData_SPIA[i] != TxData_SPIB[i])
        {
            ESTOP0;
        }
        if(RxData_SPIB[i] != TxData_SPIA[i])
        {
            ESTOP0;
        }
    }

    //
    // Loop forever
    //
    while(1);
}

//
// Function to configure SPI B as master with FIFO enabled.
//
void initSPIBMaster(void)
{
    //
    // Must put SPI into reset before configuring it
    //
    SPI_disableModule(SPIB_BASE);

    //
    // SPI configuration. Use a 500kHz SPICLK and 16-bit word size.
    //
    SPI_setConfig(SPIB_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA0,
                  SPI_MODE_MASTER, 500000, 16);
    SPI_disableLoopback(SPIB_BASE);
    SPI_setEmulationMode(SPIB_BASE, SPI_EMULATION_FREE_RUN);

    //
    // Configuration complete. Enable the module.
    //
    SPI_enableModule(SPIB_BASE);
}

//
// Function to configure SPI A as slave with FIFO enabled.
//
void initSPIASlave(void)
{
    //
    // Must put SPI into reset before configuring it
    //
    SPI_disableModule(SPIA_BASE);

    //
    // SPI configuration. Use a 500kHz SPICLK and 16-bit word size.
    //
    SPI_setConfig(SPIA_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA0,
                  SPI_MODE_SLAVE, 500000, 16);
    SPI_disableLoopback(SPIA_BASE);
    SPI_setEmulationMode(SPIA_BASE, SPI_EMULATION_FREE_RUN);

    //
    // Configuration complete. Enable the module.
    //
    SPI_enableModule(SPIA_BASE);
}

//
// Configure GPIOs for external loopback.
//
void configGPIOs(void)
{
    //
    // This test is designed for an external loopback between SPIA
    // and SPIB.
    // External Connections:
    // -GPIO40 and GPIO8  - SPISIMO
    // -GPIO41 and GPIO10 - SPISOMI
    // -GPIO22 and GPIO9  - SPICLK
    // -GPIO23 and GPIO11 - SPISTE
    //

    //
    // GPIO08 is the SPISOMIA.
    //
    GPIO_setPinConfig(GPIO_8_SPIA_SIMO);
    GPIO_setPadConfig(8, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(8, GPIO_QUAL_ASYNC);

    //
    // GPIO10 is the SPISOMIA.
    //
    GPIO_setPinConfig(GPIO_10_SPIA_SOMI);
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(10, GPIO_QUAL_ASYNC);

    //
    // GPIO09 is the SPICLKA.
    //
    GPIO_setPinConfig(GPIO_9_SPIA_CLK);
    GPIO_setPadConfig(9, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(9, GPIO_QUAL_ASYNC);

    //
    // GPIO11 is the SPISTEA.
    //
    GPIO_setPinConfig(GPIO_11_SPIA_STE);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(11, GPIO_QUAL_ASYNC);

    //
    // GPIO40 is the SPISIMOB clock pin.
    //
    GPIO_setPinConfig(GPIO_40_SPIB_SIMO);
    GPIO_setPadConfig(40, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(40, GPIO_QUAL_ASYNC);

    //
    // GPIO41 is the SPISOMIB.
    //
    GPIO_setPinConfig(GPIO_41_SPIB_SOMI);
    GPIO_setPadConfig(41, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(41, GPIO_QUAL_ASYNC);

    //
    // GPIO22 is the SPICLKB.
    //
    GPIO_setPinConfig(GPIO_22_SPIB_CLK);
    GPIO_setPadConfig(22, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(22, GPIO_QUAL_ASYNC);

    //
    // GPIO23 is the SPISTEB.
    //
    GPIO_setPinConfig(GPIO_23_SPIB_STE);
    GPIO_setPadConfig(23, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(23, GPIO_QUAL_ASYNC);
}
