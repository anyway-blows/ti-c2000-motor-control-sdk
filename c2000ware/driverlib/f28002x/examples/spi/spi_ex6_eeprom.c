//#############################################################################
//
// FILE:   spi_ex6_eeprom.c
//
// TITLE:  SPI EEPROM
//
//! \addtogroup driver_example_list
//! <h1>SPI EEPROM</h1>
//!
//! This program will write 8 bytes to EEPROM and read them back. The device
//! communicates with the EEPROM via SPI and specific opcodes. This example is
//! written to work with the SPI Serial EEPROM AT25128/256.
//!
//! \b External \b Connections \n
//!  - Connect external SPI EEPROM
//!  - Connect GPIO08 (SIMO) to external EEPROM SI pin
//!  - Connect GPIO10 (SOMI) to external EEPROM SO pin
//!  - Connect GPIO09 (CLK) to external EEPROM SCK pin
//!  - Connect GPIO11 (CS) to external EEPROM CS pin
//!  - Connect the external EEPROM VCC and GND pins
//!
//! \b Watch \b Variables \n
//!  - writeBuffer - Data that is written to external EEPROM
//!  - readBuffer  - Data that is read back from EEPROM
//!  - error       - Error count
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
//#############################################################################

//
// Included Files
//
#include "driverlib.h"
#include "device.h"

//
// Defines
//
#define EEPROM_ADDR                 0x0000
#define NUM_BYTES                   8

//
// SPI EEPROM status
//
#define MSG_STATUS_READY_M          0x0001 // EEPROM is ready (not busy)
#define MSG_STATUS_WRITE_READY_M    0x0002 // EEPROM
#define MSG_STATUS_BUSY             0xFFFF // EEPROM is busy (internal write)

//
// Opcodes for the EEPROM (8-bit)
//
#define RDSR                        0x0500
#define READ                        0x0300
#define WRITE                       0x0200
#define WREN                        0x0600
#define WRDI                        0x0400
#define WRSR                        0x0100

//
// Defines for Chip Select toggle.
//
#define CS_LOW                      GPIO_writePin(11, 0)
#define CS_HIGH                     GPIO_writePin(11, 1)

//
// Globals
//
uint16_t writeBuffer[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
uint16_t readBuffer[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

//
// Function Prototypes
//
void initSPI(void);
uint16_t readStatusRegister(void);
void writeData(uint16_t address, uint16_t * data, uint16_t length);
void readData(uint16_t address, uint16_t * data, uint16_t length);
void enableWrite(void);

//
// Main
//
void main(void)
{
    uint16_t error = 0;
    uint16_t i;

    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Disable pin locks and enable internal pullups.
    //
    Device_initGPIO();

    //
    // Initialize the SPI
    //
    initSPI();

    //
    // Wait until the EEPROM is ready to write data
    //
    while(readStatusRegister() & MSG_STATUS_READY_M == MSG_STATUS_READY_M)
    {
    }

    //
    // Enable wirte on the EEPROM
    //
    enableWrite();

    //
    // Wait until the EEPROM is ready to write data
    //
    while(readStatusRegister() & MSG_STATUS_WRITE_READY_M == MSG_STATUS_WRITE_READY_M)
    {
    }

    //
    // Write to the EEPROM
    //
    writeData(EEPROM_ADDR, writeBuffer, NUM_BYTES);

    //
    // Wait until the EEPROM is ready to write data
    //
    while(readStatusRegister() & MSG_STATUS_READY_M == MSG_STATUS_READY_M)
    {
    }

    //
    // Read from the EEPROM
    //
    readData(EEPROM_ADDR, readBuffer, NUM_BYTES);

    //
    // Check received data for correctness
    //
    for(i = 0; i < NUM_BYTES; i++)
    {
        if(writeBuffer[i] != readBuffer[i])
        {
            error++;
        }
    }

    //
    // Loop forever.
    //
    while(1);
}

//
// Function to configure SPI A in FIFO mode.
//
void initSPI()
{
    //
    // Configure SPI pins :
    //  GPIO08 - SPISIMO
    //  GPIO10 - SPISOMI
    //  GPIO09 - SPICLK
    //  GPIO11 - SPICS
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
    // GPIO11 is the SPICS.
    //
    GPIO_setPinConfig(GPIO_11_GPIO11);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(11, GPIO_QUAL_ASYNC);
    GPIO_setDirectionMode(11, GPIO_DIR_MODE_OUT);

    //
    // Must put SPI into reset before configuring it.
    //
    SPI_disableModule(SPIA_BASE);

    //
    // SPI configuration. Use a 2MHz SPICLK and 8-bit word size.
    //
    SPI_setConfig(SPIA_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA1,
                  SPI_MODE_MASTER, 1000000, 8);

    //
    // Configuration complete. Enable the module.
    //
    SPI_enableModule(SPIA_BASE);
}

//
// Function to send RDSR opcode and return the status of the EEPROM
//
uint16_t readStatusRegister(void)
{
    uint16_t temp;

    //
    // Pull chip select low.
    //
    CS_LOW;

    //
    // Send RDSR opcode
    //
    SPI_writeDataBlockingNonFIFO(SPIA_BASE, RDSR);

    //
    // Dummy read to clear INT_FLAG.
    //
    temp = SPI_readDataBlockingNonFIFO(SPIA_BASE);

    //
    // Send dummy data to receive the status.
    //
    SPI_writeDataBlockingNonFIFO(SPIA_BASE, 0x0000);

    //
    // Read status register.
    //
    temp = SPI_readDataBlockingNonFIFO(SPIA_BASE);

    //
    // Pull chip select high.
    //
    CS_HIGH;

    //
    // Read the status from the receive buffer
    //
    return(temp);
}

//
// Function to send the WREN opcode
//
void enableWrite(void)
{
    //
    // Pull chip select low.
    //
    CS_LOW;

    //
    // Send the WREN opcode.
    //
    SPI_writeDataBlockingNonFIFO(SPIA_BASE, WREN);

    //
    // Dummy read to clear INT_FLAG.
    //
    SPI_readDataBlockingNonFIFO(SPIA_BASE);

    //
    // Pull chip select high.
    //
    CS_HIGH;
}

//
// Function to write data to the EEPROM
// - address is the byte address of the EEPROM
// - data is a pointer to an array of data being sent
// - length is the number of characters in the array to send
//
void writeData(uint16_t address, uint16_t * data, uint16_t length)
{
    uint16_t i;

    //
    // Pull chip select low.
    //
    CS_LOW;

    //
    // Send the WRITE opcode.
    //
    SPI_writeDataBlockingNonFIFO(SPIA_BASE, WRITE);

    //
    // Dummy read to clear INT_FLAG.
    //
    SPI_readDataBlockingNonFIFO(SPIA_BASE);

    //
    // Send the MSB of the address of the EEPROM.
    //
    SPI_writeDataBlockingNonFIFO(SPIA_BASE, (address & 0xFF00));

    //
    // Dummy read to clear INT_FLAG.
    //
    SPI_readDataBlockingNonFIFO(SPIA_BASE);

    //
    // Send the LSB of the address to the EEPROM.
    //
    SPI_writeDataBlockingNonFIFO(SPIA_BASE, address << 8);

    //
    // Dummy read to clear INT_FLAG.
    //
    SPI_readDataBlockingNonFIFO(SPIA_BASE);

    //
    // Send the data.
    //
    for(i = 0; i < length; i++)
    {
        //
        // Send the data.
        //
        SPI_writeDataBlockingNonFIFO(SPIA_BASE, data[i] << 8);

        //
        // Dummy read to clear INT_FLAG.
        //
        SPI_readDataBlockingNonFIFO(SPIA_BASE);
    }

    //
    // Pull chip select high.
    //
    CS_HIGH;
}

//
// Function to read data from the EEPROM
// - address is the byte address of the EEPROM
// - data is a pointer to an array of data being received
// - length is the number of characters in the array to receive
//
void readData(uint16_t address, uint16_t * data, uint16_t length)
{
    uint16_t i;

    CS_LOW;

    //
    // Send the READ opcode
    //
    SPI_writeDataBlockingNonFIFO(SPIA_BASE, READ);

    //
    // Dummy read to clear INT_FLAG.
    //
    SPI_readDataBlockingNonFIFO(SPIA_BASE);

    //
    // Send the MSB of the address of the EEPROM
    //
    SPI_writeDataBlockingNonFIFO(SPIA_BASE, (address & 0xFF00));

    //
    // Dummy read to clear INT_FLAG.
    //
    SPI_readDataBlockingNonFIFO(SPIA_BASE);

    //
    // Send the LSB of the address of the EEPROM
    //
    SPI_writeDataBlockingNonFIFO(SPIA_BASE, (address << 8));

    //
    // Dummy read to clear INT_FLAG.
    //
    SPI_readDataBlockingNonFIFO(SPIA_BASE);

    //
    // Read the data from the EEPROM
    //
    for(i = 0; i < length; i++)
    {
        //
        // Send dummy data to receive the EEPROM data
        //
        SPI_writeDataBlockingNonFIFO(SPIA_BASE, 0x0000);
        data[i] = SPI_readDataBlockingNonFIFO(SPIA_BASE);
    }

    CS_HIGH;
}

//
// End of File
//

