//-----------------------------------------------------------------------------
//  FILE:           PM_tformat-Main.C
//
//  Description:    Example project for using PM tformat Library.
//                  Includes PM_tformat_lib library and corresponding
//                  include files.
//                  Initializes the encoders and performs delay compensation.
//                  Runs tformat command set.
//                  Continuously Read position value in an infinite loop
//
//  Version:        1.0
//
//  Target:         TMS320F28379D
//
//-----------------------------------------------------------------------------
//  Copyright Texas Instruments © 2004-2015
//-----------------------------------------------------------------------------
//  Revision History:
//-----------------------------------------------------------------------------
//  Date      | Description / Status
//-----------------------------------------------------------------------------
// Feb 2019  - Example project for PM T-Format Library Usage
//-----------------------------------------------------------------------------

#include "tformat.h"         // Include file for tformat interface
#include "device.h"
#include "driverlib.h"

uint32_t crcResult;
uint16_t retVal1;       //used for function return val storage and checks
uint32_t position, turns;

void main(void) {

    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Turn on the module clock.
    //
    Interrupt_disableMaster();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Disable pin locks and enable internal pull-ups.
    //
    Device_initGPIO();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    Interrupt_register(INT_SPIB_RX, &spiRxFIFOISR);

    //
    //Initialization routine for tformat operation - defined in tformat.c
    //Configures the peripherals and enables clocks for required modules
    //Configures GPIO and XBar as needed for t-format operation
    //Sets up the SPI peripheral in tformat data structure and enables interrupt
    //
    Interrupt_enable(INT_SPIB_RX);
    Interrupt_enableMaster();

    tformat_init();

    DEVICE_DELAY_US(800L);

    //
    //Optional function exercising several tformat
    //commands including eeprom accesses
    //
    retVal1 = tformat_exCommands();

//
//Infinite loop to read position using PM_TFORMAT_DATAID3 command
//- returns position and turns information
//
    while(1)
    {
        retVal1 = PM_tformat_setupCommand (PM_TFORMAT_DATAID3, 0, 0, 0);
        PM_tformat_startOperation();
        while(tformatData.dataReady != 1);// {}
        retVal1 = PM_tformat_receiveData(PM_TFORMAT_DATAID3);

        crcResult = PM_tformat_getCRC(0, 80, 8, (uint16_t *)&tformatData.rxPkts,
                    tformatCRCtable, 10);
        crcResult = crcResult ^ (0xFF);
        if(!tformat_checkCRC(crcResult, tformatData.crc))
        {
             ESTOP0;
        }

        //
        //Invert the received bit sequence for position and
        //turns for actual data
        //
        position =
            ((__flip32((uint32_t) tformatData.dataField0) >> 24 ) & 0xFF) |
            ((__flip32((uint32_t) tformatData.dataField1) >> 16 ) & 0xFF00) |
            ((__flip32((uint32_t) tformatData.dataField2) >> 8 )  & 0xFF0000);

        turns =
            ((__flip32((uint32_t) tformatData.dataField4) >> 24 ) & 0xFF) |
            ((__flip32((uint32_t) tformatData.dataField5) >> 16 ) & 0xFF00) |
            ((__flip32((uint32_t) tformatData.dataField6) >> 8 )  & 0xFF0000);

        DEVICE_DELAY_US(400L);
    }
}

//
// End of file
//
