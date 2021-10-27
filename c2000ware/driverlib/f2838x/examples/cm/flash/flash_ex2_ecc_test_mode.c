//#############################################################################
//
// FILE:   flash_ex2_ecc_test_mode.c
//
// TITLE:  Flash ECC Test Mode Example
//
//! \addtogroup driver_example_cm_list
//! <h1> Flash ECC Test Mode </h1>
//!
//! This example demonstrates ECC Test mode.
//| ECC Test mode is provided to check the correctness of SECDED logic.
//! Four types of errors are inserted to demonstrate the usage of ECC Test Mode
//! 1. Single bit error detection
//! 2. Double bit error detection
//! 3. Single bit ECC error detection
//! 4. Double bit ECC error detection
//!
//! Before running this example, please run the cm_common_config_c28x Example
//! from the c28x folder. It will initialize the clock, configure CPU1 Flash
//! wait-states, fall back power mode, performance features and ECC.
//!
//!
//! \b External \b Connections \n
//!  - None
//!
//! \b Watch \b Variables \n
//! - None
//!
//
//#############################################################################
// $TI Release: F2838x Support Library v3.04.00.00 $
// $Release Date: Fri Feb 12 19:08:49 IST 2021 $
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
#include "driverlib_cm.h"
#include "cm.h"

//
// Include Flash API include file
//
#include "F021_F2838x_CM.h"

uint32 flash_address = 0x210000;
uint64 flash_data = 0xAAAABBBBCCCCDDDD;
uint8 flash_ecc = 0;
uint8 error = 0;

#define DATA_BIT_ERROR_POSITION 7
#define ECC_BIT_ERROR_POSITION  5

//
// Inserts a single bit error in the data before providing the data to ECC Test
// Block
//
void singleBitDataErrorExample();

//
// Inserts a double bit error in the data before providing the data to ECC Test
// Block
//
void doubleBitDataErrorExample();

//
// Inserts a single bit error in the ECC of the data before providing the ecc to
// ECC Test Block
//
void singleBitECCErrorExample();

//
// Inserts a double bit error in the ECC of the data before providing the ecc to
// ECC Test Block
//
void doubleBitECCErrorExample();

//
// Sets ECC Test Block parameters
// Enables the ECC Test mode, uses Lower ECC Test block and performs ECC
// calculation
//
void setupECCTestBlock(uint64 data, uint32 address, uint16 ecc);

#define SINGLE_BIT_DATA_ERROR_EXAMPLE   0
#define DOUBLE_BIT_DATA_ERROR_EXAMPLE   1
#define SINGLE_BIT_ECC_ERROR_EXAMPLE    2
#define DOUBLE_BIT_ECC_ERROR_EXAMPLE    3

//
// Modify the below value to run a different example
//
uint8 current_example = SINGLE_BIT_DATA_ERROR_EXAMPLE;
//
// Main
//
void main(void)
{
    //
    // Initializes device clock and peripherals
    //
    CM_init();

    //
    // Call Flash Initialization to setup flash wait states
    //
    Flash_initModule(FLASH0CTRL_BASE, FLASH0ECC_BASE, 2);

    //
    // Initialize the Flash API by providing the Flash register base address
    // and operating frequency.
    //
    Fapi_initializeAPI(F021_CPU0_BASE_ADDRESS, 125);

    //
    // Get the corresponding ECC of the data
    //
    flash_ecc = Fapi_calculateEcc(flash_address, flash_data);

    switch(current_example) {

    case SINGLE_BIT_DATA_ERROR_EXAMPLE:
    {
        //#####################################################################
        // Single bit data error insertion
        //#####################################################################

        singleBitDataErrorExample();

        //
        // Get the error type
        //
        if(FLASH_SINGLE_ERROR != Flash_getECCTestStatus(FLASH0ECC_BASE))
            error++;

        //
        // Check if the detected error is data error
        //
        if(FLASH_DATA_BITS != Flash_getECCTestSingleBitErrorType(
                                                  FLASH0ECC_BASE))
            error++;

        //
        // Check if the detected error position matches with the error position
        // set
        //
        if(DATA_BIT_ERROR_POSITION != Flash_getECCTestErrorPosition(
                                                     FLASH0ECC_BASE))
            error++;

        //
        // Check if the SECDED logic corrected single bit error
        //
        if((uint32)flash_data != Flash_getTestDataOutLow(FLASH0ECC_BASE))
            error++;
    }
    break;

    case DOUBLE_BIT_DATA_ERROR_EXAMPLE:
    {
        //#####################################################################
        // Double bit data error insertion
        //#####################################################################

        doubleBitDataErrorExample();

        //
        // Get the error type
        //
        if(FLASH_UNC_ERROR != Flash_getECCTestStatus(FLASH0ECC_BASE))
            error++;
    }
    break;

    case SINGLE_BIT_ECC_ERROR_EXAMPLE:
    {
        //#####################################################################
        // Single bit ECC error insertion
        //#####################################################################

        singleBitECCErrorExample();

        //
        // Get the error type
        //
        if(FLASH_SINGLE_ERROR != Flash_getECCTestStatus(FLASH0ECC_BASE))
            error++;

        //
        // Check if the detected error is ecc error
        //
        if(FLASH_CHECK_BITS != Flash_getECCTestSingleBitErrorType(
                                                   FLASH0ECC_BASE))
            error++;

        //
        // Check if the detected error position matches with the error position
        // set
        //
        if(ECC_BIT_ERROR_POSITION != Flash_getECCTestErrorPosition(
                                                    FLASH0ECC_BASE))
            error++;
    }
    break;

    case DOUBLE_BIT_ECC_ERROR_EXAMPLE:
    {
        //#####################################################################
        // Double bit ECC error insertion
        //#####################################################################

        doubleBitECCErrorExample();

        //
        // Get the error type
        //
        if(FLASH_UNC_ERROR != Flash_getECCTestStatus(FLASH0ECC_BASE))
            error++;
    }
    break;

    default:
    {
        error++;
    }

    }

    //
    // Revert back the ECC Test mode setting
    //
    Flash_disableECCTestMode(FLASH0ECC_BASE);
}

void singleBitDataErrorExample()
{
    //
    // Insert single bit error in data
    //
    setupECCTestBlock(flash_data ^ (1 << DATA_BIT_ERROR_POSITION),
                     flash_address, flash_ecc);

}

void doubleBitDataErrorExample()
{
    //
    // Insert double bit error in data
    //
    setupECCTestBlock(flash_data ^ 0x3, flash_address, flash_ecc);
}


void singleBitECCErrorExample()
{
    //
    // Insert single bit error in ECC
    //
    setupECCTestBlock(flash_data, flash_address,
                                  flash_ecc ^ (1 << ECC_BIT_ERROR_POSITION));
}

void doubleBitECCErrorExample()
{
    //
    // Insert double bit error in ECC
    //
    setupECCTestBlock(flash_data, flash_address, flash_ecc ^ 0x3);
}

void setupECCTestBlock(uint64 data, uint32 address, uint16 ecc)
{
    //
    // Write 128-bit flash address in FADDR_TEST
    //
    Flash_setECCTestAddress(FLASH0ECC_BASE, address);

    //
    // Write lower 32 bits of data in FDATAL_TEST
    //
    Flash_setDataLowECCTest(FLASH0ECC_BASE, (uint32)data);

    //
    // Write upper 32 bits of data in FDATAH_TEST
    //
    Flash_setDataHighECCTest(FLASH0ECC_BASE, (uint32)(data >> 32));

    //
    // Write corresponding ECC in the FECC_TEST
    // Insert double bit error in flash ecc
    //
    Flash_setECCTestECCBits(FLASH0ECC_BASE, ecc);

    //
    // Select lower ECC block
    // Only one of the SECDED modules (out of the two SECDED modules that work
    // on lower 64 bits and upper 64 bits of a read 128-bit data) at a time
    // can be tested.
    //
    Flash_selectLowECCBlock(FLASH0ECC_BASE);

    //
    // Enable the ECC Test Mode
    //
    Flash_enableECCTestMode(FLASH0ECC_BASE);

    //
    // Perform ECC calculation
    //
    Flash_performECCCalculation(FLASH0ECC_BASE);
}

//
// End of File
//
