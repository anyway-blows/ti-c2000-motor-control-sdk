//#############################################################################
//
// FILE:   flashapi_cm_ex1_programming.c
//
// TITLE:  Flash programming example
//
//! \addtogroup driver_example_list
//! <h1> Flash Programming with AutoECC, DataAndECC, DataOnly and EccOnly </h1>
//!
//! This example demonstrates how to program Flash using API's following options
//! 1. AutoEcc generation
//! 2. DataOnly and EccOnly
//! 3. DataAndECC
//!
//! Before running this example, please run the cm_common_config_c28x Example
//! from the c28x folder. It will initialize the clock, configure CPU1 Flash
//! wait-states, fall back power mode, performance features and ECC.
//!
//! \b External \b Connections \n
//!  - None.
//!
//! \b Watch \b Variables \n
//!  - None.
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

//
// Include Flash API example header file
//
#include "flash_programming_f2838x_cm.h"


//
// Defines
//

//
// Length of data buffer used for program
//
#define  WORDS_IN_FLASH_BUFFER    0x100

//
// Globals
//

//
// Data Buffers used for program operation using the flash API program function
//
#pragma DATA_SECTION(Buffer,"DataBufferSection");
uint8   Buffer[WORDS_IN_FLASH_BUFFER];
uint32   *Buffer32 = (uint32 *)Buffer;


//
// Prototype of the functions used in this example
//
void Example_Error(Fapi_StatusType status);
void Example_Done(void);
void Example_CallFlashAPI(void);
void FMSTAT_Fail(void);
void ECC_Fail(void);
void Example_EraseSector(void);
void Example_ProgramUsingAutoECC(void);
void Example_ProgramUsingDataOnlyECCOnly(void);
void Example_ProgramUsingDataAndECC(void);

//
// Main
//
void main(void)
{
    //
    // Initialize device clock and peripherals
    // Copy the Flash initialization code from Flash to RAM
    // Copy the Flash API from Flash to RAM
    // Configure Flash wait-states, fall back power mode, performance features
    // and ECC
    //
    CM_init();

    //
    // At 125MHz, execution wait-states for external oscillator is 2. Modify the
    // wait-states when the CM operating frequency is changed.
    //
    Flash_initModule(FLASH0CTRL_BASE, FLASH0ECC_BASE, 2);

    //
    // Pump access must be gained by the core using pump semaphore
    //
    Flash_claimPumpSemaphore(FLASH_CM_WRAPPER);

    //
    //  Flash API functions should not be executed from the same bank on which
    //  erase/program operations are in progress.
    //  Also, note that there should not be any access to the Flash bank on
    //  which erase/program operations are in progress.  Hence below function
    //  is mapped to RAM for execution.
    //
    Example_CallFlashAPI();

    //
    // Release the pump access
    //
    Flash_releasePumpSemaphore();

    //
    // Example is done here
    //
    Example_Done();
}


//*****************************************************************************
//  Example_CallFlashAPI
//
//  This function will interface to the flash API.
//  Flash API functions used in this function are executed from RAM in this
//  example.
//*****************************************************************************
#ifdef __cplusplus
#pragma CODE_SECTION(".TI.ramfunc");
#else
#pragma CODE_SECTION(Example_CallFlashAPI, ".TI.ramfunc");
#endif
void Example_CallFlashAPI(void)
{
    uint16 i = 0;
    Fapi_StatusType  oReturnCheck;

    //
    // Initialize the Flash API by providing the Flash register base address
    // and operating frequency(in MHz).
    // This function is required to initialize the Flash API based on CM
    // operating frequency before any other Flash API operation can be performed.
    // This function must also be called whenever System frequency or RWAIT is
    // changed.
    //
    oReturnCheck = Fapi_initializeAPI(F021_CPU0_BASE_ADDRESS,
                                      CM_CLK_FREQ/1000000U);

    if(oReturnCheck != Fapi_Status_Success)
    {
        //
        // Check Flash API documentation for possible errors
        //
        Example_Error(oReturnCheck);
    }

    //
    // Initialize the Flash banks and FMC for erase and program operations.
    // Fapi_setActiveFlashBank() function sets the Flash banks and FMC for
    // further Flash operations to be performed on the banks.
    //
    oReturnCheck = Fapi_setActiveFlashBank(Fapi_FlashBank0);

    if(oReturnCheck != Fapi_Status_Success)
    {
        //
        // Check Flash API documentation for possible errors
        //
        Example_Error(oReturnCheck);
    }

    //
    // Erase the sector before programming
    //
    Example_EraseSector();

    //
    // Fill a buffer with data to program into the flash.
    //
    for(i=0; i < WORDS_IN_FLASH_BUFFER; i++)
    {
        Buffer[i] = i;
    }

    //
    // Program the sector using AutoECC option
    //
    Example_ProgramUsingAutoECC();

    //
    // Erase the sector before programming
    //
    Example_EraseSector();

    //
    // Program the sector using DataOnly and ECCOnly options
    //
    Example_ProgramUsingDataOnlyECCOnly();

    //
    // Erase the sector before programming
    //
    Example_EraseSector();

    //
    // Program the sector using DataAndECC option
    //
    Example_ProgramUsingDataAndECC();

    //
    // Erase the sector for cleaner exit from the example.
    //
    Example_EraseSector();

}

//*****************************************************************************
//  Example_ProgramUsingAutoECC
//
//  Example function to Program data in Flash using "AutoEccGeneration" option.
//  Flash API functions used in this function are executed from RAM in this
//  example.
//*****************************************************************************
#ifdef __cplusplus
#pragma CODE_SECTION(".TI.ramfunc");
#else
#pragma CODE_SECTION(Example_ProgramUsingAutoECC, ".TI.ramfunc");
#endif
void Example_ProgramUsingAutoECC(void)
{
    uint32 u32Index = 0;
    uint16 i = 0;
    Fapi_StatusType  oReturnCheck;
    Fapi_FlashStatusType  oFlashStatus;
    Fapi_FlashStatusWordType  oFlashStatusWord;

    //
    // A data buffer of max 16 bytes can be supplied to the program function.
    // Each byte is programmed until the whole buffer is programmed or a
    // problem is found. However to program a buffer that has more than 16
    // bytes, program function can be called in a loop to program 16 bytes for
    // each loop iteration until the whole buffer is programmed.
    //
    // Remember that the main array flash programming must be aligned to
    // 64-bit address boundaries and each 64 bit word may only be programmed
    // once per write/erase cycle.  Meaning the length of the data buffer
    // (3rd parameter for Fapi_issueProgrammingCommand() function) passed
    // to the program function can only be either 8 or 16.
    //
    // Program data in Flash using "AutoEccGeneration" option.
    // When AutoEccGeneration opton is used, Flash API calculates ECC for the
    // given 64-bit data and programs it along with the 64-bit main array data.
    // Note that any unprovided data with in a 64-bit data slice
    // will be assumed as 1s for calculating ECC and will be programmed.
    //
    // Note that data buffer (Buffer) is aligned on 64-bit boundary for verify
    // reasons.
    //
    // Monitor ECC address for Sector6 while programming with AutoEcc mode.
    //
    // In this example, 0x100 bytes are programmed in Flash Sector6
    // along with auto-generated ECC.
    //

    for(i=0, u32Index = Bzero_Sector6_start;
       (u32Index < (Bzero_Sector6_start + WORDS_IN_FLASH_BUFFER));
       i+= 16, u32Index+= 16)
    {
        oReturnCheck = Fapi_issueProgrammingCommand((uint32 *)u32Index, Buffer+i,
                                              16, 0, 0, Fapi_AutoEccGeneration);

        //
        // Wait until the Flash program operation is over
        //
        while(Fapi_checkFsmForReady() == Fapi_Status_FsmBusy);

        if(oReturnCheck != Fapi_Status_Success)
        {
            //
            // Check Flash API documentation for possible errors
            //
            Example_Error(oReturnCheck);
        }

        //
        // Read FMSTAT register contents to know the status of FSM after
        // program command to see if there are any program operation related
        // errors
        //
        oFlashStatus = Fapi_getFsmStatus();
        if(oFlashStatus != 0)
        {
            //
            // Check FMSTAT and debug accordingly
            //
            FMSTAT_Fail();
        }

        //
        // For illustration purpose both Fapi_doVerify and Fapi_doVerifyByByte
        // are used in this example. Use either one of them based on the need.
        //

        //
        // Verify the programmed values.  Check for any ECC errors.
        // Use this API when the length of the buffer is a multiple of 32-bit
        //
        oReturnCheck = Fapi_doVerify((uint32 *)u32Index,
                                     4, Buffer32 + (i/4),
                                     &oFlashStatusWord);

        if(oReturnCheck != Fapi_Status_Success)
        {
            //
            // Check Flash API documentation for possible errors
            //
            Example_Error(oReturnCheck);
        }

        //
        // Verify the programmed values against supplied data by byte.
        // Check for any ECC errors.
        // Use this API when the length of the buffer is a multiple of 8-bit
        //
        oReturnCheck = Fapi_doVerifyByByte((uint8 *)u32Index,
                                           16, Buffer + i,
                                           &oFlashStatusWord);

        if(oReturnCheck != Fapi_Status_Success)
        {
          //
          // Check Flash API documentation for possible errors
          //
          Example_Error(oReturnCheck);
        }
    }
}

//*****************************************************************************
//  Example_ProgramUsingDataOnlyECCOnly
//
//  Example function to Program data in Flash using "DataOnly" option and ECC
//  using "EccOnly" option.
//  Flash API functions used in this function are executed from RAM in this
//  example.
//*****************************************************************************
#ifdef __cplusplus
#pragma CODE_SECTION(".TI.ramfunc");
#else
#pragma CODE_SECTION(Example_ProgramUsingDataOnlyECCOnly, ".TI.ramfunc");
#endif
void Example_ProgramUsingDataOnlyECCOnly(void)
{
    uint32 u32Index = 0;
    uint16 i = 0, ECC_B = 0, ECC_LB = 0, ECC_HB = 0;
    uint64 *LData, *HData, dataLow, dataHigh;
    Fapi_StatusType  oReturnCheck;
    Fapi_FlashStatusType  oFlashStatus;
    Fapi_FlashStatusWordType  oFlashStatusWord;

    //
    // Program data using "DataOnly" option and ECC using "EccOnly" option.
    //
    // When DataOnly option is used, Flash API will program only the data
    // portion in Flash at the address specified.
    //
    // When EccOnly option is used, Flash API will program only the ECC portion
    // in Flash ECC memory space (Flash main array address should be provided
    // for this function and not the corresponding ECC address).
    // Fapi_calculateEcc is used to calculate the corresponding ECC of the data.
    //
    // Note that data buffer (Buffer) is aligned on 64-bit boundary for verify
    // reasons.
    //
    // In this example, 0x100 bytes are programmed in Flash Sector6
    // along with the specified ECC.
    //

    for(i=0, u32Index = Bzero_Sector6_start;
       (u32Index < (Bzero_Sector6_start + WORDS_IN_FLASH_BUFFER));
       i+= 16, u32Index+= 16)
    {
        //
        // Point LData to the lower 64 bit data
        // and   HData to the higher 64 bit data
        //
        LData = (uint64 *)(Buffer32 + i/4);
        HData = (uint64 *)(Buffer32 + i/4 + 2);

        //
        // Calculate ECC for lower 64 bit and higher 64 bit data
        //
        ECC_LB = Fapi_calculateEcc(u32Index,*LData);
        ECC_HB = Fapi_calculateEcc(u32Index+8,*HData);
        ECC_B = ((ECC_HB<<8) | ECC_LB);

        oReturnCheck = Fapi_issueProgrammingCommand((uint32 *)u32Index,Buffer+i,
                                                       16, 0, 0, Fapi_DataOnly);

        //
        // Wait until the Flash program operation is over
        //
        while(Fapi_checkFsmForReady() == Fapi_Status_FsmBusy);

        if(oReturnCheck != Fapi_Status_Success)
        {
            //
            // Check Flash API documentation for possible errors
            //
            Example_Error(oReturnCheck);
        }

        //
        // Read FMSTAT register contents to know the status of FSM after
        // program command to see if there are any program operation related
        // errors
        //
        oFlashStatus = Fapi_getFsmStatus();
        if(oFlashStatus != 0)
        {
            //Check FMSTAT and debug accordingly
            FMSTAT_Fail();
        }

        oReturnCheck = Fapi_issueProgrammingCommand((uint32 *)u32Index, 0, 0,
                                            (uint8 *)&ECC_B, 2, Fapi_EccOnly);

        //
        // Wait until the Flash program operation is over
        //
        while(Fapi_checkFsmForReady() == Fapi_Status_FsmBusy);

        if(oReturnCheck != Fapi_Status_Success)
        {
            //
            // Check Flash API documentation for possible errors
            //
            Example_Error(oReturnCheck);
        }

        //
        // Read FMSTAT register contents to know the status of FSM after
        // program command to see if there are any program operation related
        // errors
        //
        oFlashStatus = Fapi_getFsmStatus();
        if(oFlashStatus != 0)
        {
            //
            // Check FMSTAT and debug accordingly
            //
            FMSTAT_Fail();
        }

        Flash_enableECC(FLASH0ECC_BASE);

        //
        // Read back the programmed data to check if there are any ECC failures
        //
        dataLow = *(uint64 *)(u32Index);

        Flash_ErrorStatus errorStatusLow = Flash_getLowErrorStatus(FLASH0ECC_BASE);
        if((errorStatusLow != FLASH_NO_ERR) || (dataLow != *LData))
        {
            ECC_Fail();
        }

        dataHigh = *(uint64 *)(u32Index + 8);

        Flash_ErrorStatus errorStatusHigh = Flash_getHighErrorStatus(FLASH0ECC_BASE);
        if((errorStatusHigh != FLASH_NO_ERR) || (dataHigh != *HData))
        {
            ECC_Fail();
        }

        //
        // For illustration purpose both Fapi_doVerify and Fapi_doVerifyByByte
        // are used in this example. Use either one of them based on the need.
        //

        //
        // Verify the programmed values.  Check for any ECC errors.
        // Use this API when the length of the buffer is a multiple of 32-bit
        //
        oReturnCheck = Fapi_doVerify((uint32 *)u32Index,
                                     4, Buffer32 + (i/4),
                                     &oFlashStatusWord);

        if(oReturnCheck != Fapi_Status_Success)
        {
            //
            // Check Flash API documentation for possible errors
            //
            Example_Error(oReturnCheck);
        }

        //
        // Verify the programmed values against supplied data by byte.
        // Check for any ECC errors.
        // Use this API when the length of the buffer is a multiple of 8-bit
        //
        oReturnCheck = Fapi_doVerifyByByte((uint8 *)u32Index,
                                           16, Buffer + i,
                                           &oFlashStatusWord);

        if(oReturnCheck != Fapi_Status_Success)
        {
          //
          // Check Flash API documentation for possible errors
          //
          Example_Error(oReturnCheck);
        }
    }
}

//*****************************************************************************
//  Example_ProgramUsingDataAndECC
//
//  Example function to Program data in Flash using "DataAndEcc" option.
//  Flash API functions used in this function are executed from RAM in this
//  example.
//*****************************************************************************
#ifdef __cplusplus
#pragma CODE_SECTION(".TI.ramfunc");
#else
#pragma CODE_SECTION(Example_ProgramUsingDataAndECC, ".TI.ramfunc");
#endif
void Example_ProgramUsingDataAndECC(void)
{
    uint32 u32Index = 0;
    uint16 i = 0, ECC_B = 0, ECC_LB = 0, ECC_HB = 0;
    uint64 *LData, *HData, dataLow, dataHigh;
    Fapi_StatusType  oReturnCheck;
    Fapi_FlashStatusType  oFlashStatus;
    Fapi_FlashStatusWordType  oFlashStatusWord;

    //
    // Program data and ECC in Flash using "DataAndEcc" option.
    // When DataAndECC option is used, Flash API will program both the supplied
    // data and ECC in Flash at the address specified.
    // Fapi_calculateEcc is used to calculate the corresponding ECC of the data.
    //
    // Note that data buffer (Buffer) is aligned on 64-bit boundary for verify
    // reasons.
    //
    // In this example, 0x100 bytes are programmed in Flash Sector6
    // along with the specified ECC.
    //

    for(i=0, u32Index = Bzero_Sector6_start;
       (u32Index < (Bzero_Sector6_start + WORDS_IN_FLASH_BUFFER));
       i+= 16, u32Index+= 16)
    {
        //
        // Point LData to the lower 64 bit data
        // and   HData to the higher 64 bit data
        //
        LData = (uint64 *)(Buffer32 + i/4);
        HData = (uint64 *)(Buffer32 + i/4 + 2);

        //
        // Calculate ECC for lower 64 bit and higher 64 bit data
        //
        ECC_LB = Fapi_calculateEcc(u32Index,*LData);
        ECC_HB = Fapi_calculateEcc(u32Index+8,*HData);
        ECC_B = ((ECC_HB<<8) | ECC_LB);

        oReturnCheck = Fapi_issueProgrammingCommand((uint32 *)u32Index,Buffer+i,
                                       16, (uint8 *)&ECC_B, 2, Fapi_DataAndEcc);

        //
        // Wait until the Flash program operation is over
        //
        while(Fapi_checkFsmForReady() == Fapi_Status_FsmBusy);

        if(oReturnCheck != Fapi_Status_Success)
        {
            //
            // Check Flash API documentation for possible errors
            //
            Example_Error(oReturnCheck);
        }

        //
        // Read FMSTAT register contents to know the status of FSM after
        // program command to see if there are any program operation related
        // errors
        //
        oFlashStatus = Fapi_getFsmStatus();
        if(oFlashStatus != 0)
        {
            //
            // Check FMSTAT and debug accordingly
            //
            FMSTAT_Fail();
        }

        Flash_enableECC(FLASH0ECC_BASE);

        //
        // Read back the programmed data to check if there are any ECC failures
        //
        dataLow = *(uint64 *)(u32Index);

        Flash_ErrorStatus errorStatusLow = Flash_getLowErrorStatus(FLASH0ECC_BASE);
        if((errorStatusLow != FLASH_NO_ERR) || (dataLow != *LData))
        {
            ECC_Fail();
        }

        dataHigh = *(uint64 *)(u32Index + 8);

        Flash_ErrorStatus errorStatusHigh = Flash_getHighErrorStatus(FLASH0ECC_BASE);
        if((errorStatusHigh != FLASH_NO_ERR) || (dataHigh != *HData))
        {
            ECC_Fail();
        }

        //
        // For illustration purpose both Fapi_doVerify and Fapi_doVerifyByByte
        // are used in this example. Use either one of them based on the need.
        //

        //
        // Verify the programmed values.  Check for any ECC errors.
        // Use this API when the length of the buffer is a multiple of 32-bit
        //
        oReturnCheck = Fapi_doVerify((uint32 *)u32Index,
                                     4, Buffer32 + (i/4),
                                     &oFlashStatusWord);

        if(oReturnCheck != Fapi_Status_Success)
        {
            //
            // Check Flash API documentation for possible errors
            //
            Example_Error(oReturnCheck);
        }

        //
        // Verify the programmed values against supplied data by byte.
        // Check for any ECC errors.
        // Use this API when the length of the buffer is a multiple of 8-bit
        //
        oReturnCheck = Fapi_doVerifyByByte((uint8 *)u32Index,
                                           16, Buffer + i,
                                           &oFlashStatusWord);

        if(oReturnCheck != Fapi_Status_Success)
        {
          //
          // Check Flash API documentation for possible errors
          //
          Example_Error(oReturnCheck);
        }
    }

}

//*****************************************************************************
//  Example_EraseSector
//
//  Example function to Erase data of a sector in Flash.
//  Flash API functions used in this function are executed from RAM in this
//  example.
//*****************************************************************************
#ifdef __cplusplus
#pragma CODE_SECTION(".TI.ramfunc");
#else
#pragma CODE_SECTION(Example_EraseSector, ".TI.ramfunc");
#endif
void Example_EraseSector(void)
{
    Fapi_StatusType  oReturnCheck;
    Fapi_FlashStatusType  oFlashStatus;
    Fapi_FlashStatusWordType  oFlashStatusWord;

    //
    // Erase the sector that is programmed in the above example
    // Erase Sector6
    //
    oReturnCheck = Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector,
                   (uint32 *)Bzero_Sector6_start);

    //
    // Wait until FSM is done with erase sector operation
    //
    while (Fapi_checkFsmForReady() != Fapi_Status_FsmReady){}

    if(oReturnCheck != Fapi_Status_Success)
    {
        //
        // Check Flash API documentation for possible errors
        //
        Example_Error(oReturnCheck);
    }

    //
    // Read FMSTAT register contents to know the status of FSM after
    // erase command to see if there are any erase operation related errors
    //
    oFlashStatus = Fapi_getFsmStatus();
    if(oFlashStatus != 0)
    {
        //
        // Check Flash API documentation for FMSTAT and debug accordingly
        // Fapi_getFsmStatus() function gives the FMSTAT register contents.
        // Check to see if any of the EV bit, ESUSP bit, CSTAT bit or
        // VOLTSTAT bit is set (Refer to API documentation for more details).
        //
        FMSTAT_Fail();
    }

    //
    // Verify that Sector6 is erased
    //
    oReturnCheck = Fapi_doBlankCheck((uint32 *)Bzero_Sector6_start,
                   Sector64KB_u32length,
                   &oFlashStatusWord);

    if(oReturnCheck != Fapi_Status_Success)
    {
        //
        // Check Flash API documentation for error info
        //
        Example_Error(oReturnCheck);
    }
}


//******************************************************************************
// For this example, just stop here if an API error is found
//******************************************************************************
void Example_Error(Fapi_StatusType status)
{
    //
    //  Error code will be in the status parameter
    //
    __asm("   bkpt #0");
}

//******************************************************************************
//  For this example, once we are done just stop here
//******************************************************************************
void Example_Done(void)
{
    __asm("   bkpt #0");
}

//******************************************************************************
// For this example, just stop here if FMSTAT fail occurs
//******************************************************************************
void FMSTAT_Fail(void)
{
    __asm("   bkpt #0");
}

//******************************************************************************
// For this example, just stop here if ECC fail occurs
//******************************************************************************
void ECC_Fail(void)
{
    __asm("   bkpt #0");
}

//
// End of File
//
