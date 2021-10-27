//#############################################################################
//
// FILE:   boot_ex1_cpu1_cpu2_cm_secure_flash_cm.c
//
// TITLE:  Secure Flash Boot Example
//
// <h1> CM Secure Flash Boot </h1>
//
// This example demonstrates how to use the secure flash boot mode for CM.
// (Requires CPU1 example application)
//
// Secure flash boot performs a CMAC authentication on the entry sector of
// flash upon device boot up. If authentication passes, the application will
// begin execution. Learn more on the secure flash boot mode in the device 
// technical reference manual.
//
// This project shows how to use the C2000 HEX Utility to generate a CMAC Tag
// based on a user CMAC key and embed the value into the flash application.
// Additionally, the example details the method to call the CMAC API from
// the user application to calculate CMAC on other flash sectors beyond the 
// the application entry flash sector.
//
// Determining Pass/Fail without debugger connected:
// CM - ControlCARD LED3. 
// - LED off = Secure Boot failed
// - LED On (Solid) = Secure Boot Passed, Full Flash CMAC failed
// - LED Blinking = Secure Boot Passed and Full Flash CMAC passed
//
// \b External \b Connections \n
//  - None.
//
// \b Watch \b Variables \n
//  - None.
//
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
// Defines
//
#define DEVICE_GPIO_PIN_LED3      145U
#define CMAC_AUTH_PASS            0UL
#define CMAC_AUTH_START_ADDRESS   0x00200000UL
#define CMAC_AUTH_END_ADDRESS     0x00280000UL
#define CMAC_AUTH_TAG_ADDRESS     0x00204004UL

//
// Structure required for CMAC symbol reserving in memory.
// Required for performing CMAC on non-entry sector of flash.
//
// Read more in the Assembly Language Tools User's Guide
//
struct CMAC_TAG
{
    uint8_t tag[16];
    uint32_t start;
    uint32_t end;
};

//
// Create CMAC symbol for flash entry point 1 (0x00200000).
//
// Reserves memory for storing the golden CMAC tag that gets
// generated by the HEX Utility.
//
#pragma RETAIN(cmac_sb_1)
#pragma LOCATION(cmac_sb_1, 0x00200004)
const uint8_t cmac_sb_1[16] = { 0 };

//
// Create CMAC symbol for full length flash authentication.
// The start and end CMAC_TAG struct members are zero, therefore
// the CMAC algorithm runs over entire memory region specified in 
// the HEX directive. 
//
// Note that the location 0x00204004 can be set to anywhere as long 
// as it is within the specified CMAC authentication memory.
//
// Reserves memory for storing the golden CMAC tag that gets
// generated by the HEX Utility.
//
#pragma RETAIN(cmac_all)
#pragma LOCATION(cmac_all, 0x00204004)
const struct CMAC_TAG cmac_all = {{ 0 }, 0x0, 0x0};

//
// Function Prototypes
//
uint32_t CMBROM_calculateCMAC(uint32_t startAddress, 
                              uint32_t endAddress, 
                              uint32_t tagAddress);
                                
//
// Main
//
void main(void)
{
    uint32_t applicationCMACStatus = CMAC_AUTH_PASS;     
    
    //
    // Initialize device clock and peripherals
    //
    CM_init();

    //
    // Initialize GPIO and configure the GPIO pin as a push-pull output
    //
    // This is configured by CPU1

    //
    // Turn on LED3 to indicate CM has entered application
    //
    GPIO_writePin(DEVICE_GPIO_PIN_LED3, 1);

    //
    // Call CMAC Authentication API to verify contents of all of
    // device flash.
    //
    // Upon failure, enter infinite loop.
    //
    applicationCMACStatus = CMBROM_calculateCMAC(CMAC_AUTH_START_ADDRESS, 
	                                             CMAC_AUTH_END_ADDRESS, 
                                                 CMAC_AUTH_TAG_ADDRESS);
    
    if(CMAC_AUTH_PASS != applicationCMACStatus)
    {
        //
        // Application will get stuck here upon failure
        //        
        while(1);
    }
    else
    {
        IPC_setFlagLtoR(IPC_CM_L_CPU1_R, IPC_FLAG1);
    }

    //
    // Loop Forever
    //
    for(;;)
    {
        //
        // Turn on LED
        //
        GPIO_writePin(DEVICE_GPIO_PIN_LED3, 1);

        //
        // Delay for 500000uS.
        //
        DEVICE_DELAY_US(500000);

        //
        // Turn off LED
        //
        GPIO_writePin(DEVICE_GPIO_PIN_LED3, 0);

        //
        // Delay for 500000uS.
        //
        DEVICE_DELAY_US(500000);
    }
}

//
// End of File
//
