//#############################################################################
//
// FILE:   boot_ex1_cpu1_cpu2_cm_secure_flash_cpu1.c
//
// TITLE:  Secure Flash Boot Example
//
//! \addtogroup driver_example_list
//! <h1> CPU1 Secure Flash Boot </h1>
//!
//! This example demonstrates how to use the secure flash boot mode for CPU1
//! as well as release CPU2 and CM for secure flash boot.
//!
//! Secure flash boot performs a CMAC authentication on the entry sector of
//! flash upon device boot up. If authentication passes, the application will
//! begin execution. Learn more on the secure flash boot mode in the device 
//! technical reference manual.
//!
//! This project shows how to use the C2000 HEX Utility to generate a CMAC Tag
//! based on a user CMAC key and embed the value into the flash application.
//! Additionally, the example details the method to call the CMAC API from
//! the user application to calculate CMAC on other flash sectors beyond the 
//! the application entry flash sector.
//!
//! How to Run:
//! - Load application into CPU1 flash (as well as CPU2 and CM applications)
//! - Disconnect and reconnect to only CPU1
//! - In memory window, set address 0xD00/D01 to 0x5AFFFFFF and address 0xD04 to
//!   0x000A (This sets emulation boot to secure flash boot)
//! - Reset CPU1 via CCS and click resume
//! - Observe the LEDs
//!
//! Determining Pass/Fail without debugger connected:
//! \b CPU1 - ControlCARD LED1. 
//! - LED off = Secure Boot failed
//! - LED On (Solid) = Secure Boot Passed, Full Flash CMAC failed
//! - LED Blinking = Secure Boot Passed and Full Flash CMAC passed
//! \b CPU2 - ControlCARD LED2. 
//! - LED off = Secure Boot failed
//! - LED On (Solid) = Secure Boot Passed, Full Flash CMAC failed
//! - LED Blinking = Secure Boot Passed and Full Flash CMAC passed
//! \b CM - ControlCARD LED3. 
//! - LED off = Secure Boot failed
//! - LED On (Solid) = Secure Boot Passed, Full Flash CMAC failed
//! - LED Blinking = Secure Boot Passed and Full Flash CMAC passed
//!
//! \b External \b Connections \n
//!  - None.
//!
//! \b Watch \b Variables \n
//!  - cpu1_SuccessfullyBooted - True when CPU1 full flash CMAC authentication 
//!                              passes. Otherwise, false.
//!  - cpu2_SuccessfullyBooted - True when CPU2 full flash CMAC authentication 
//!                              passes and CPU1 receives IPC. Otherwise, false.
//!  - cm_SuccessfullyBooted - True when CM full flash CMAC authentication 
//!                              passes and CPU1 receives IPC. Otherwise, false.
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
#include "driverlib.h"
#include "device.h"

//
// Defines
//
#define CMAC_AUTH_PASS            0UL
#define CMAC_AUTH_START_ADDRESS   0x00080000UL
#define CMAC_AUTH_END_ADDRESS     0x000C0000UL
#define CMAC_AUTH_TAG_ADDRESS     0x00087002UL

//
// Globals
//
bool cpu1_SuccessfullyBooted = false;
bool cpu2_SuccessfullyBooted = false;
bool cm_SuccessfullyBooted = false;

//
// Structure required for CMAC symbol reserving in memory.
// Required for performing CMAC on non-entry sector of flash.
//
// Read more in the Assembly Language Tools User's Guide
//
struct CMAC_TAG
{
    char tag[8];
    uint32_t start;
    uint32_t end;
};

//
// Create CMAC symbol for flash entry point 1 (0x80000).
//
// Reserves memory for storing the golden CMAC tag that gets
// generated by the HEX Utility.
//
#pragma RETAIN(cmac_sb_1)
#pragma LOCATION(cmac_sb_1, 0x080002)
const char cmac_sb_1[8] = { 0 };

//
// Create CMAC symbol for full length flash authentication.
// The start and end CMAC_TAG struct members are zero, therefore
// the CMAC algorithm runs over entire memory region specified in 
// the HEX directive. 
//
// Note that the location 0x087002 can be set to anywhere as long 
// as it is within the specified CMAC authentication memory.
//
// Reserves memory for storing the golden CMAC tag that gets
// generated by the HEX Utility.
//
#pragma RETAIN(cmac_all)
#pragma LOCATION(cmac_all, 0x087002)
const struct CMAC_TAG cmac_all = {{ 0 }, 0x0, 0x0};

//
// Function Prototypes
//
uint32_t CPU1BROM_calculateCMAC(uint32_t startAddress, 
                                uint32_t endAddress, 
                                uint32_t tagAddress);
__interrupt void appIPCHandler(void);

//
// Main
//
void main(void)
{
    uint32_t applicationCMACStatus = CMAC_AUTH_PASS;
    
    //
    // Initialize device clock and peripherals
    //
    Device_init();
    
    //
    // Initialize GPIO and configure the GPIO pins as a push-pull output
    //
    Device_initGPIO();
    GPIO_setPadConfig(DEVICE_GPIO_PIN_LED1, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_LED1, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_LED2, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_LED2, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_LED3, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_LED3, GPIO_DIR_MODE_OUT);    
    
    //
    // Turn LEDS off and Allocate LED GPIOs to CPU2 and CM
    //
    GPIO_writePin(DEVICE_GPIO_PIN_LED2, 1);
    GPIO_writePin(DEVICE_GPIO_PIN_LED3, 0);
    GPIO_setMasterCore(DEVICE_GPIO_PIN_LED2, GPIO_CORE_CPU2);
    GPIO_setMasterCore(DEVICE_GPIO_PIN_LED3, GPIO_CORE_CM);
    
    //
    // Turn on LED1 to indicate CPU1 has entered application
    //
    GPIO_writePin(DEVICE_GPIO_PIN_LED1, 0);
    
    //
    // Call CMAC Authentication API to verify contents of all of
    // device flash.
    //
    // Upon failure, enter infinite loop.
    //
    applicationCMACStatus = CPU1BROM_calculateCMAC(CMAC_AUTH_START_ADDRESS, 
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
        cpu1_SuccessfullyBooted = true;
    }
    
    //
    // Secure Boot CPU2 core
    //
    Device_bootCPU2(BOOTMODE_BOOT_TO_SECURE_FLASH_SECTOR0);
    
    //
    // Secure Boot CM core
    //
    Device_bootCM(BOOTMODE_BOOT_TO_SECURE_FLASH_SECTOR0);

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
    // Configure and Enable IPC interrupts
    //
    IPC_registerInterrupt(IPC_CPU1_L_CPU2_R, IPC_INT0, appIPCHandler);
    IPC_registerInterrupt(IPC_CPU1_L_CM_R, IPC_INT1, appIPCHandler);

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

    //
    // Loop Forever - Blink LED to indicate full flash CMAC has passed
    //
    for(;;)
    {
        //
        // Turn on LED
        //
        GPIO_writePin(DEVICE_GPIO_PIN_LED1, 0);

        //
        // Delay for a bit.
        //
        DEVICE_DELAY_US(500000);

        //
        // Turn off LED
        //
        GPIO_writePin(DEVICE_GPIO_PIN_LED1, 1);

        //
        // Delay for a bit.
        //
        DEVICE_DELAY_US(500000);
    }
}

//
// appIPCHandler - Handle notifications from CPU2 and CM when
//                 their applications complete successfully
//
__interrupt void appIPCHandler(void)
{
    if(IPC_isFlagBusyRtoL(IPC_CPU1_L_CPU2_R, IPC_FLAG0))
    {
        cpu2_SuccessfullyBooted = true;
        IPC_ackFlagRtoL(IPC_CPU1_L_CPU2_R, IPC_FLAG0);
    }
    else if(IPC_isFlagBusyRtoL(IPC_CPU1_L_CM_R, IPC_FLAG1))
    {
        cm_SuccessfullyBooted = true;
        IPC_ackFlagRtoL(IPC_CPU1_L_CM_R, IPC_FLAG1);
    }
    else
    {
        // No action
    }
    
    //
    // Acknowledge this interrupt located in PIE group 1
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);    
}

//
// End of File
//
