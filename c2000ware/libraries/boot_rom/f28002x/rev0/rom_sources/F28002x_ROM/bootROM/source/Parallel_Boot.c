//###########################################################################
//
// FILE:    bootloader_parallel.c
//
// TITLE:   Parallel Port I/O bootloader
//
// Functions involved in running Parallel I/O bootloader
//
// ------------------------------------------------------------------
// |Opt No.|  BOOTDEF      |  Dx GPIO       |  DSP Ctrl | Host Ctrl |
// ------------------------------------------------------------------
// |  0    |  0x00         | 28, 1-7        |  16       |  29       |
// |  1    |  0x20         | 0-7            |  16       |  11       |
// ------------------------------------------------------------------
//
//###########################################################################
// $TI Release: $
// $Release Date: $
//###########################################################################

//
// Included Files
//
#include "cpu1bootrom.h"

static uint32_t Parallel_bootMode;
                              
//
// Function Prototypes
//
uint16_t Parallel_GetWordData_8bit(void);
void Parallel_GPIOSelect(uint32_t  bootMode);

//
// Parallel_Boot - This module is the main Parallel boot routine. It will load
//                 code via GP I/Os. This boot mode accepts 8-bit data. 8-bit
//                 data is expected to be the order LSB followed by MSB.
//
//                 This function returns a entry point address back
//                 to the system initialization routine which in turn calls
//                 the ExitBoot routine.
//
uint32_t Parallel_Boot(uint32_t  BootMode)
{
    uint32_t entryAddress;
    uint16_t wordData;
    
    Parallel_bootMode = BootMode;

    //
    // Setup for Parallel boot
    //
    Parallel_GPIOSelect(BootMode);

    //
    // CPU1 Patch/Escape Point 12
    //
    entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_12;
    if((entryAddress != 0xFFFFFFFFUL) &&
       (entryAddress != 0x00000000UL))
    {
        //
        // If OTP is programmed, then call OTP patch function
        //
        ((void (*)(void))entryAddress)();
    }

    //
    // Check for the key value. This version only
    // supports 8-bit data.
    //
    GetWordData = Parallel_GetWordData_8bit;
    wordData = GetWordData();

    if(wordData != BROM_EIGHT_BIT_HEADER)
    {
       return FLASH_ENTRY_POINT;
    }

    //
    // Read and discard the reserved words
    //
    ReadReservedFn();

    //
    // Get the entry point address
    //
    entryAddress = GetLongData();

    //
    // Load the data
    //
    CopyData();

    return entryAddress;
}

//
// Parallel_GetWordData_8bit - The 8-bit function is used if the input stream is
//                             an 8-bit input stream and the upper 8-bits of the
//                             GPIO port are ignored.  In the 8-bit case the
//                             first fetches the LSB and then the MSB from the
//                             GPIO port. These two bytes are then put together
//                             to form a single 16-bit word that is then passed
//                             back to the host. Note that in this case, the
//                             input stream from the host is in the order LSB
//                             followed by MSB
//
uint16_t Parallel_GetWordData_8bit(void)
{
    uint16_t wordData = 0;
    uint32_t portData;
    
    switch (Parallel_bootMode)
    {
        case PARALLEL_BOOT:
        default:        //
            // This routine tells the host that the DSP is ready to
            // receive data.  The DSP then waits for the host to
            // signal that data is ready on the GP I/O port.
            //
            GPIO_writePin(16, 0U);
            while(GPIO_readPin(29) != 0U){}

            //
            // Get LSB (pins 28, 1-7)
            //
            portData = GPIO_readPortData(GPIO_PORT_A);
            portData = (portData & 0xFEU) | ((portData & 0x10000000U) >> 28U);
            wordData = (uint16_t)(portData & 0xFFU);
            
            //
            // This routine tells the host that the DSP has received
            // the data.  The DSP then waits for the host to acknowledge
            // the receipt before continuing.
            //
            GPIO_writePin(16, 1U);
            while(GPIO_readPin(29) != 1U){}

            //
            // Fetch the MSB.
            //
            wordData = wordData & 0x00FFU;

            //
            // This routine tells the host that the DSP is ready to
            // receive data.  The DSP then waits for the host to
            // signal that data is ready on the GP I/O port.
            //
            GPIO_writePin(16, 0U);
            while(GPIO_readPin(29) != 0U){}

            portData = GPIO_readPortData(GPIO_PORT_A);
            portData = (portData & 0xFEU) | ((portData & 0x10000000U) >> 28U);
            wordData |= (uint16_t)((portData & 0xFFU) << 8U);

            //
            // This routine tells the host that the DSP has received
            // the data.  The DSP then waits for the host to acknowledge
            // the receipt before continuing.
            //
            GPIO_writePin(16, 1U);
            while(GPIO_readPin(29) != 1U){}
            break;
            
        case PARALLEL_BOOT_ALT1:            
            //
            // This routine tells the host that the DSP is ready to
            // receive data.  The DSP then waits for the host to
            // signal that data is ready on the GP I/O port.
            //
            GPIO_writePin(16, 0U);
            while(GPIO_readPin(11) != 0U){}

            //
            // Get LSB
            //
            wordData = (uint16_t)(GPIO_readPortData(GPIO_PORT_A) & 0xFFU);

            //
            // This routine tells the host that the DSP has received
            // the data.  The DSP then waits for the host to acknowledge
            // the receipt before continuing.
            //
            GPIO_writePin(16, 1U);
            while(GPIO_readPin(11) != 1U){}

            //
            // Fetch the MSB.
            //
            wordData = wordData & 0x00FFU;

            //
            // This routine tells the host that the DSP is ready to
            // receive data.  The DSP then waits for the host to
            // signal that data is ready on the GP I/O port.
            //
            GPIO_writePin(16, 0U);
            while(GPIO_readPin(11) != 0U){}

            wordData |= (uint16_t)((GPIO_readPortData(GPIO_PORT_A) & 0xFFU) << 8U);

            //
            // This routine tells the host that the DSP has received
            // the data.  The DSP then waits for the host to acknowledge
            // the receipt before continuing.
            //
            GPIO_writePin(16, 1U);
            while(GPIO_readPin(11) != 1U){}
            break;
    }
    return wordData;
}

//
// Parallel_GPIOSelect - Configure the GPIOs used for Parallel IO bootloader
//
// HOST_CTRL_GPIO is an input control from the Host
// to the DSP Ack/Rdy
// - may require an external pull-up
//
// DSP_CTRL_GPIO is an output from the DSP Ack/Rdy
// - may require an external pull-up for host to correctly
//   read "1" initially.
//
// DSP_CTRL_GPIO set to 1 initially
// 0 = input   1 = output
//
void Parallel_GPIOSelect(uint32_t  bootMode)
{
    //
    // Unlock the GPIO configuration registers
    //
    GPIO_unlockPortConfig(GPIO_PORT_A,0xFFFFFFFFUL);
    
    switch (bootMode)
    {
        case PARALLEL_BOOT:
        default:
            //
            // Enable Pull-ups on GPIO 28, 1-7
            //
            EALLOW;
            HWREG(GPIOCTRL_BASE + GPIO_O_GPAPUD)   &= (uint32_t) 0xEFFFFF01UL;
            EDIS;

            //
            // Configure GPIO 28, 1-7, 16, 29 as GPIO
            //
            GPIO_setPinConfig(GPIO_28_GPIO28);
            GPIO_setPinConfig(GPIO_1_GPIO01);
            GPIO_setPinConfig(GPIO_2_GPIO02);
            GPIO_setPinConfig(GPIO_3_GPIO03);
            GPIO_setPinConfig(GPIO_4_GPIO04);
            GPIO_setPinConfig(GPIO_5_GPIO05);
            GPIO_setPinConfig(GPIO_6_GPIO06);
            GPIO_setPinConfig(GPIO_7_GPIO07);
            GPIO_setPinConfig(GPIO_16_GPIO16);
            GPIO_setPinConfig(GPIO_29_GPIO29);
            
            //
            // Configure GPIO28, GPIO1-GPIO7, GPIO29 as input
            // Configure GPIO16 as output
            //
            EALLOW;
            HWREG(GPIOCTRL_BASE + GPIO_O_GPADIR)   &= (uint32_t) 0xCFFFFF01UL;
            HWREG(GPIOCTRL_BASE + GPIO_O_GPADIR)   |= (uint32_t) 0x00010000UL;
            EDIS;
            break;
        
        case PARALLEL_BOOT_ALT1:
            //
            // Enable Pull-ups on GPIO 0-7
            //
            EALLOW;
            HWREG(GPIOCTRL_BASE + GPIO_O_GPAPUD)   &= (uint32_t) 0xFFFFFF00UL;
            EDIS;

            //
            // Configure GPIO 0-7, 11, 16 as GPIO
            //
            GPIO_setPinConfig(GPIO_0_GPIO00);
            GPIO_setPinConfig(GPIO_1_GPIO01);
            GPIO_setPinConfig(GPIO_2_GPIO02);
            GPIO_setPinConfig(GPIO_3_GPIO03);
            GPIO_setPinConfig(GPIO_4_GPIO04);
            GPIO_setPinConfig(GPIO_5_GPIO05);
            GPIO_setPinConfig(GPIO_6_GPIO06);
            GPIO_setPinConfig(GPIO_7_GPIO07);
            GPIO_setPinConfig(GPIO_11_GPIO11);
            GPIO_setPinConfig(GPIO_16_GPIO16);

            //
            // Configure GPIO0-GPIO7, GPIO11 as input
            // Configure GPIO16 as output
            //
            EALLOW;
            HWREG(GPIOCTRL_BASE + GPIO_O_GPADIR)   &= (uint32_t) 0xFFFFF700UL;
            HWREG(GPIOCTRL_BASE + GPIO_O_GPADIR)   |= (uint32_t) 0x00010000UL;
            EDIS;
            break;
    }            
}
