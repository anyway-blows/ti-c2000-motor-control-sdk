//#############################################################################
//
// FILE:   sdl_ex_ram_access_protect.c
//
// TITLE:  RAM access protection violation detection
//
//! \defgroup sdl_ex_ram_access_protect RAM access protection violation detection
//! <h1>sdl_ex_ram_access_protect</h1>
//!
//! This example demonstrates a functional test of the RAM access protection
//! violation detection and handling for several different types accesses. The
//! example generates a number of master or non-master fetch, write, or read
//! violations from the CPU, CLA, and DMA as applicable to the device.
//!
//! \b External \b Connections \n
//!  - None.
//!
//! \b Watch \b Variables \n
//!  - \b violationISRFlag - Indicates that the access violation interrupt was
//!       triggered and called the ISR.
//!  - \b illegalISRFlag - Indicates that a CPU fetch violation interrupt was
//!       generated and the ITRAP ISR ran.
//!  - \b violationStatus - Access protection violation status flags read in
//!       the ISR.
//!  - \b violationAddr - Address at which the access violation occurred.
//!  - \b result - Status of a successful detection and handling of RAM access
//!       protection violations.
//!
//
//#############################################################################
// $TI Release: C2000 Diagnostic Library v2.01.00 $
// $Release Date: Fri Feb 12 19:23:23 IST 2021 $
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
#define PASS                0U
#define FAIL                1U

#define LS7_ORIG_VALUE      0xCBA98765U
#define GS0_ORIG_VALUE1     0xF0F0F0F0U
#define GS0_ORIG_VALUE2     0x5A5A5A5AU

#define ISR_LOOP_TIMEOUT    0x3U

//
// Globals
//
uint32_t result = FAIL;
volatile bool violationISRFlag;
volatile bool illegalISRFlag;
uint32_t violationStatus;
uint32_t violationAddr;

//
// Reserving memory locations for testing.
//
#pragma DATA_SECTION(gs0Data1, "ramgs0");
volatile uint32_t gs0Data1 = GS0_ORIG_VALUE1;
#pragma DATA_SECTION(gs0Data2, "ramgs0");
volatile uint32_t gs0Data2 = GS0_ORIG_VALUE1;

//
// Function Prototypes
//
__interrupt void accessViolationISR(void);
__interrupt void illegalISR(void);
uint32_t generateMasterCPUWrViolation(void);
uint32_t generateMasterCPUFetchViolation(void);
uint32_t generateMasterDMAWrViolation(void);

#pragma CODE_SECTION(testCPUFetchFunc, "ramls7");
void testCPUFetchFunc(void);

#ifdef _FLASH
extern uint16_t ramls7LoadStart;
extern uint16_t ramls7LoadSize;
extern uint16_t ramls7RunStart;
#endif

//
// Main
//
void main(void)
{
    uint16_t failCount;

    //
    // Initialize device clock and peripherals.
    //
    Device_init();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

#ifdef _FLASH
    //
    // Need to copy testCPUFetchFunc() from Flash to RAM for the _FLASH build.
    //
    memcpy(&ramls7RunStart, &ramls7LoadStart, (size_t)&ramls7LoadSize);
#endif

    //
    // Clear access violation interrupt status.
    //
    MemCfg_clearViolationInterruptStatus(0xFFFFFFFFU);

    //
    // Enable access violation interrupt & register ISR.
    //
    Interrupt_register(INT_ILLEGAL, illegalISR);
    Interrupt_register(INT_RAM_ACC_VIOL, accessViolationISR);
    Interrupt_enable(INT_RAM_ACC_VIOL);

    //
    // Enable Global Interrupt (INTM) and Real Time interrupt (DBGM).
    //
    EINT;
    ERTM;

    //
    // Master CPU write violation generation & handling.
    //
    failCount = generateMasterCPUWrViolation();

    //
    // Master CPU fetch violation generation & handling.
    //
    failCount += generateMasterCPUFetchViolation();

    //
    // Master DMA write violation generation & handling.
    //
    failCount += generateMasterDMAWrViolation();

    //
    // Status of a successful generation, detection, and handling of RAM access
    // protection violations.
    //
    if(failCount != 0U)
    {
        result = FAIL;
    }
    else
    {
        result = PASS;
    }

    //
    // Loop here and check results in the CCS Expressions view.
    //
    while(1);
}

//
// illegalISR - ISR called when an ITRAP occurs, such as one generated by a
//              CPU fetch to a fetch-protected memory.
//
__interrupt void illegalISR(void)
{
    illegalISRFlag = true;

    //
    // Need to remove the protection on LS1RAM to allow us to return.
    //
    MemCfg_setProtection(MEMCFG_SECT_LS7, MEMCFG_PROT_ALLOWCPUFETCH);
}

//
// accessViolationISR - RAM access protection violation ISR.
//
__interrupt void accessViolationISR(void)
{
    violationISRFlag = true;

    //
    // Capture the violation type and the address at which violation occurred.
    //
    violationStatus = MemCfg_getViolationInterruptStatus();
    violationAddr = MemCfg_getViolationAddress(violationStatus);

    //
    // Clear flags.
    //
    MemCfg_clearViolationInterruptStatus(violationStatus);

    //
    // Clear the interrupt at PIE level.
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP12);
}

//
// generateMasterCPUWrViolation - Generate master CPU write violation.
//
uint32_t generateMasterCPUWrViolation()
{
    uint16_t fail = 0U;
    uint16_t timeout= ISR_LOOP_TIMEOUT;

    //
    // Enable violation interrupt.
    //
    MemCfg_enableViolationInterrupt(MEMCFG_MVIOL_CPUWRITE);

    //
    // Set write protection which generates master CPU write violation in
    // case of CPU writes.
    //
    MemCfg_setProtection(MEMCFG_SECT_GS0, MEMCFG_PROT_BLOCKCPUWRITE);

    //
    // Clear access violation status variables.
    //
    violationISRFlag = false;
    violationAddr = 0U;
    violationStatus = 0U;

    //
    // Write to memory will generate master CPU write violation as
    // CPU writes are blocked.
    //
    gs0Data1 = 0xEFFFFFFFU;

    while((violationISRFlag != true) && (timeout != 0U))
    {
        timeout--;
    }

    //
    // Disable interrupt.
    //
    MemCfg_disableViolationInterrupt(MEMCFG_MVIOL_CPUWRITE);

    //
    // Check if interrupt occurred as expected or if the loop timed out.
    //
    if(timeout == 0U)
    {
        fail++;
    }

    //
    // Check if the violation address was gs0Data1.
    //
    if(violationAddr != (uint32_t)&gs0Data1)
    {
        fail++;
    }

    //
    // Confirm the expected violation type was detected.
    //
    if((violationStatus & MEMCFG_MVIOL_CPUWRITE) == 0U)
    {
        fail++;
    }

    //
    // Confirm that the write didn't affect the data.
    //
    if(gs0Data1 != GS0_ORIG_VALUE1)
    {
        fail++;
    }

    return(fail);
}

//
// generateMasterCPUFetchViolation - Generate master CPU fetch violation.
//
uint32_t generateMasterCPUFetchViolation()
{
    uint16_t fail = 0U;
    uint16_t timeout= ISR_LOOP_TIMEOUT;

    //
    // Enable violation interrupt
    //
    MemCfg_enableViolationInterrupt(MEMCFG_MVIOL_CPUFETCH);

    //
    // Set fetch protection which generates master CPU fetch violation in
    // case of CPU fetch.
    //
    MemCfg_setProtection(MEMCFG_SECT_LS7, MEMCFG_PROT_BLOCKCPUFETCH);

    //
    // Clear access violation status variables
    //
    illegalISRFlag = false;
    violationISRFlag = false;
    violationAddr = 0U;
    violationStatus = 0U;

    //
    // Calling this function (which has been placed in LS1) will cause a CPU
    // fetch access violation.
    //
    testCPUFetchFunc();

    //
    // The violation will generate both an ITRAP interrupt and the RAM access
    // violation interrupt.
    //
    while(((illegalISRFlag != true) || (violationISRFlag != true)) &&
          (timeout != 0U))
    {
        timeout--;
    }

    //
    // Disable interrupt.
    //
    MemCfg_disableViolationInterrupt(MEMCFG_MVIOL_CPUFETCH);

    //
    // Check if interrupt occurred as expected or if the loop timed out.
    //
    if(timeout == 0U)
    {
        fail++;
    }

    //
    // Check if the violation address was testCPUFetchFunc().
    //
    if(violationAddr != (uint32_t)&testCPUFetchFunc)
    {
        fail++;
    }

    //
    // Confirm the expected violation type was detected.
    //
    if((violationStatus & MEMCFG_MVIOL_CPUFETCH) == 0U)
    {
        fail++;
    }

    return(fail);
}

//
// generateMasterDMAWrViolation - Generate master DMA read access violation.
//
uint32_t generateMasterDMAWrViolation(void)
{
    uint16_t fail = 0U;
    uint16_t timeout= ISR_LOOP_TIMEOUT;

    //
    // Enable interrupt.
    //
    MemCfg_enableViolationInterrupt(MEMCFG_MVIOL_DMAWRITE);

    //
    // Set write protection which generates an access violation in case of
    // DMA writes.
    //
    MemCfg_setProtection(MEMCFG_SECT_GS0, MEMCFG_PROT_BLOCKDMAWRITE);

    //
    // Clear access violation status variables.
    //
    violationISRFlag = false;
    violationAddr = 0U;
    violationStatus = 0U;

    //
    // Force master DMA write violation by configuration a DMA channel to
    // transfer data to the protected area.
    //
    DMA_initController();
    DMA_configAddresses(DMA_CH5_BASE, (void *)&gs0Data1,
                                      (void *)&gs0Data2);
    DMA_configBurst(DMA_CH5_BASE, 1U, 0U, 0U);
    DMA_configTransfer(DMA_CH5_BASE, 1U, 0U, 0U);
    DMA_configMode(DMA_CH5_BASE, DMA_TRIGGER_SOFTWARE, DMA_CFG_ONESHOT_DISABLE |
                   DMA_CFG_CONTINUOUS_DISABLE | DMA_CFG_SIZE_32BIT);
    DMA_startChannel(DMA_CH5_BASE);
    DMA_enableTrigger(DMA_CH5_BASE);
    DMA_forceTrigger(DMA_CH5_BASE);

    while((violationISRFlag != true) && (timeout != 0U))
    {
        timeout--;
    }

    //
    // Disable interrupt and DMA channel.
    //
    MemCfg_disableViolationInterrupt(MEMCFG_MVIOL_DMAWRITE);
    DMA_disableTrigger(DMA_CH5_BASE);

    //
    // Check if interrupt occurred as expected or if the loop timed out.
    //
    if(timeout == 0U)
    {
        fail++;
    }

    //
    // Check if the violation address was gs0Data1.
    //
    if(violationAddr != (uint32_t)&gs0Data1)
    {
        fail++;
    }

    //
    // Confirm the expected violation type was detected.
    //
    if((violationStatus & MEMCFG_MVIOL_DMAWRITE) == 0U)
    {
        fail++;
    }

    //
    // Confirm that the write didn't affect the data.
    //
    if(gs0Data1 != GS0_ORIG_VALUE1)
    {
        fail++;
    }

    return(fail);
}

//
// testCPUFetchFunc - Function to be called to generate a CPU fetch violation.
//
void testCPUFetchFunc(void)
{
    NOP;
}

//
// End of File
//
