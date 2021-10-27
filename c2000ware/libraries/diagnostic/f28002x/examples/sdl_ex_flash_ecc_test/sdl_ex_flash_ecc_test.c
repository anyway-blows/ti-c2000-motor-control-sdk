//#############################################################################
//
// FILE:   sdl_ex_flash_ecc_test.c
//
// TITLE:  Test of ECC logic in Flash
//
//! \defgroup sdl_ex_flash_ecc_test Test of ECC logic in Flash
//! <h1>sdl_ex_flash_ecc_test</h1>
//!
//! This example demonstrates how to test the Flash ECC logic functionality.
//!
//! A software test of the Flash ECC logic can be performed with the help of
//! ECC test registers. Using the test registers, you can generate both single
//! bit errors and uncorrectable errors. For additional details on the
//! implementation of this diagnostic, see the "SECDED Logic Correctness Check"
//! section in the device technical reference manual.
//!
//! \b External \b Connections \n
//!  - None.
//!
//! \b Watch \b Variables \n
//!  - \b nmiISRFlag - Indicates that the NMI was triggered and called the ISR.
//!  - \b nmiStatus - NMI status flags read in the ISR.
//!  - \b errorISRFlag - Indicates that the correctable ECC error interrupt was
//!       triggered and called the ISR.
//!  - \b errorStatus - Error status flags read in the ISR, indicating whether
//!       an error was single-bit or uncorrectable.
//!  - \b errorCount - Number of correctable errors detected read in the ISR.
//!  - \b errorType - For a single-bit error, indicates whether it was in the
//!       data or ECC bits.
//!  - \b dataOut - For a single-bit error, displays the corrected data.
//!  - \b result - Status of a successful detection and handling of ECC errors.
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
#define PASS                    0U
#define FAIL                    1U

#define ISR_LOOP_TIMEOUT        0x3U

//
// Flash ECC Logic test values
//
#define TEST_FLASH_DATAL        ((uint32_t)(TEST_FLASH_DATA64))
#define TEST_FLASH_DATAH        ((uint32_t)(TEST_FLASH_DATA64 >> 32))
#define TEST_FLASH_ADDR         0x00090000UL
#define TEST_FLASH_DATA64       0xFEDCBA0987654321ULL

//
// Typedefs
//
typedef enum
{
    TEST_ECC_BLOCK_LOW  = 0x00,    //!< Low 64-bit block
    TEST_ECC_BLOCK_HIGH = 0x02     //!< High 64-bit block
} ECCBlock;

//
// Globals
//
uint16_t calcECC;
uint32_t result = FAIL;
volatile bool nmiISRFlag;
uint16_t nmiStatus;
volatile bool errorISRFlag;
uint16_t errorStatus;
uint16_t errorCount;
uint32_t errorType;
uint64_t dataOut;

//
// Function Prototypes
//
__interrupt void nmiISR(void);
__interrupt void corrErrorISR(void);
uint16_t runCorrectableDataErrorTest(void);
uint16_t runCorrectableECCErrorTest(void);
uint16_t runUncorrectableErrorTest(void);
void testECCBlock(uint64_t data, uint32_t address, ECCBlock eccBlock,
                  uint16_t ecc);
#pragma CODE_SECTION(testECCBlock, ".TI.ramfunc");
uint16_t calculateECC(uint32_t address, uint64_t data);

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

    //
    // Clear all the NMI and Flash error status flags.
    //
    EALLOW;
    HWREG(FLASH0ECC_BASE + FLASH_O_ERR_STATUS_CLR) = 0xFFFFFFFFU;
    HWREG(FLASH0ECC_BASE + FLASH_O_ERR_INTCLR) = 0xFFFFFFFFU;
    EDIS;

    SysCtl_clearAllNMIFlags();

    //
    // Plug the NMI and Flash correctable error ISRs.
    //
    Interrupt_register(INT_NMI, &nmiISR);
    Interrupt_register(INT_FLASH_CORR_ERR, &corrErrorISR);

    //
    // Enabling the NMI global interrupt (typically already enabled by boot ROM
    // or GEL file).
    //
    SysCtl_enableNMIGlobalInterrupt();

    //
    // Enable Flash correctable error interrupt.
    //
    Interrupt_enable(INT_FLASH_CORR_ERR);

    //
    // Enable Global Interrupt (INTM) and Real Time interrupt (DBGM).
    //
    EINT;
    ERTM;

    //
    // Calculate ECC on test data.
    //
    calcECC = calculateECC(TEST_FLASH_ADDR, TEST_FLASH_DATA64);

    //
    // Enable ECC.
    //
    Flash_enableECC(FLASH0ECC_BASE);

    //
    // Test detection of correctable ECC errors in Flash data bits.
    //
    failCount = runCorrectableDataErrorTest();

    //
    // Test detection of correctable ECC errors in Flash ECC bits.
    //
    failCount += runCorrectableECCErrorTest();

    //
    // Test detection of uncorrectable ECC errors in Flash.
    //
    failCount += runUncorrectableErrorTest();

    //
    // Status of a successful handling of the ECC errors.
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
// nmiISR -  The interrupt service routine called when the NMI is generated
//           on an uncorrectable Flash ECC error.
//
__interrupt void nmiISR(void)
{
    //
    // Set a flag indicating the NMI ISR occurred and get the NMI status.
    //
    nmiISRFlag = true;
    nmiStatus = SysCtl_getNMIFlagStatus();

    //
    // Record the error test status.
    //
    errorStatus = Flash_getECCTestStatus(FLASH0ECC_BASE);

    //
    // Clear all the flags.
    //
    EALLOW;
    HWREG(FLASH0ECC_BASE + FLASH_O_ERR_STATUS_CLR) = 0xFFFFFFFFU;
    EDIS;

    Flash_clearUncorrectableInterruptFlag(FLASH0ECC_BASE);
    SysCtl_clearAllNMIFlags();
}

//
// corrErrorISR - The interrupt service routine called when the correctable
//                error count hits the configured interrupt threshold.
//
__interrupt void corrErrorISR(void)
{
    //
    // Set a flag indicating the RAM error ISR occurred.
    //
    errorISRFlag = true;

    //
    // Record the type of error injected, the status flags, the corrected data,
    // and the number of single-bit ECC errors detected.
    //
    errorType = Flash_getECCTestSingleBitErrorType(FLASH0ECC_BASE);
    errorStatus = Flash_getECCTestStatus(FLASH0ECC_BASE);
    dataOut = ((uint64_t)Flash_getTestDataOutHigh(FLASH0ECC_BASE) << 32) |
              (uint64_t)Flash_getTestDataOutLow(FLASH0ECC_BASE);
    errorCount = Flash_getErrorCount(FLASH0ECC_BASE);

    //
    // Clear all the flags.
    //
    EALLOW;
    HWREG(FLASH0ECC_BASE + FLASH_O_ERR_STATUS_CLR) = 0xFFFFFFFFU;
    EDIS;

    Flash_clearSingleErrorInterruptFlag(FLASH0ECC_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP12);
}

//
// runCorrectableDataErrorTest - Runs a test of the correctable data error
//      detection and correction as well as the error interrupt threshold.
//
uint16_t runCorrectableDataErrorTest(void)
{
    uint16_t fail = 0U;
    uint16_t timeout= ISR_LOOP_TIMEOUT;

    //
    // Clear error status variables.
    //
    errorISRFlag = false;
    errorStatus = 0U;
    errorType = 0U;
    errorCount = 0U;
    dataOut = 0U;

    //
    // Configure the correctable error interrupt threshold to >1 errors.
    //
    Flash_setErrorThreshold(FLASH0ECC_BASE, 1U);

    //
    // Flip a single bit in the data to inject a correctable error for the
    // test to detect.
    //
    testECCBlock(TEST_FLASH_DATA64 ^ 0x80U, TEST_FLASH_ADDR,
                 TEST_ECC_BLOCK_LOW, calcECC);

    //
    // Generate another single bit error to increase the count above the
    // threshold.
    //
    testECCBlock(TEST_FLASH_DATA64 ^ ((uint64_t)0x1U << 37), TEST_FLASH_ADDR,
                 TEST_ECC_BLOCK_LOW, calcECC);

    //
    // Wait until the error interrupt is fired.
    //
    while((errorISRFlag != true) && (timeout != 0U))
    {
        timeout--;
    }

    //
    // Check if interrupt occurred as expected or if the loop timed out.
    //
    if(timeout == 0U)
    {
        fail++;
    }

    //
    // Check if the appropriate correctable Flash error flag was set.
    //
    if((errorStatus & FLASH_SINGLE_ERROR) != FLASH_SINGLE_ERROR)
    {
        fail++;
    }

    //
    // Check if the error count is 2.
    //
    if(errorCount != 2U)
    {
        fail++;
    }

    //
    // Check if the error was a data error.
    //
    if(errorType != FLASH_DATA_ERR)
    {
        fail++;
    }

    //
    // Check that the data value was corrected properly.
    //
    if(dataOut != TEST_FLASH_DATA64)
    {
        fail++;
    }

    return(fail);
}

//
// runCorrectableECCErrorTest - Runs a test of the correctable error detection.
//
uint16_t runCorrectableECCErrorTest(void)
{
    uint16_t fail = 0U;
    uint16_t timeout= ISR_LOOP_TIMEOUT;

    //
    // Clear error status variables.
    //
    errorISRFlag = false;
    errorStatus = 0U;
    errorType = 0U;

    //
    // Configure the correctable error interrupt threshold to >0 errors.
    //
    Flash_setErrorThreshold(FLASH0ECC_BASE, 0U);

    //
    // Flip a single bit in the data to inject a correctable error for the
    // test to detect.
    //
    testECCBlock(TEST_FLASH_DATA64, TEST_FLASH_ADDR,
                 TEST_ECC_BLOCK_LOW, calcECC ^ 0x2U);

    //
    // Wait until the error interrupt is fired.
    //
    while((errorISRFlag != true) && (timeout != 0U))
    {
        timeout--;
    }

    //
    // Check if interrupt occurred as expected or if the loop timed out.
    //
    if(timeout == 0U)
    {
        fail++;
    }

    //
    // Check if the appropriate correctable Flash error flag was set.
    //
    if((errorStatus & FLASH_SINGLE_ERROR) != FLASH_SINGLE_ERROR)
    {
        fail++;
    }

    //
    // Check if the error was an ECC error.
    //
    if(errorType != FLASH_ECC_ERR)
    {
        fail++;
    }

    return(fail);
}

//
// runUncorrectableErrorTest - Runs a test of the uncorrectable data error
//      detection and associated NMI generation.
//
uint16_t runUncorrectableErrorTest(void)
{
    uint16_t fail = 0U;
    uint16_t timeout= ISR_LOOP_TIMEOUT;

    //
    // Clear error status variables.
    //
    nmiISRFlag = false;
    nmiStatus = 0U;
    errorStatus = 0U;

    //
    // Flip multiple bits in the data to inject an uncorrectable error for the
    // test to detect.
    //
    testECCBlock(TEST_FLASH_DATA64 ^ 0x30U, TEST_FLASH_ADDR,
                 TEST_ECC_BLOCK_LOW, calcECC);

    //
    // Wait until the NMI is fired.
    //
    while((nmiISRFlag != true) && (timeout != 0U))
    {
        timeout--;
    }

    //
    // Check if interrupt occurred as expected or if the loop timed out.
    //
    if(timeout == 0U)
    {
        fail++;
    }

    //
    // Check if the NMI triggered was due to an uncorrectable Flash error.
    //
    if((nmiStatus & SYSCTL_NMI_FLUNCERR) != SYSCTL_NMI_FLUNCERR)
    {
        fail++;
    }

    //
    // Check if the appropriate uncorrectable Flash error flag was set.
    //
    if((errorStatus & FLASH_UNC_ERROR) != FLASH_UNC_ERROR)
    {
        fail++;
    }

    return(fail);
}

//
// testECCBlock - Sets up the ECC test registers and performs the ECC
//                calculation to detect and error if one is injected.
//
void testECCBlock(uint64_t data, uint32_t address, ECCBlock eccBlock,
                  uint16_t ecc)
{
    //
    // Write 128-bit flash address in FADDR_TEST
    //
    Flash_setECCTestAddress(FLASH0ECC_BASE, address);

    //
    // Write lower 32 bits of data in FDATAL_TEST
    //
    Flash_setDataLowECCTest(FLASH0ECC_BASE, (uint32_t)data);

    //
    // Write upper 32 bits of data in FDATAH_TEST
    //
    Flash_setDataHighECCTest(FLASH0ECC_BASE, (uint32_t)(data >> 32));

    //
    // Write corresponding ECC in the FECC_TEST
    // Insert double bit error in flash ecc
    //
    Flash_setECCTestECCBits(FLASH0ECC_BASE, ecc);

    //
    // Select the ECC block to be tested. Only one of the SECDED modules (out
    // of the two SECDED modules that work on lower 64 bits and upper 64 bits
    // of a read 128-bit data) at a time can be tested.
    //
    if(eccBlock == TEST_ECC_BLOCK_LOW)
    {
        Flash_selectLowECCBlock(FLASH0ECC_BASE);
    }
    else
    {
        Flash_selectHighECCBlock(FLASH0ECC_BASE);
    }

    //
    // Enable the ECC Test Mode
    //
    Flash_enableECCTestMode(FLASH0ECC_BASE);

    //
    // Perform ECC calculation
    //
    Flash_performECCCalculation(FLASH0ECC_BASE);

    //
    // Disable the ECC Test Mode
    //
    Flash_disableECCTestMode(FLASH0ECC_BASE);
}

//
// calcECC - Calculate the ECC for an address/data pair. This code comes from
//           the ECC Calculation Algorithm appendix of the Flash API reference
//           guide (SPNU628). If you are using the Flash API in your code, you
//           could use Fapi_calculateEcc() instead.
//
uint16_t calculateECC(uint32_t address, uint64_t data)
{
    const uint32_t addrSyndrome[8] = {0x554EAU, 0x0BAD1U, 0x2A9B5U, 0x6A78DU,
                                      0x19F83U, 0x07F80U, 0x7FF80U, 0x0007FU};
    const uint64_t dataSyndrome[8] = {0xB4D1B4D14B2E4B2EU, 0x1557155715571557U,
                                      0xA699A699A699A699U, 0x38E338E338E338E3U,
                                      0xC0FCC0FCC0FCC0FCU, 0xFF00FF00FF00FF00U,
                                      0xFF0000FFFF0000FFU, 0x00FFFF00FF0000FFU};
    const uint16_t parity = 0xFCU;
    uint64_t xorData;
    uint32_t xorAddr;
    uint16_t bit, eccBit, eccVal;

    //
    // Extract bits "20:2" of the address
    //
    address = (address >> 2) & 0x7FFFFU;

    //
    // Compute the ECC one bit at a time.
    //
    eccVal = 0U;

    for (bit = 0U; bit < 8U; bit++)
    {
        //
        // Apply the encoding masks to the address and data
        //
        xorAddr = address & addrSyndrome[bit];
        xorData = data & dataSyndrome[bit];

        //
        // Fold the masked address into a single bit for parity calculation.
        // The result will be in the LSB.
        //
        xorAddr = xorAddr ^ (xorAddr >> 16);
        xorAddr = xorAddr ^ (xorAddr >> 8);
        xorAddr = xorAddr ^ (xorAddr >> 4);
        xorAddr = xorAddr ^ (xorAddr >> 2);
        xorAddr = xorAddr ^ (xorAddr >> 1);

        //
        // Fold the masked data into a single bit for parity calculation.
        // The result will be in the LSB.
        //
        xorData = xorData ^ (xorData >> 32);
        xorData = xorData ^ (xorData >> 16);
        xorData = xorData ^ (xorData >> 8);
        xorData = xorData ^ (xorData >> 4);
        xorData = xorData ^ (xorData >> 2);
        xorData = xorData ^ (xorData >> 1);

        //
        // Merge the address and data, extract the ECC bit, and add it in
        //
        eccBit = ((uint16_t)xorData ^ (uint16_t)xorAddr) & 0x0001U;
        eccVal |= eccBit << bit;
    }

    //
    // Handle the bit parity. For odd parity, XOR the bit with 1
    //
    eccVal ^= parity;
    return eccVal;
}

//
// End of File
//
