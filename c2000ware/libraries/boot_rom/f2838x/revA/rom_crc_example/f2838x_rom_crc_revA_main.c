//#############################################################################
//
// FILE:    f2838x_rom_crc_revA_main.c
//
// TITLE:   F2838x ROM CRC RevA Example
//
// This example demonstrates how to calculate the CRCs for unsecure boot ROM
// and boot content in TI OTP. Using the VCRC is how the golden CRCs are
// calculated and stored in TI OTP. Calculating the CRCs and comparing to the
// golden CRCs in TI OTP enables sanity checking that boot ROM contents are
// unaltered. The calculation runs on the following address ranges:
// - Unsecure ROM address range: 0x3E8000 to 0x3FFFFF
// - TI OTP address range: 0x70260 to 0x703D0
// - Golden Unsecure ROM CRC location address: 0x703CC
// - Golden TI OTP ROM contents CRC location address: 0x703CE
//
// Note that the TI OTP ROM content CRC is seeded with the CRC of the unsecure
// boot ROM CRC.
//
// Watch Variables
// - crcRomResult : Contains the 32-bit CRC for unsecure boot ROM
// - crcOTPResult : Contains the 32-bit CRC for TI OTP containing boot content
// - crcStatus : Indicates the status of the CRC calculation
//               (1 = both CRCs match, 0 = one or more of the CRCs don't match)
//
//#############################################################################
// $TI Release: F2838x ROM revA $
// $Release Date: July, 31 2020 $
// $Copyright: Copyright (C) 2020 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//#############################################################################

//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "vcu2_crc.h"

//
// Defines
//
#define UNSECURE_ROM_START_ADDRESS      0x3E8000
#define UNSECURE_ROM_CRC_LENGTH         0x30000
#define UNSECURE_ROM_CRC_GOLDEN_ADDRESS 0x703CC
#define OTP_CRC_START_ADDRESS           0x70260
#define OTP_CRC_LENGTH                  0x2D6
#define OTP_CRC_GOLDEN_ADDRESS          0x703CE

//
// Globals
//
volatile uint32_t crcRomResult;
volatile uint32_t crcOTPResult;
volatile uint16_t crcStatus = 0;
volatile uint16_t failCount = 0;

//
// Declare VCRC structures
// (Refer to the VCRC library in C2000Ware
//  for more usage details)
//
CRC_Obj CRC;
CRC_Handle handleCRC;

//
// Function Prototypes
//
void calculateUnsecureROMCRC();
void calculateOTPCRC();

//
// Main
//
void main(void)
{
    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Initialize the VCRC handle
    //
    handleCRC = &CRC;

	//
	// Generate CRC for Unsecure ROM
	// Range - 0x3E8000 to 0x3FFFFF
	// Length = 0x30000 (in number of bytes)
	//
    calculateUnsecureROMCRC();

    //
    // Generate CRC for TI OTP (ROM content only)
    // Range - 0x70260 to 0x703CB
    // Length = 0x2D6 (in number of bytes)
    //
    // Seeded with unsecure ROM CRC
    //
    calculateOTPCRC();

    //
    // Check for mismatch errors
    //
    if(failCount == 0)
    {
        crcStatus = 1;
    }

    //
    // CRC Example Complete
    //
    ESTOP0;
}

//
// calculateUnsecureROMCRC - Calculate the unsecure ROM CRC and compare
//                           to golden CRC value
//
void calculateUnsecureROMCRC()
{
    uint16_t i;
    uint16_t q, r;

    //
    // Break length into blocks of length 0xFFFF
    //
    q = (uint16_t)((uint32_t)((UNSECURE_ROM_CRC_LENGTH) & 0xFFFF0000) >> 16);

    //
    // Since we can do a max of 65535 bytes at a time,
    // we need to add q to r to do the remainder
    //
    r = (uint16_t)(((UNSECURE_ROM_CRC_LENGTH) & 0x0000FFFF)) + q;

    //
    // Set starting address of unsecure ROM
    //
    uint16_t * address = (uint16_t *)UNSECURE_ROM_START_ADDRESS;

    //
    // Configure the parameters VCRC requires
    //
    CRC.crcResult = 0;
    CRC.seedValue = INIT_CRC32;
    CRC.parity = CRC_parity_even;
    CRC.pMsgBuffer = (uint16_t *)&address[0];
    CRC.run = (void (*)(void *))CRC_run32BitPoly1;

    //
    // Run the CRC for 65535 bytes q times
    //
    for(i = 0; i < q; i++)
    {
        //
        // When CRC starts on low byte, it ends on a low byte, the next
        // set of 65535 bytes must start with high byte of the last address
        //
        CRC.nMsgBytes = 0xFFFF;
        CRC.run(handleCRC);
        CRC.seedValue = CRC.crcResult;  // Seed with previous 65535 byte CRC
        CRC.parity ^= 1;
        CRC.pMsgBuffer = (uint16_t *)((uint32_t)(address + ((i+1)*0x7FFFUL) - CRC.parity));
    }
    //
    // Run the final CRC calculation on remaining data
    //
    CRC.nMsgBytes = r;
    CRC.run(handleCRC);

    //
    // Capture the result
    //
    crcRomResult = CRC.crcResult;

    //
    // Compare with golden CRC value
    //
    if(crcRomResult != HWREG(UNSECURE_ROM_CRC_GOLDEN_ADDRESS))
    {
        failCount++;
    }
}

//
// calculateOTPCRC - Calculate the TI OTP ROM contents CRC and compare
//                   to golden CRC value
//
void calculateOTPCRC()
{
    //
    // Set the starting address
    //
    uint16_t * address = (uint16_t *)OTP_CRC_START_ADDRESS;

    //
    // Configure the parameters VCRC requires
    // (Seed with the unsecure ROM CRC)
    //
    CRC.crcResult = 0;
    CRC.parity = CRC_parity_even;
    CRC.pMsgBuffer = (uint16_t *)&address[0];
    CRC.nMsgBytes = OTP_CRC_LENGTH;
    CRC.seedValue = crcRomResult;
    CRC.run = (void (*)(void *))CRC_run32BitPoly1;

    //
    // Run the CRC calculation
    //
    CRC.run(handleCRC);

    //
    // Capture the result
    //
    crcOTPResult = CRC.crcResult;

    //
    // Compare with golden CRC value
    //
    if(crcOTPResult != HWREG(OTP_CRC_GOLDEN_ADDRESS))
    {
        failCount++;
    }
}

//
// End of file
//
