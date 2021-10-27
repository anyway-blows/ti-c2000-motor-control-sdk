//###########################################################################
//
// FILE:   test_block_write_3_bytes.c
//
// TITLE:  Block Write (3 bytes)
//
//###########################################################################
// $TI Release: C28x PMBus Communications Stack Library v1.03.00.00 $
// $Release Date: Fri Feb 12 19:16:58 IST 2021 $
// $Copyright: Copyright (C) 2015-2021 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

//
// Includes
//
#include "pmbus_master_test.h"

//*****************************************************************************
//
// PMBusMaster_initBlockWrite3BytesTest
//
//*****************************************************************************
void PMBusMaster_initBlockWrite3BytesTest(PMBus_TestHandle handle)
{
    //
    // Reset flags
    //
    PMBusMaster_resetGlobalFlags();

    //
    // Reset test object
    //
    PMBusMaster_resetTestObject(handle);

    //
    // Set the block length to 3
    //
    handle->count = 3U;

    //
    // Enable the test
    //
#if ENABLE_TEST_12 == 1
    handle->enabled = true;
#else
    handle->enabled = false;
#endif //ENABLE_TEST_12 == 1
}

//*****************************************************************************
//
// PMBusMaster_runBlockWrite3BytesTest
//
//*****************************************************************************
void PMBusMaster_runBlockWrite3BytesTest(PMBus_TestHandle handle)
{
    //
    // Locals
    //
    uint16_t i = 0U;

    //
    // Put command in buffer
    //
    pmbusMasterBuffer[0] = PMBUS_CMD_PAGE_PLUS_WRITE;

    for(i = 1U; i <= handle->count; i++)
    {
        //
        // Bytes #0 to #N-1 (2)
        //
        pmbusMasterBuffer[i] = pmbusMasterBuffer[0] ^ i;
    }

    //
    // Block writes must be done in chunks of 4 bytes starting with the
    // command byte
    //

    //
    // Config the master to enable PEC, enable Write (by omitting the read
    // option from the configWord, you enable write), enable command,
    // >3 bytes to transfer
    //
    PMBus_configMaster(PMBUSA_BASE, SLAVE_ADDRESS, handle->count,
                       (PMBUS_MASTER_ENABLE_PEC | PMBUS_MASTER_ENABLE_CMD));

    //
    // Transfer the first 3 bytes, i.e., command, byte #0, #1, #2
    // NOTE: Byte count is automatically inserted after command
    //
    PMBus_putMasterData(PMBUSA_BASE, &pmbusMasterBuffer[0], 4U);

    //
    // NOTE: PEC is automatically inserted at the end of block write
    // transmission
    //

    //
    // Wait for the EOM, and slave to ack the address before
    // reading data -- done in the ISR
    //
    while(endOfMessage == false)
    {
    }

    //
    // Once the bus is free, if the slave NACK'd its a failure
    //
    (slaveAckReceived == true) ? handle->pass++ : handle->fail++;

    //
    // Check that expected pass counter value is correct
    //
    if(handle->pass != 1U)
    {
        handle->fail++;
    }

    return;
}

//
// End of File
//
