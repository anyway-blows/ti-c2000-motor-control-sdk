//###########################################################################
//
// FILE:   test_block_write.c
//
// TITLE:  Block Write (255 bytes) Test
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
// PMBusMaster_initBlockWriteTest
//
//*****************************************************************************
void PMBusMaster_initBlockWriteTest(PMBus_TestHandle handle)
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
    // Set the block length to 255
    //
    handle->count = 255U;

    //
    // Enable the test
    //
#if ENABLE_TEST_4 == 1
    handle->enabled = true;
#else
    handle->enabled = false;
#endif //ENABLE_TEST_4 == 1
}

//*****************************************************************************
//
// PMBusMaster_runBlockWriteTest
//
//*****************************************************************************
void PMBusMaster_runBlockWriteTest(PMBus_TestHandle handle)
{
    //
    // Locals
    //
    uint16_t i = 0U;

    //
    // Set command in buffer
    //
    pmbusMasterBuffer[0] = PMBUS_CMD_VIN_OV_FAULT_LIMIT;

    for(i = 1U; i <= handle->count; i++)
    {
        //
        // Bytes #0 to #N-1 (254)
        //
        pmbusMasterBuffer[i] = i - 1;
    }

    //
    // Block writes must be done in chunks of 4 bytes starting with the
    // command byte
    //

    //
    // Config the master to enable PEC, enable Write (by omitting the read
    // option from the configWord, you enable write), enable command,
    // >=3 bytes to transfer
    //
    PMBus_configMaster(PMBUSA_BASE, SLAVE_ADDRESS, handle->count,
                       (PMBUS_MASTER_ENABLE_PEC | PMBUS_MASTER_ENABLE_CMD));

    //
    // Transfer the first 4 bytes, i.e., command, byte #0, #1, #2
    // NOTE: Byte count is automatically inserted after command
    //
    PMBus_putMasterData(PMBUSA_BASE, &pmbusMasterBuffer[0], 4U);

    //
    // Write the remaining bytes in chunks of 4, last transaction is N+2 % 4
    //
    for(i = 4U; i <= handle->count;)
    {
        //
        // Wait for DATA_REQUEST to assert
        //
        while(masterDataRequested == false)
        {
        }

        masterDataRequested = false;

        if((handle->count - i - 1U) >= 4U)
        {
            //
            // Transfer 4 bytes at a time
            //
            PMBus_putMasterData(PMBUSA_BASE, &pmbusMasterBuffer[i], 4U);

            i = i + 4U;
        }
        else
        {
            //
            // Remaing bytes < 4U
            //
            PMBus_putMasterData(PMBUSA_BASE, &pmbusMasterBuffer[i],
                                (handle->count + 1U - i));

            i += (handle->count + 1U - i);
        }

        //
        // NOTE: PEC is automatically inserted at the end of block write
        // transmission
        //
    }

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
