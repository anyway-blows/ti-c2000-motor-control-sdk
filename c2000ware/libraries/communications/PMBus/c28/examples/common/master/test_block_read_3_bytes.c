//###########################################################################
//
// FILE:   test_block_read_3_bytes.c
//
// TITLE:  Block Read (3 bytes) Test
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
// PMBusMaster_initBlockRead3BytesTest
//
//*****************************************************************************
void PMBusMaster_initBlockRead3BytesTest(PMBus_TestHandle handle)
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
    // Set count to 3
    //
    handle->count = 3U;

    //
    // Enable the test
    //
#if ENABLE_TEST_13 == 1
    handle->enabled = true;
#else
    handle->enabled = false;
#endif //ENABLE_TEST_13 == 1
}

//*****************************************************************************
//
// PMBusMaster_runBlockRead3BytesTest
//
//*****************************************************************************
void PMBusMaster_runBlockRead3BytesTest(PMBus_TestHandle handle)
{
    //
    // Locals
    //
    uint16_t count = 0U;
    uint16_t nBytes = 0U;
    uint16_t i = 0U;

    //
    // Clear the spot where the command will go
    //
    memset(&pmbusMasterBuffer[0], 0U, handle->count * sizeof(uint16_t));

    //
    // Do a read byte
    // Config the master to send a read-block command (by asserting the command
    // bit) and then to issue a repeated start followed by a read (i.e. enable
    // read) and get handle->count data bytes and a PEC from the slave. Also
    // enable PEC proecessing on the master side to verify the PEC
    //
    PMBus_configMaster(PMBUSA_BASE, SLAVE_ADDRESS, handle->count,
                       (PMBUS_MASTER_ENABLE_PEC | PMBUS_MASTER_ENABLE_READ |
                        PMBUS_MASTER_ENABLE_CMD));

    //
    // Load the command byte into the transmit register,
    // in this case the PMBUS_CMD_READ_EOUT command
    //
    pmbusMasterBuffer[0] = PMBUS_CMD_READ_EOUT;
    PMBus_putMasterData(PMBUSA_BASE, &pmbusMasterBuffer[0], 1U);

    //
    // Wait for the EOM, the slave transmits all its data, the
    // master NACKs it, and then issues an EOM on the bus
    // -- this flag is set in the ISR
    // NOTE: Although the master issues a NACK to the slave to indicate the
    // end of transmission, the NACK bit (status) is not asserted at the master
    // end, only at the slave end.
    //
    while(endOfMessage == false)
    {
        if(masterDataAvailable == true)
        {
            //
            // Reset the flag
            //
            masterDataAvailable = false;
            count = PMBus_getMasterData(PMBUSA_BASE, &pmbusMasterBuffer[nBytes],
                                        pmbusStatus);
            nBytes += count;
        }
    }

    if((endOfMessage == true) && (masterDataAvailable == true))
    {
        //
        // EOM = 1 DATA_READY = 1, but this wasnt captured in the while loop
        // above
        //
        count = PMBus_getMasterData(PMBUSA_BASE, &pmbusMasterBuffer[nBytes],
                                    pmbusStatus);
        nBytes += count;

        //
        // No need to reset these flags, they will be reset at the start of the
        // next test
        //
    }

    //
    // Check that we got nBytes + 1 (byte count) bytes
    //
    (nBytes == handle->count + 1) ? handle->pass++ : handle->fail++;
    (pmbusMasterBuffer[0] == handle->count) ? handle->pass++ : handle->fail++;

    for(i = 1U; i <= handle->count; i++)
    {
        if(pmbusMasterBuffer[i] == (PMBUS_CMD_READ_EOUT ^ i))
        {
            handle->pass++;
        }
        else
        {
            handle->fail++;
        }
    }

    (receivedPecValid == true) ? handle->pass++ : handle->fail++;

    //
    // Check that expected pass counter value is correct
    //
    if(handle->pass != 6U)
    {
        handle->fail++;
    }

    return;
}

//
// End of File
//
