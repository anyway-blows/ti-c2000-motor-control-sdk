//###########################################################################
//
// FILE:   test_block_wrpc.c
//
// TITLE:  Block Write-Read Process Call (255 bytes) Test
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
// PMBusMaster_initBlockWriteReadProcessCallTest
//
//*****************************************************************************
void PMBusMaster_initBlockWriteReadProcessCallTest(PMBus_TestHandle handle)
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
    // Set count to 255
    //
    handle->count = 255U;

    //
    // Enable the test
    //
#if ENABLE_TEST_9 == 1
    handle->enabled = true;
#else
    handle->enabled = false;
#endif //ENABLE_TEST_9 == 1
}

//*****************************************************************************
//
// PMBusMaster_runBlockWriteReadProcessCallTest
//
//*****************************************************************************
void PMBusMaster_runBlockWriteReadProcessCallTest(PMBus_TestHandle handle)
{
    //
    // Locals
    //
    uint16_t i;
    uint16_t count = 0U;
    uint16_t nBytes = 0U;

    //
    // Clear the spot where the command will go
    //
    memset(&pmbusMasterBuffer[0], 0U, handle->count * sizeof(uint16_t));

    //
    // command, soon to be overwritten by block read
    //
    pmbusMasterBuffer[0]  = PMBUS_CMD_PAGE_PLUS_READ;

    for(i = 1U; i <= handle->count; i++)
    {
        //
        // Bytes #0 to #N-1
        //
        pmbusMasterBuffer[i] = pmbusMasterBuffer[0] ^ i;
    }

    //
    // Block-Write, Block-Read, and Process Call
    // Config the master to enable PEC, enable Write (by omitting the read
    // option from the configWord, you enable write), enable command,
    // enable process call (the master does the block read, with the repeated
    // start, right after the block write finishes)
    // Write, and then read, handle->count bytes
    //
    PMBus_configMaster(PMBUSA_BASE, SLAVE_ADDRESS, handle->count,
                       (PMBUS_MASTER_ENABLE_PEC | PMBUS_MASTER_ENABLE_PRC_CALL |
                        PMBUS_MASTER_ENABLE_CMD));

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
            // remaining bytes < 4U
            //
            PMBus_putMasterData(PMBUSA_BASE, &pmbusMasterBuffer[i],
                                (handle->count + 1U - i));

            i += (handle->count + 1U - i);
        }

        //
        // NOTE: PEC is not transmitted for a Block Write-Read Process Call
        //
    }
    //
    // Master automatically issues a repeated start (because of the process
    // call), and waits for the slave to respond with the block data...when
    // the necessary bytes (specified in PMBMC.BYTE_COUNT) is received, master
    // NACKs last byte and generates an EOM
    // However, if the slave is transmitting more than 3 bytes, the
    // DATA_READY goes high first, the RXBUF is full and must be read
    // before the slave can send any more data
    //

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
        if(pmbusMasterBuffer[i] == PMBUS_CMD_PAGE_PLUS_READ)
        {
            handle->pass++;
        }
        else
        {
            handle->fail++;
        }
    }

    (receivedPecValid == true) ? handle->pass++: handle->fail++;

    //
    // Check that expected pass counter value is correct
    //
    if(handle->pass != 258U)
    {
        handle->fail++;
    }

    return;
}

//
// End of File
//
