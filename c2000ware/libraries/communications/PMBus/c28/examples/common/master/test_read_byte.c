//###########################################################################
//
// FILE:   test_read_byte.c
//
// TITLE:  Read Byte Test
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
// PMBusMaster_initReadByteTest
//
//*****************************************************************************
void PMBusMaster_initReadByteTest(PMBus_TestHandle handle)
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
    // Set count to 1
    //
    handle->count = 1;

    //
    // Enable the test
    //
#if ENABLE_TEST_6 == 1
    handle->enabled = true;
#else
    handle->enabled = false;
#endif //ENABLE_TEST_6 == 1
}

//*****************************************************************************
//
// PMBusMaster_runReadByteTest
//
//*****************************************************************************
void PMBusMaster_runReadByteTest(PMBus_TestHandle handle)
{
    //
    // Locals
    //
    uint16_t nBytes = 0U;

    //
    // Clear the spot where the command will go
    //
    memset(&pmbusMasterBuffer[0], 0U, handle->count * sizeof(uint16_t));

    //
    // Do a read byte
    // Config the master to send a read-byte command (by asserting the command
    // bit) and then to issue a repeated start followed by a read (i.e. enable
    // read) and get a data byte and a PEC from the slave. Also enable PEC
    // proecessing on the master side to verify the PEC
    //
    PMBus_configMaster(PMBUSA_BASE, SLAVE_ADDRESS, 1U,
                       (PMBUS_MASTER_ENABLE_PEC | PMBUS_MASTER_ENABLE_READ |
                        PMBUS_MASTER_ENABLE_CMD));

    //
    // Load the command byte into the transmit register,
    // in this case the PMBUS_CMD_OPERATION command
    //
    pmbusMasterBuffer[0] = PMBUS_CMD_OPERATION;
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
    }

    //
    // Read received data
    //
    nBytes = PMBus_getMasterData(PMBUSA_BASE, &pmbusMasterBuffer[0],
                                 pmbusStatus);

    //
    // Check the received byte, data, and PEC
    //
    (nBytes == 1U) ? handle->pass++ : handle->fail++;
    (pmbusMasterBuffer[0] == (PMBUS_CMD_OPERATION ^ CMD_XOR_MASK_V1)) ?
                             handle->pass++ : handle->fail++;
    (receivedPecValid == true) ? handle->pass++ : handle->fail++;

    //
    // Check that expected pass counter value is correct
    //
    if(handle->pass != 3U)
    {
        handle->fail++;
    }

    return;
}

//
// End of File
//
