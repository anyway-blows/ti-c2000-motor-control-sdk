//###########################################################################
//
// FILE:   test_extended_read_word.c
//
// TITLE:  Read Word (Extended) Test
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
// PMBusMaster_initExtendedReadWordTest
//
//*****************************************************************************
void PMBusMaster_initExtendedReadWordTest(PMBus_TestHandle handle)
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
    // Set count to 2
    //
    handle->count = 2;

    //
    // Enable the test
    //
#if ENABLE_TEST_21 == 1
    handle->enabled = true;
#else
    handle->enabled = false;
#endif //ENABLE_TEST_21 == 1
}

//*****************************************************************************
//
// PMBusMaster_runExtendedReadWordTest
//
//*****************************************************************************
void PMBusMaster_runExtendedReadWordTest(PMBus_TestHandle handle)
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
    // Do a extended read byte
    // Configure the master to send a read-word command (by asserting the
    // command and extended command bits) and then to issue a repeated start
    // followed by a read (i.e. enable read) and get two data bytes and a
    // PEC from the slave. Also enable PEC processing on the master side to
    // verify the PEC
    //
    PMBus_configMaster(PMBUSA_BASE, SLAVE_ADDRESS, TEST21_2BYTES,
                       (PMBUS_MASTER_ENABLE_PEC | PMBUS_MASTER_ENABLE_READ |
                        PMBUS_MASTER_ENABLE_CMD | PMBUS_MASTER_ENABLE_EXT_CMD));

    //
    // Load the extended command and command bytes into the transmit register,
    // in this case the PMBUS_CMD_VOUT_COMMAND command
    //
    pmbusMasterBuffer[0] = PMBUS_CMD_MFR_SPECIFIC_COMMAND;
    pmbusMasterBuffer[1] = PMBUS_CMD_VOUT_COMMAND;
    PMBus_putMasterData(PMBUSA_BASE, &pmbusMasterBuffer[0], handle->count);

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

    nBytes = PMBus_getMasterData(PMBUSA_BASE, &pmbusMasterBuffer[0],
                                 pmbusStatus);

    //
    // Check the received byte
    //
    (nBytes == handle->count) ? handle->pass++ : handle->fail++;

    (pmbusMasterBuffer[0] == (PMBUS_CMD_VOUT_COMMAND ^ CMD_XOR_MASK_V1)) ? 
                             handle->pass++ : handle->fail++;

    (pmbusMasterBuffer[1] == (PMBUS_CMD_VOUT_COMMAND ^ CMD_XOR_MASK_V2)) ?
                             handle->pass++ : handle->fail++;

    (receivedPecValid == true) ? handle->pass++ : handle->fail++;

    //
    // Check that expected pass counter value is correct
    //
    if(handle->pass != 4U)
    {
        handle->fail++;
    }

    return;
}

//
// End of File
//
