//###########################################################################
//
// FILE:   test_process_call.c
//
// TITLE:  Process Call (write word, then read word)
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
// PMBusMaster_initProcessCallTest
//
//*****************************************************************************
void PMBusMaster_initProcessCallTest(PMBus_TestHandle handle)
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
    // Enable the test
    //
#if ENABLE_TEST_14 == 1
    handle->enabled = true;
#else
    handle->enabled = false;
#endif //ENABLE_TEST_14 == 1
}

//*****************************************************************************
//
// PMBusMaster_runProcessCallTest
//
//*****************************************************************************
void PMBusMaster_runProcessCallTest(PMBus_TestHandle handle)
{
    //
    // Locals
    //
    uint16_t nBytes = 0U;

    //
    // Add command (index 0) and
    // word data to buffer (index 1 and 2)
    //
    pmbusMasterBuffer[0] = PMBUS_CMD_VIN_OV_FAULT_LIMIT;
    pmbusMasterBuffer[1] = TEST14_DATA_BYTE_1;
    pmbusMasterBuffer[2] = TEST14_DATA_BYTE_2;

    //
    // Do a Process Call (write word then read word)
    // Configure the master to enable PEC (processing), enable Write (by
    // omitting the read option from the configWord, you enable write),
    // enable command, enable process call (the master does the read word,
    // with the repeated start, right after the write word finishes)
    // Write, and then read 2 bytes
    //
    PMBus_configMaster(PMBUSA_BASE, SLAVE_ADDRESS, 2U,
                       (PMBUS_MASTER_ENABLE_PEC | PMBUS_MASTER_ENABLE_PRC_CALL |
                        PMBUS_MASTER_ENABLE_CMD));

    //
    // Send data
    //
    PMBus_putMasterData(PMBUSA_BASE, &pmbusMasterBuffer[0], 3U);

    //
    // Master automatically issues a repeated start (because of the process
    // call), and waits for the slave to respond with the word...when
    // the necessary bytes (specified in PMBMC.BYTE_COUNT) is received, master
    // NACKs last byte and generates an EOM
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
    }

    if((endOfMessage == true) && (masterDataAvailable == true))
    {
        //
        // EOM = 1 DATA_READY = 1
        //
        nBytes = PMBus_getMasterData(PMBUSA_BASE, &pmbusMasterBuffer[nBytes],
                                     pmbusStatus);

        //
        // No need to reset these flags, they will be reset at the start of the
        // next test
        //
    }

    //
    // Check that we got 2 bytes, correct data, and valid PEC
    //
    (nBytes == 2U) ? handle->pass++ : handle->fail++;
    (pmbusMasterBuffer[0] == (TEST14_DATA_BYTE_1 ^ TEST14_XOR_MASK)) ? 
                             handle->pass++ : handle->fail++;
    (pmbusMasterBuffer[1] == (TEST14_DATA_BYTE_2 ^ TEST14_XOR_MASK)) ? 
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
