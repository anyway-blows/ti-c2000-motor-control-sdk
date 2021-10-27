//###########################################################################
//
// FILE:   test_alert.c
//
// TITLE:  Alert Test
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
// PMBusMaster_initAlert
//
//*****************************************************************************
void PMBusMaster_initAlertTest(PMBus_TestHandle handle)
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
#if ENABLE_TEST_15 == 1
    handle->enabled = true;
#else
    handle->enabled = false;
#endif //ENABLE_TEST_15 == 1
}

//*****************************************************************************
//
// PMBusMaster_runAlertTest
//
//*****************************************************************************
void PMBusMaster_runAlertTest(PMBus_TestHandle handle)
{
    //
    // Increment fail, a DATA_REQUEST should set it back to 0
    //
    handle->fail++;

    //
    // Do a Quick command
    // Config the master enable Write (by omitting the read
    // option from the configWord, you enable write), 0 byte
    //
    PMBus_configMaster(PMBUSA_BASE, SLAVE_ADDRESS, 0U, PMBUS_ENABLE_WRITE);

    //
    // Wait for the EOM, and slave to ack the address before
    // reading data -- done in the ISR
    //
    while(endOfMessage == false)
    {
        if(masterDataRequested == true)
        {
            //
            // decrement fail if DATA_REQUEST was asserted
            //
            handle->fail--;

            //
            // Reset the flag
            //
            masterDataRequested = false;

            //
            // NACK the transaction to complete the quick command
            //
            PMBus_nackTransaction(PMBUSA_BASE);
        }
    }

    //
    // Reset the EOM flag
    //
    endOfMessage = false;

    //
    // Once the bus is free, if the slave NACK'd its a failure
    //
    (slaveAckReceived == true) ? handle->pass++ : handle->fail++;

    //
    // Loop till the alert signal is asserted
    //
    while(alertEdgeAsserted == false)
    {
    }

    //
    // Clear the spot where the alerting slave's address will go
    //
    memset(&pmbusMasterBuffer[0], 0U, handle->count * sizeof(uint16_t));

    //
    // An alert was received
    // Configure the master to set the address to the alert address,
    // enable Read and expect the alerting slave's address (no PEC)
    //
    PMBus_configMaster(PMBUSA_BASE, ALERT_RESPONSE_ADDRESS, 1U,
                       PMBUS_MASTER_ENABLE_READ);

    //
    // Wait for the EOM, and slave to ack the address before
    // reading data -- done in the ISR
    //
    while(endOfMessage == false)
    {
    }
    PMBus_getMasterData(PMBUSA_BASE, &pmbusMasterBuffer[0],
                        pmbusStatus);

    //
    // NOTE:  the alert line needs to be asserted for the slave to respond
    // to the alert response address, if you de-assert early the slave
    // ignores the alert response address line. The slave state machine will
    // de-assert when it detects an EOM, NACK and ALERT_EDGE
    //

    //
    // Check DATA_READY is asserted at the end of alert response
    //
    (masterDataAvailable == true) ? handle->pass++ : handle->fail++;

    //
    // Check that the alerting slave sent its address
    //
    (bytesReceived == 1U) ? handle->pass++ : handle->fail++;

    //
    // Slave sends its address in the 7 most significant bits with a
    // 0 or 1 in the LSb, which we dont care about
    //
    ((pmbusMasterBuffer[0] & SLAVE_ADDRESS_MASK) == (SLAVE_ADDRESS << 1U)) ?
                                                    handle->pass++ :
                                                    handle->fail++;

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
