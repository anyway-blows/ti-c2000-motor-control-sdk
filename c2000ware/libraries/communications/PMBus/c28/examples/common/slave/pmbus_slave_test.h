//###########################################################################
//
// FILE:   pmbus_slave_test.h
//
// TITLE:  Prototypes for the different slave command handlers
//
//###########################################################################
// $TI Release: C28x PMBus Communications Stack Library v1.03.00.00 $
// $Release Date: Fri Feb 12 19:16:58 IST 2021 $
// $Copyright: Copyright (C) 2015-2021 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################
#ifndef PMBUS_SLAVE_TEST_H
#define PMBUS_SLAVE_TEST_H

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//!
//! \defgroup PMBUS_SLAVE_TESTS PMBus Slave Mode Tests
//
//!
//! \ingroup PMBUS_SLAVE_TESTS
// @{
//
//*****************************************************************************

//
// Includes
//
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "pmbus_examples_setup.h"
#include "pmbus_stack_config.h"
#include "pmbus_stack_handler.h"

//
// Globals
//
extern PMBus_StackObject pmbusStackSlave;
extern PMBus_StackHandle pmbusStackSlaveHandle;
extern uint16_t pmbusSlaveBuffer[300U];

extern volatile uint16_t testsCompleted;
extern volatile uint16_t pass, fail;

//
// Function Prototypes
//

//*****************************************************************************
//
//! Send Byte Test Handler
//!
//! \param handle is the handle to the PMBus stack object
//!
//! \note Expected Pass Value: 4
//!
//! \return None.
//
//*****************************************************************************
void
PMBusSlave_sendByteTestHandler(PMBus_StackHandle handle);

//*****************************************************************************
//
//! Write Byte Test Handler
//!
//! \param handle is the handle to the PMBus stack object
//!
//! \note Expected Pass Value: 5
//!
//! \return None.
//
//*****************************************************************************
void
PMBusSlave_writeByteTestHandler(PMBus_StackHandle handle);

//*****************************************************************************
//
//! Write Word Test Handler
//!
//! \param handle is the handle to the PMBus stack object
//!
//! \note Expected Pass Value: 6
//!
//! \return None.
//
//*****************************************************************************
void
PMBusSlave_writeWordTestHandler(PMBus_StackHandle handle);

//*****************************************************************************
//
//! Block Write (255 bytes) Test Handler
//!
//! \param handle is the handle to the PMBus stack object
//!
//! \note Expected Pass Value: 260
//!
//! \return None.
//
//*****************************************************************************
void
PMBusSlave_blockWriteTestHandler(PMBus_StackHandle handle);

//*****************************************************************************
//
//! Receive Byte Test Handler
//!
//! \param handle is the handle to the PMBus stack object
//!
//! \note Expected Pass Value: 1
//!
//! \return None.
//
//*****************************************************************************
void
PMBusSlave_receiveByteTestHandler(PMBus_StackHandle handle);

//*****************************************************************************
//
//! Read Byte Test Handler
//!
//! \param handle is the handle to the PMBus stack object
//!
//! \note Expected Pass Value: 1
//!
//! \return None.
//
//*****************************************************************************
void
PMBusSlave_readByteTestHandler(PMBus_StackHandle handle);

//*****************************************************************************
//
//! Read Word Test Handler
//!
//! \param handle is the handle to the PMBus stack object
//!
//! \note Expected Pass Value: 1
//!
//! \return None.
//
//*****************************************************************************
void
PMBusSlave_readWordTestHandler(PMBus_StackHandle handle);

//*****************************************************************************
//
//! Block Read (255 bytes) Test Handler
//!
//! \param handle is the handle to the PMBus stack object
//!
//! \note Expected Pass Value: 1
//!
//! \return None.
//
//*****************************************************************************
void
PMBusSlave_blockReadTestHandler(PMBus_StackHandle handle);

//*****************************************************************************
//
//! Block Read (3 bytes) Test Handler
//!
//! \param handle is the handle to the PMBus stack object
//!
//! \note Expected Pass Value: 1
//!
//! \return None.
//
//*****************************************************************************
void
PMBusSlave_blockRead3BytesTestHandler(PMBus_StackHandle handle);

//*****************************************************************************
//
//! Block Write/Read/Process Call (255 bytes) Test Handler
//!
//! \param handle is the handle to the PMBus stack object
//!
//! \note Expected Pass Value: 259
//!
//! \return None.
//
//*****************************************************************************
void
PMBusSlave_blockWriteReadProcessCallTestHandler(PMBus_StackHandle handle);

//*****************************************************************************
//
//! Quick Command Test Handler
//!
//! \param handle is the handle to the PMBus stack object
//!
//! \note Expected Pass Value: 4
//!
//! \return None.
//
//*****************************************************************************
void
PMBusSlave_quickCommandTestHandler(PMBus_StackHandle handle);

//*****************************************************************************
//
//! Block Write (2 bytes) Test Handler
//!
//! \param handle is the handle to the PMBus stack object
//!
//! \note An attempted block write with 1 byte is a write byte, 2 bytes
//! a write word - the master does not put the byte count on the line.
//!
//! \note Make sure to run the write word test before this, as the original
//! write word Handler overwrites the handler to point to this function.
//!
//! \note Expected Pass Value: 6
//!
//! \return None.
//
//*****************************************************************************
void
PMBusSlave_blockWrite2BytesTestHandler(PMBus_StackHandle handle);

//*****************************************************************************
//
//! Block Write (3 bytes) Test Handler
//!
//! \param handle is the handle to the PMBus stack object
//!
//! \note An attempted block write with 1 byte is a write byte, 2 bytes
//! a write word - the master does not put the byte count on the line.
//!
//! \note Make sure to run the block write test before this, as the original
//! block write handler overwrites the handler to point to this function.
//!
//! \note Expected Pass Value: 8
//!
//! \return None.
//
//*****************************************************************************
void
PMBusSlave_blockWrite3BytesTestHandler(PMBus_StackHandle handle);

//*****************************************************************************
//
//! Process Call Test Handler
//!
//! \param handle is the handle to the PMBus stack object
//!
//! \note Expected Pass Value: 5
//!
//! \return None.
//
//*****************************************************************************
void
PMBusSlave_processCallTestHandler(PMBus_StackHandle handle);

//*****************************************************************************
//
//! Alert Test Handler
//!
//! \param handle is the handle to the PMBus stack object
//!
//! \note Expected Pass Value: 1
//!
//! \return None.
//
//*****************************************************************************
void
PMBusSlave_alertTestHandler(PMBus_StackHandle handle);

//*****************************************************************************
//
//! Alert (from 2nd slave) Test Handler
//!
//! \param handle is the handle to the PMBus stack object
//!
//! \note This test requires a 2nd PMBus slave on the network asserting
//!       the alert line.
//!
//! \note Expected Pass Value: 1
//!
//! \return None.
//
//*****************************************************************************
void
PMBusSlave_noAlertTestHandler(PMBus_StackHandle handle);

//*****************************************************************************
//
//! Group Command (slave 1st addressed) Test Handler
//!
//! \param handle is the handle to the PMBus stack object
//!
//! \note Expected Pass Value: 6
//!
//! \return None.
//
//*****************************************************************************
void
PMBusSlave_groupCommandTestHandler(PMBus_StackHandle handle);

//*****************************************************************************
//
//! Extended Write Byte Test Handler
//!
//! \param handle is the handle to the PMBus stack object
//!
//! \note Expected Pass Value: 6
//!
//! \return None.
//
//*****************************************************************************
void
PMBusSlave_extendedWriteByteTestHandler(PMBus_StackHandle handle);

//*****************************************************************************
//
//! Extended Write Word Test Handler
//!
//! \param handle is the handle to the PMBus stack object
//!
//! \note Expected Pass Value: 7
//!
//! \return None.
//
//*****************************************************************************
void
PMBusSlave_extendedWriteWordTestHandler(PMBus_StackHandle handle);

//*****************************************************************************
//
//! Extended Read Byte Test Handler
//!
//! \param handle is the handle to the PMBus stack object
//!
//! \note Expected Pass Value: 1
//!
//! \return None.
//
//*****************************************************************************
void
PMBusSlave_extendedReadByteTestHandler(PMBus_StackHandle handle);

//*****************************************************************************
//
//! ExtendedRead Word Test Handler
//!
//! \param handle is the handle to the PMBus stack object
//!
//! \note Expected Pass Value: 1
//!
//! \return None.
//
//*****************************************************************************
void
PMBusSlave_extendedReadWordTestHandler(PMBus_StackHandle handle);

//*****************************************************************************
//
// Close the Doxygen group.
// @}
//
//*****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif  // PMBUS_SLAVE_TEST_H
