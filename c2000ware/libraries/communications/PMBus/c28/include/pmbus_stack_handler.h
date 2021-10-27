//###########################################################################
//
// FILE:   pmbus_stack_Handler.h
//
// TITLE:  PMBUS Communications State Machine
//
//###########################################################################
// $TI Release: C28x PMBus Communications Stack Library v1.03.00.00 $
// $Release Date: Fri Feb 12 19:16:58 IST 2021 $
// $Copyright: Copyright (C) 2015-2021 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################
#ifndef PMBUS_STACK_HANDLER_H
#define PMBUS_STACK_HANDLER_H

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
//! \defgroup PMBUS_STACK_HANDLER PMBus State Machine Handler
//
//!
//! \ingroup PMBUS_STACK_HANDLER
// @{
//
//*****************************************************************************

//
// Includes
//
#include "pmbus_stack_assert.h"
#include "pmbus_stack_config.h"

//*****************************************************************************
//
//! PMBus Slave Stack State Machine Handler
//!
//! \param handle is the handle to the PMBus stack object
//!
//! This function implements the state machine of the PMBus in slave mode.
//! This handler is designed to operate within the PMBus interrupt service
//! routine (ISR) triggered by the following interrupts:
//! - Data Ready (Read buffer is full)
//! - Data Request (Master has requested data)
//! - EOM (Master signals an end of a block message)
//!
//! \note The handler must be called in the PMBus ISR only
//!
//! \return None.
//
//*****************************************************************************
extern void
PMBusStack_slaveStateHandler(PMBus_StackHandle handle);

//*****************************************************************************
//
//! PMBus Slave Idle State Handler
//!
//! \param handle is the handle to the PMBus stack object
//!
//! This function handles the idle state in the slave state machine.
//!
//! \return None.
//
//*****************************************************************************
extern void
PMBusStack_slaveIdleStateHandler(PMBus_StackHandle handle);

//*****************************************************************************
//
//! PMBus Slave Receive Byte Wait-for-EOM State Handler
//!
//! \param handle is the handle to the PMBus stack object
//!
//! This function handles the state in the slave state machine that is entered
//! when a receive byte request is active and slave is waiting for
//! end-of-message.
//!
//! \return None.
//
//*****************************************************************************
extern void
PMBusStack_slaveReceiveByteWaitForEOMStateHandler(PMBus_StackHandle handle);

//*****************************************************************************
//
//! PMBus Slave Read Block State Handler
//!
//! \param handle is the handle to the PMBus stack object
//!
//! This function handles the state in the slave state machine that is entered
//! when a read block command is used.
//!
//! \return None.
//
//*****************************************************************************
extern void
PMBusStack_slaveReadBlockStateHandler(PMBus_StackHandle handle);

//*****************************************************************************
//
//! PMBus Slave Read/Wait for EOM State Handler
//!
//! \param handle is the handle to the PMBus stack object
//!
//! This function handles the state in the slave state machine that is entered
//! when reading/waiting for the end-of-message.
//!
//! \return None.
//
//*****************************************************************************
extern void
PMBusStack_slaveReadWaitForEOMStateHandler(PMBus_StackHandle handle);

//*****************************************************************************
//
//! PMBus Slave Block Write or Process Call State Handler
//!
//! \param handle is the handle to the PMBus stack object
//!
//! This function handles the state in the slave state machine that is entered
//! when a block write or process call commands are used.
//!
//! \return None.
//
//*****************************************************************************
extern void
PMBusStack_slaveBlockWriteOrProcessCallStateHandler(PMBus_StackHandle handle);

//*****************************************************************************
//
//! PMBus Slave Extended Read/Write Byte/Word State Handler
//!
//! \param handle is the handle to the PMBus stack object
//!
//! This function handles the state in the slave state machine that is entered
//! when extended commands are used. These include extended read byte,
//! read word, write byte, and write word transactions.
//!
//! \return None.
//
//*****************************************************************************
extern void
PMBusStack_slaveExtendedCommandStateHandler(PMBus_StackHandle handle);

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
#endif // extern "C"

#endif // PMBUS_STACK_HANDLER_H
