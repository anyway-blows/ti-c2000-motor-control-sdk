//###########################################################################
//
// FILE:   pmbus_stack_compatibility.h
//
// TITLE:  PMBus Stack Library Compatibility Header
//
// IMPORTANT: This re-maps the legacy F28004x PMBus stack library API, ENUM, 
//            Struct, etc names to the new names of the PMBus library
//            (Legacy PMBus library refers to the version released in
//             C2000Ware v2.00.00.03 and older)
//
// Legacy Users: If you've previously developed using the legacy PMBus library,
//               include this header in your project to map the legacy names
//               to the new library names           
//
// New Users: If this is your first time developing with this library,
//            DON'T include this file.
//
//###########################################################################
// $TI Release: C28x PMBus Communications Stack Library v1.03.00.00 $
// $Release Date: Fri Feb 12 19:16:58 IST 2021 $
// $Copyright: Copyright (C) 2015-2021 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################
#ifndef PMBUS_STACK_COMPATIBILITY_H
#define PMBUS_STACK_COMPATIBILITY_H

//
// Library API re-defining
//
#define PMBus_Stack_assertionFailed                       PMBusStack_assertionFailed
#define PMBus_Stack_errorHandler                          PMBusStack_errorHandler
#define PMBus_Stack_initModule                            PMBusStack_initModule
#define PMBus_Stack_defaultTransactionHandler             PMBusStack_defaultTransactionHandler
#define PMBus_Stack_Obj_setModuleBase                     PMBusStackObject_setModuleBase
#define PMBus_Stack_Obj_getModuleBase                     PMBusStackObject_getModuleBase
#define PMBus_Stack_Obj_setModuleStatus                   PMBusStackObject_setModuleStatus
#define PMBus_Stack_Obj_getModuleStatus                   PMBusStackObject_getModuleStatus
#define PMBus_Stack_Obj_setMode                           PMBusStackObject_setMode
#define PMBus_Stack_Obj_getMode                           PMBusStackObject_getMode
#define PMBus_Stack_Obj_setAddress                        PMBusStackObject_setSlaveAddress
#define PMBus_Stack_Obj_getAddress                        PMBusStackObject_getSlaveAddress
#define PMBus_Stack_Obj_setAddressMask                    PMBusStackObject_setSlaveAddressMask
#define PMBus_Stack_Obj_getAddressMask                    PMBusStackObject_getSlaveAddressMask
#define PMBus_Stack_Obj_setCurrentState                   PMBusStackObject_setCurrentState
#define PMBus_Stack_Obj_getCurrentState                   PMBusStackObject_getCurrentState
#define PMBus_Stack_Obj_setNextState                      PMBusStackObject_setNextState
#define PMBus_Stack_Obj_getNextState                      PMBusStackObject_getNextState
#define PMBus_Stack_Obj_setPtrBuffer                      PMBusStackObject_setBufferPointer
#define PMBus_Stack_Obj_getPtrBuffer                      PMBusStackObject_getBufferPointer
#define PMBus_Stack_Obj_setCurrPtr                        PMBusStackObject_setCurrentPositionPointer
#define PMBus_Stack_Obj_getCurrPtr                        PMBusStackObject_getCurrentPositionPointer
#define PMBus_Stack_Obj_setNBytes                         PMBusStackObject_setNumOfBytes
#define PMBus_Stack_Obj_getNBytes                         PMBusStackObject_getNumOfBytes
#define PMBus_Stack_Obj_setPECValid                       PMBusStackObject_setPECValidity
#define PMBus_Stack_Obj_getPECValid                       PMBusStackObject_isPECValid
#define PMBus_Stack_Obj_setTransaction                    PMBusStackObject_setTransactionType
#define PMBus_Stack_Obj_getTransaction                    PMBusStackObject_getTransactionType
#define PMBus_Stack_Obj_setTransactionHandler             PMBusStackObject_setTransactionHandler
#define PMBus_Stack_Obj_getTransactionHandler             PMBusStackObject_getTransactionHandler
#define PMBus_Stack_doesCommandMatchTransaction           PMBusStackObject_isCommandAndTransactionValid

#define PMBus_Stack_slaveHandler                          PMBusStack_slaveStateHandler
#define PMBus_Stack_slaveIdleHandler                      PMBusStack_slaveIdleStateHandler
#define PMBus_Stack_slaveReceiveByteWaitForEomHandler     PMBusStack_slaveReceiveByteWaitForEOMStateHandler
#define PMBus_Stack_slaveReadBlockHandler                 PMBusStack_slaveReadBlockStateHandler
#define PMBus_Stack_slaveReadWaitForEOMHandler            PMBusStack_slaveReadWaitForEOMStateHandler
#define PMBus_Stack_slaveBlockWriteOrProcessCallHandler   PMBusStack_slaveBlockWriteOrProcessCallStateHandler
#define PMBUS_STACK_extendedCommandHandler                PMBusStack_slaveExtendedCommandStateHandler

//
// ENUM, Structs, etc re-defining
//
#define PMBus_Stack_mode                                  PMBus_StackMode
#define PMBus_Stack_State                                 PMBus_StackState
#define PMBus_Stack_Obj                                   PMBus_StackObject
#define PMBus_Transaction_Obj                             PMBus_TransactionObject
#define PMBus_Stack_Handle                                PMBus_StackHandle
#define PMBus_Transaction_Obj_u                           PMBus_TransactionObjectUnion
#define PMBus_Stack_commandTransactionMap                 PMBusStack_commandTransactionMap
#define PMBUS_STACK_Slave                                 PMBusStackSlave

#endif // PMBUS_STACK_COMPATIBILITY_H
