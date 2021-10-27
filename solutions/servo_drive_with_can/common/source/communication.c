//#############################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:28 CDT 2021 $
// $Copyright:
// Copyright (C) 2017-2019 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

// *****************************************************************************
//! \file   /solutions/servo_drive_with_can/common/source/communication.c
//!
//! \brief  Communication with CAN for servo drive project
//!
//!
//!
// *****************************************************************************


// **************************************************************************
// the includes

#include "communication.h"

// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions

CAN_COMM_Handle CAN_COMM_init(void *pMemory, const size_t numBytes)
{
    CAN_COMM_Handle handle;
    CAN_COMM_Obj *obj;

    if(numBytes < sizeof(CAN_COMM_Obj))
    {
        return((CAN_COMM_Handle)NULL);
    }

    // assign the handle
    handle = (CAN_COMM_Handle)pMemory;

    // assign the object
    obj = (CAN_COMM_Obj *)handle;

    //assign CANA base
    obj->canHandle = CANA_BASE;

    obj->speedConv_sf = 0.1f;          // uint16->float(unit=0.1Hz)
    obj->speedInv_sf = 10.0f;          // float->uint16(unit=0.1Hz)

    obj->txMsgCount = 0;                // for debug
    obj->rxMsgCount = 0;                // for debug

    obj->waitTimeCnt = 1000;            // 1s/1000ms
    obj->waitTimeDelay = 2;             // 2ms

    obj->flagTxDone = true;             // To enable CAN
    obj->flagRxDone = false;

    obj->speedRef_Hz = 0.0f;            // 0Hz
    obj->speedFdb_Hz = 40.0f;           // 40Hz

    obj->flagCmdRxRun = false;
    obj->flagStateRxRun = false;

    obj->flagCmdEnable = false;

    return(handle);
} // end of CPU_USAGE_init() function

