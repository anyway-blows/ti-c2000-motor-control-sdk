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

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

//
//! \file   /solutions/servo_drive_with_can/common/include/communication.h
//!
//! \brief  header file to be included in all labs
//!         support for servo drive with can project
//!
//

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
//! \addtogroup COMMUNICATION
//! @{
//
//*****************************************************************************

// Included Files
#include "driverlib.h"
#include "device.h"
#include "math.h"
#include "types.h"


// **************************************************************************
// the defines_CAN_COMM_Obj_

typedef struct _CAN_COMM_Obj_
{

    uint32_t    canHandle;

    float32_t   speedConv_sf;
    float32_t   speedInv_sf;
    float32_t   speedRef_Hz;
    float32_t   speedFdb_Hz;

    uint16_t    rxMsgData[8];
    uint16_t    txMsgData[8];

    uint16_t    rxMsgCount;
    uint16_t    txMsgCount;

    uint16_t    waitTimeCnt;
    uint16_t    waitTimeDelay;

    uint16_t    errorFlag;        // for debug

    bool        flagRxDone;
    bool        flagTxDone;

    bool        flagCmdRxRun;
    bool        flagStateRxRun;

    bool        flagCmdEnable;
} CAN_COMM_Obj;

//Define handle:
typedef struct _CAN_COMM_Obj_ *CAN_COMM_Handle;

extern CAN_COMM_Handle canCommHandle;
extern CAN_COMM_Obj canComm;

// the typedefs
#define MSG_DATA_LENGTH         8
#define TX_MSG_OBJ_ID           1
#define RX_MSG_OBJ_ID           2

// **************************************************************************
// the globals

// **************************************************************************
// the function prototypes


extern CAN_COMM_Handle CAN_COMM_init(void *pMemory,const size_t numBytes);


//*****************************************************************************
//
// Close the Doxygen group.
//! @}
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

#endif // end of COMMUNICATION_H defines
