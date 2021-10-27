//###########################################################################
//
// FILE:   fcl_f2838x_ecat_cm.h
//
// TITLE:  F2838x CM System Solution Header
//
//###########################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:28 CDT 2021 $
// $Copyright:
// Copyright (C) 2017-2021 Texas Instruments Incorporated - http://www.ti.com/
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
//###########################################################################

#ifndef FCL_F2838X_ECAT_CM_H
#define FCL_F2838X_ECAT_CM_H

//
// Included Files
//
#include "ecat_def.h"
#include "ecatappl.h"
#include "f2838x_cm_data_objects.h"

#endif // FCL_F2838X_ECAT_CM_H

//
// Defines
//
#if defined(_F2838X_ETHERCAT_SOL_) && (_F2838X_ETHERCAT_SOL_ == 1)
    #define PROTO
#else
    #define PROTO extern
#endif

#define BIT_MASK  0x1U
#define BYTE_MASK 0xFFU

//
// Connected drive command(ECAT) settings
//
typedef enum
{
    ECAT_CMD_STOP,
    ECAT_CMD_SPD_MODE,
    ECAT_CMD_POS_MODE
} ECAT_Command_e;

//
// IPC Struct
//
typedef struct
{
    ECAT_Command_e command;
    int32_t speedRef;
    int32_t positionRef;
}ECAT_IPC_PutDataBuffer;

typedef struct
{
    int32_t speed;
    int32_t position;
    int32_t torque;
    ECAT_Command_e operationStatus;
}ECAT_IPC_GetDataBuffer;


//
// Function Prototypes
//

//*****************************************************************************
//
// Mainloop Application Function
//
// This function for this application loops back the output data to the input
// data for the EtherCAT Master to read.
//
// Returns - None
//
//*****************************************************************************
PROTO void APPL_Application(void);

//*****************************************************************************
//
// Get Device ID
//
// This function returns the explicit device ID.
//
// Returns - Explicit device ID
//
//*****************************************************************************
#if EXPLICIT_DEVICE_ID
PROTO UINT16 APPL_GetDeviceID(void);
#endif

//*****************************************************************************
//
// Acknowledge Error Function
//
// stateTrans indicates the current state transition
//
// This function for this application performs no action however it will get
// called when the master acknowledges an error.
//
// Returns - None
//
//*****************************************************************************
PROTO void APPL_AckErrorInd(UINT16 stateTrans);

//*****************************************************************************
//
// Start Mailbox Handler
//
// This function is called during the state transition from INIT to PREOP and
// performs no action for this application.
//
// Returns - Application Layer Status Code - ALSTATUSCODE_NOERROR
//
//*****************************************************************************
PROTO UINT16 APPL_StartMailboxHandler(void);

//*****************************************************************************
//
// Stop Mailbox Handler
//
// This function is called during the state transition from PREOP to INIT and
// performs no action for this application.
//
// Returns - Application Layer Status Code - ALSTATUSCODE_NOERROR
//
//*****************************************************************************
PROTO UINT16 APPL_StopMailboxHandler(void);

//*****************************************************************************
//
// Start Input Handler
//
// pIntMask is current value of the AL Event Mask
//
// This function is called during the state transition from PREOP to SAFEOP and
// performs no action for this application.
//
// Returns - Application Layer Status Code - ALSTATUSCODE_NOERROR
//
//*****************************************************************************
PROTO UINT16 APPL_StartInputHandler(UINT16 *pIntMask);

//*****************************************************************************
//
// Stop Input Handler
//
// This function is called during the state transition from SAFEOP to PREOP and
// performs no action for this application.
//
// Returns - Application Layer Status Code - ALSTATUSCODE_NOERROR
//
//*****************************************************************************
PROTO UINT16 APPL_StopInputHandler(void);

//*****************************************************************************
//
// Start Output Handler
//
// This function is called during the state transition from SAFEOP to OP and
// performs no action for this application.
//
// Returns - Application Layer Status Code - ALSTATUSCODE_NOERROR
//
//*****************************************************************************
PROTO UINT16 APPL_StartOutputHandler(void);

//*****************************************************************************
//
// Stop Output Handler
//
// This function is called during the state transition from OP to SAFEOP and
// performs no action for this application.
//
// Returns - Application Layer Status Code - ALSTATUSCODE_NOERROR
//
//*****************************************************************************
PROTO UINT16 APPL_StopOutputHandler(void);

//*****************************************************************************
//
// Calculate the Process Data Sizes
//
// pInputSize is a pointer to store the input process data size
// pOutputSize is a pointer to store the output process data size
//
// This function is called during the state transition from PREOP to SAFEOP and
// calculates the process data size in bytes for both input and output process
// data.
//
// Returns - Application Layer Status Codes
//           ALSTATUSCODE_NOERROR - No error
//           ALSTATUSCODE_INVALIDOUTPUTMAPPING - Output mapping error
//           ALSTATUSCODE_INVALIDINPUTMAPPING - Input mapping error
//
//*****************************************************************************
PROTO UINT16 APPL_GenerateMapping(UINT16 *pInputSize, UINT16 *pOutputSize);

//*****************************************************************************
//
// Maps the Input Process Data to Generic Stack
//
// pIntMask is the pointer to the input process data destination
//
// This function is called after the application call to map the input process
// data from local device memory to the generic stack (ESC memory).
//
// Returns - None
//
//*****************************************************************************
PROTO void APPL_InputMapping(UINT16 *pData);

//*****************************************************************************
//
// Maps the Output Process Data to Generic Stack
//
// pIntMask is the pointer to the output process data
//
// This function is called before the application call to copy the output
// process data from the generic stack (ESC memory) to local device memory.
//
// Returns - None
//
//*****************************************************************************
PROTO void APPL_OutputMapping(UINT16 *pData);

#undef PROTO

//
// End of File
//
