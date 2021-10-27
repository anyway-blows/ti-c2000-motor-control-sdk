//#############################################################################
//
// FILE:   fcl_f2838x_ecat_cm.c
//
// TITLE:  EtherCAT System Solution for F2838x CM
//
// This reference solution demonstrates usage of the EtherCAT stack using
// CAN-over-EtherCAT (CoE) mailbox protocol to perform transmission of
// data from the EtherCAT master to CPU1 and then status information
// from CPU1 to the EtherCAT master.
//
// This file initializes the device and EtherCAT hardware before starting
// the EtherCAT state machine handled by the slave stack code files. The
// slave stack code will call these "APPL" functions during state transitions
// as well as while running its main loop to copy data from the EtherCAT
// RAM to device RAM and from device RAM to EtherCAT RAM.
//
// Note
//  Slave Stack Code (SSC) tool must be used to generate the stack files
//  required by this solution.
//
// External Connections
//  - The controlCARD RJ45 port 0 is connected to PC running TwinCAT master
//  - If distributed clocks enabled, connect and observe SYNC0/1 signals on
//    GPIO127 and GPIO128
//
// Watch Variables
//  - TBD
//
//#############################################################################
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
//#############################################################################

//
// Included Files
//
#include <stdint.h>
#include "ecat_def.h"
#include "applInterface.h"

#define _F2838X_ETHERCAT_SOL_ 1
#include "fcl_f2838x_ecat_cm.h"
#undef _F2838X_ETHERCAT_SOL_

//
// IPC Buffer
//
#pragma DATA_SECTION(ipcCMToCPUDataBuffer, "MSGRAM_CM_TO_CPU1_ECAT")
ECAT_IPC_PutDataBuffer ipcCMToCPUDataBuffer;

#pragma DATA_SECTION(ipcCPUToCMDataBuffer, "MSGRAM_CPU1_TO_CM_ECAT")
ECAT_IPC_GetDataBuffer ipcCPUToCMDataBuffer;

//
// PDO_ResetOutputs - Resets the Output data to zero
//
void PDO_ResetOutputs(void)
{
   DriveCommand0x7000.DriveCommand = 0x0U;
   SpeedReference0x7002.SpeedReference = 0x0L;
   PositionReference0x7004.PositionReference = 0x0L;
}

//
// APPL_AckErrorInd - Called when error state was acknowledged by the master
//
void APPL_AckErrorInd(UINT16 stateTrans)
{
    //
    // No implementation for this application
    //
}

//
// APPL_StartMailboxHandler - Informs application of state transition from INIT
//                            to PREOP
//
UINT16 APPL_StartMailboxHandler(void)
{
    return(ALSTATUSCODE_NOERROR);
}

//
// APPL_StopMailboxHandler - Informs application of state transition from PREOP
//                           to INIT
//
UINT16 APPL_StopMailboxHandler(void)
{
    return(ALSTATUSCODE_NOERROR);
}

//
// APPL_StartInputHandler - Informs application of state transition from PREOP
//                          to SAFEOP
//
UINT16 APPL_StartInputHandler(UINT16 *pIntMask)
{
    return(ALSTATUSCODE_NOERROR);
}

//
// APPL_StopInputHandler - Informs application of state transition from SAFEOP
//                         PREOP
//
UINT16 APPL_StopInputHandler(void)
{
    return(ALSTATUSCODE_NOERROR);
}

//
// APPL_StartOutputHandler - Informs application of state transition from SAFEOP
//                           to OP
//
UINT16 APPL_StartOutputHandler(void)
{
    return(ALSTATUSCODE_NOERROR);
}

//
// APPL_StopOutputHandler - Informs application of state transition from OP to
//                          SAFEOP
//
UINT16 APPL_StopOutputHandler(void)
{
    //
    // Reset output data to zero
    //
    PDO_ResetOutputs();

    return(ALSTATUSCODE_NOERROR);
}

//
// APPL_GenerateMapping - Calculates the input and output process data sizes
//
UINT16 APPL_GenerateMapping(UINT16 *pInputSize, UINT16 *pOutputSize)
{
    UINT16 result = ALSTATUSCODE_NOERROR;
    UINT16 inputSize = 0U;
    UINT16 outputSize = 0U;
    UINT16 PDOAssignEntryCnt = 0U;
    OBJCONST TOBJECT OBJMEM * pPDO = NULL;
    UINT8 PDOSubindex0 = 0U;
    UINT8 *pPDOEntry = NULL;
    UINT8 PDOEntryCnt = 0U;

    //
    // Scan Object 0x1C12 (SyncManager 2)
    // (Sums up sizes of all SyncManager assigned RxPDOs subindexes)
    //
    for(PDOAssignEntryCnt = 0U; PDOAssignEntryCnt < sRxPDOassign.u16SubIndex0;
        PDOAssignEntryCnt++)
    {
        //
        // Get object handle for specified RxPDO assigned to SyncManager
        //
        pPDO = OBJ_GetObjectHandle(sRxPDOassign.aEntries[PDOAssignEntryCnt]);

        //
        // Confirm that Object isn't NULL before continuing
        //
        if(pPDO != NULL)
        {
            //
            // Get number of Object Entry subindexes
            //
            PDOSubindex0 = *((UINT8 *)pPDO->pVarPtr);

            //
            // Add up sizes of all the Object Entry subindexes
            //
            for(PDOEntryCnt = 0U; PDOEntryCnt < PDOSubindex0; PDOEntryCnt++)
            {
                pPDOEntry = ((UINT8 *)pPDO->pVarPtr +
                             (OBJ_GetEntryOffset((PDOEntryCnt + 1U),
                                                 pPDO) >> 3U));
                //
                // Increment the expected output size
                // depending on the mapped entry
                //
                outputSize += (UINT16)(*pPDOEntry);
            }
        }
        else
        {
            //
            // Assigned PDO was not found in object dictionary.
            // Return invalid mapping status.
            //
            outputSize = 0U;
            result = ALSTATUSCODE_INVALIDOUTPUTMAPPING;
            break;
        }
    }

    outputSize = (outputSize + 7U) >> 3U;

    //
    // Continue scanning of TXPDO if no error during RXPDO scanning
    //
    if(result == ALSTATUSCODE_NOERROR)
    {
        //
        // Scan Object 0x1C13 (SyncManager 3)
        // (Sums up sizes of all SyncManager assigned TXPDO subindexes)
        //
        for(PDOAssignEntryCnt = 0U;
            PDOAssignEntryCnt < sTxPDOassign.u16SubIndex0;
            PDOAssignEntryCnt++)
        {
            //
            // Get object handle for specified TxPDO assigned to SyncManager
            //
            pPDO =
             OBJ_GetObjectHandle(sTxPDOassign.aEntries[PDOAssignEntryCnt]);

            //
            // Confirm that Object isn't NULL before continuing
            //
            if(pPDO != NULL)
            {
                //
                // Get number of Object Entry subindexes
                //
                PDOSubindex0 = *((UINT8 *)pPDO->pVarPtr);

                //
                // Add up sizes of all the Object Entry subindexes
                //
                for(PDOEntryCnt = 0U; PDOEntryCnt < PDOSubindex0; PDOEntryCnt++)
                {
                    pPDOEntry = ((UINT8 *)pPDO->pVarPtr +
                                 (OBJ_GetEntryOffset((PDOEntryCnt + 1U),
                                                     pPDO) >> 3U));
                    //
                    // Increment the expected output size
                    // depending on the mapped entry
                    //
                    inputSize += (UINT16)(*pPDOEntry);
                }
            }
            else
            {
                //
                // Assigned PDO was not found in object dictionary.
                // Return invalid mapping status.
                //
                inputSize = 0U;
                result = ALSTATUSCODE_INVALIDINPUTMAPPING;
                break;
            }
        }
    }

    inputSize = (inputSize + 7U) >> 3U;

    //
    // Assign calculated sizes
    //
    *pInputSize = inputSize;
    *pOutputSize = outputSize;

    return(result);
}

//
// APPL_InputMapping - Copies the input data from local device memory to ESC
//                     memory
//
void APPL_InputMapping(UINT16 *pData)
{
    UINT16 j = 0U;
    UINT8 *pTmpData = (UINT8 *)pData;

    //
    // Loop through all TxPDO entries and write out data
    //
    for(j = 0U; j < sTxPDOassign.u16SubIndex0; j++)
    {
        switch(sTxPDOassign.aEntries[j])
        {
            //
            // TxPDO 0 (Data from ESC to master)
            //
            case 0x1A00U:
                //
                // SpeedStatus (32 bits Data)
                //
                *(volatile INT32 *)pTmpData =
                 SpeedStatus0x6000.SpeedStatus;
                pTmpData += 4U;

                //
                // PositionStatus (32 bits Data)
                //
                *(volatile INT32 *)pTmpData =
                 PositionStatus0x6002.PositionStatus;
                pTmpData += 4U;

                //
                // TorqueStatus (32 bits Data)
                //
                *(volatile INT32 *)pTmpData =
                 TorqueStatus0x6004.TorqueStatus;
                pTmpData += 4U;

                //
                // DriveStatus (16 bits Data)
                //
                *(volatile UINT16 *)pTmpData =
                 DriveStatus0x6006.DriveStatus;
                break;
        }
    }
}

//
// APPL_OutputMapping - Copies the output data from ESC memory to local device
//                      memory
//
void APPL_OutputMapping(UINT16 *pData)
{
    UINT16 j = 0U;
    UINT8 *pTmpData = (UINT8 *)pData;

    //
    // Loop through all RxPDO entries and read in data
    //
    for(j = 0U; j < sRxPDOassign.u16SubIndex0; j++)
    {
        switch(sRxPDOassign.aEntries[j])
        {
            //
            // RxPDO 0 (Data from Master to ESC)
            //
            case 0x1600U:
                //
                // DriveCommand (16 bits Data)
                //
                DriveCommand0x7000.DriveCommand =
                 *(volatile UINT16 *)pTmpData;
                pTmpData += 2U;

                //
                // SpeedReference (32 bits Data)
                //
                SpeedReference0x7002.SpeedReference =
                 *(volatile INT32 *)pTmpData;
                pTmpData += 4U;

                //
                // PositionReference (32 bits Data)
                //
                PositionReference0x7004.PositionReference =
                 *(volatile INT32 *)pTmpData;
                break;
        }
    }
}

//
// APPL_Application - Called as part of main stack loop.
//
void APPL_Application(void)
{
    //
    // Send data to CPU1
    //
    ipcCMToCPUDataBuffer.command = (ECAT_Command_e)DriveCommand0x7000.DriveCommand;
    ipcCMToCPUDataBuffer.speedRef = SpeedReference0x7002.SpeedReference;
    ipcCMToCPUDataBuffer.positionRef = PositionReference0x7004.PositionReference;


    //
    // Read status data to send to EtherCAT Master
    //
    SpeedStatus0x6000.SpeedStatus = ipcCPUToCMDataBuffer.speed;
    PositionStatus0x6002.PositionStatus = ipcCPUToCMDataBuffer.position;
    TorqueStatus0x6004.TorqueStatus = ipcCPUToCMDataBuffer.torque;
    DriveStatus0x6006.DriveStatus = (uint16_t)ipcCPUToCMDataBuffer.operationStatus;
}

#if EXPLICIT_DEVICE_ID
//
// APPL_GetDeviceID - Return explicit device ID for F2838x.CM
//
UINT16 APPL_GetDeviceID(void)
{
    return(0x5U);
}
#endif // EXPLICIT_DEVICE_ID

#if USE_DEFAULT_MAIN
//
// Main Echoback Function
//
int main(void)
{
    //
    // Initialize device hardware and EtherCAT slave controller
    //
    HW_Init();

    //
    // Initialize Buffer
    //
    ipcCMToCPUDataBuffer.command = ECAT_CMD_STOP;
    ipcCMToCPUDataBuffer.positionRef = 0x0L;
    ipcCMToCPUDataBuffer.speedRef = 0x0L;

    //
    // Initialize Slave Stack
    //
    MainInit();

    bRunApplication = TRUE;
    do
    {
        //
        // Run Slave Stack
        //
        MainLoop();
    } while(bRunApplication == TRUE);

    //
    // De-Initialize device and resources
    //
    HW_Release();

    return(0U);
}
#endif // USE_DEFAULT_MAIN

//
// End of File
//
