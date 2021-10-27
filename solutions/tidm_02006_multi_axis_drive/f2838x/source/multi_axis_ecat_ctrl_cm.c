//#############################################################################
//
// FILE:   multi_axis_ecat_ctrl_cm.c
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
#include <stdbool.h>
#include <stdint.h>
#include "ecat_def.h"
#include "applInterface.h"

#define _F2838X_ETHERCAT_SOL_ 1
#include "multi_axis_ecat_ctrl_cm.h"
#undef _F2838X_ETHERCAT_SOL_

#define POWER_ON_DELAY_ECAT       8000000L

//
// IPC Buffer
//
#pragma DATA_SECTION(ipcCMToCPUDataBuffer, "MSGRAM_CM_TO_CPU1_ECAT")
ECAT_IPC_PutDataBuffer ipcCMToCPUDataBuffer;

#pragma DATA_SECTION(ipcCPUToCMDataBuffer, "MSGRAM_CPU1_TO_CM_ECAT")
ECAT_IPC_GetDataBuffer ipcCPUToCMDataBuffer;

ECAT_IPC_GetDataBuffer dataBufferFromCPU;
ECAT_IPC_PutDataBuffer dataBufferToCPU;

uint32_t mainLoopCntr = 0;
uint32_t backTicker = 0;
volatile uint16_t enableECATFlag;
//
// PDO_ResetOutputs - Resets the Output data to zero
//
void PDO_ResetOutputs(void)
{
    NodeMS0x7000.SynchAll = 0x0U;
    NodeMS0x7000.CommandNodeM = 0x0U;
    NodeMS0x7000.SpeedRefNodeM = 0x0L;
    NodeMS0x7000.PositionRefNodeM = 0x0L;

    Node1S0x7002.CommandNode1 = 0x0U;
    Node1S0x7002.SpeedRefNode1 = 0x0L;
    Node1S0x7002.PositionRefNode1 = 0x0L;

    Node2S0x7004.CommandNode2 = 0x0U;
    Node2S0x7004.SpeedRefNode2 = 0x0L;
    Node2S0x7004.PositionRefNode2 = 0x0L;

    Node3S0x7006.CommandNode3 = 0x0U;
    Node3S0x7006.SpeedRefNode3 = 0x0L;
    Node3S0x7006.PositionRefNode3 = 0x0L;

    Node4S0x7008.CommandNode4 = 0x0U;
    Node4S0x7008.SpeedRefNode4 = 0x0L;
    Node4S0x7008.PositionRefNode4 = 0x0L;
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
                // Status (16 bits Data)
                *(volatile UINT16 *)pTmpData = NodeMS0x6000.StatusNodeM;
                pTmpData += 2U;

                // Speed (32 bits Data)
                *(volatile INT32 *)pTmpData = NodeMS0x6000.SpeedNodeM;
                pTmpData += 4U;

                // Position (32 bits Data)
                *(volatile INT32 *)pTmpData = NodeMS0x6000.PositionNodeM;
                pTmpData += 4U;

                // Status (16 bits Data)
                *(volatile UINT16 *)pTmpData = Node1S0x6002.StatusNode1;
                pTmpData += 2U;

                // Speed (32 bits Data)
                *(volatile INT32 *)pTmpData = Node1S0x6002.SpeedNode1;
                pTmpData += 4U;

                // Position (32 bits Data)
                *(volatile INT32 *)pTmpData = Node1S0x6002.PositionNode1;
                pTmpData += 4U;

                // Status (16 bits Data)
                *(volatile UINT16 *)pTmpData = Node2S0x6004.StatusNode2;
                pTmpData += 2U;

                // Speed (32 bits Data)
                *(volatile INT32 *)pTmpData = Node2S0x6004.SpeedNode2;
                pTmpData += 4U;

                // Position (32 bits Data)
                *(volatile INT32 *)pTmpData = Node2S0x6004.PositionNode2;
                pTmpData += 4U;

                // Status (16 bits Data)
                *(volatile UINT16 *)pTmpData = Node3S0x6006.StatusNode3;
                pTmpData += 2U;

                // Speed (32 bits Data)
                *(volatile INT32 *)pTmpData = Node3S0x6006.SpeedNode3;
                pTmpData += 4U;

                // Position (32 bits Data)
                *(volatile INT32 *)pTmpData = Node3S0x6006.PositionNode3;
                pTmpData += 4U;

                // Status (16 bits Data)
                *(volatile UINT16 *)pTmpData = Node4S0x6008.StatusNode4;
                pTmpData += 2U;

                // Speed (32 bits Data)
                *(volatile INT32 *)pTmpData = Node4S0x6008.SpeedNode4;
                pTmpData += 4U;

                // Position (32 bits Data)
                *(volatile INT32 *)pTmpData = Node4S0x6008.PositionNode4;
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
                // Synchronize (16 bits Data)
                NodeMS0x7000.SynchAll = *(volatile UINT16 *)pTmpData;
                pTmpData += 2U;

                // DriveCommand (16 bits Data)
                NodeMS0x7000.CommandNodeM = *(volatile UINT16 *)pTmpData;
                pTmpData += 2U;

                // SpeedReference (32 bits Data)
                NodeMS0x7000.SpeedRefNodeM = *(volatile INT32 *)pTmpData;
                pTmpData += 4U;

                // PositionReference (32 bits Data)
                NodeMS0x7000.PositionRefNodeM = *(volatile INT32 *)pTmpData;
                pTmpData += 4U;

                // DriveCommand (16 bits Data)
                Node1S0x7002.CommandNode1 = *(volatile UINT16 *)pTmpData;
                pTmpData += 2U;

                // SpeedReference (32 bits Data)
                Node1S0x7002.SpeedRefNode1 = *(volatile INT32 *)pTmpData;
                pTmpData += 4U;

                // PositionReference (32 bits Data)
                Node1S0x7002.PositionRefNode1 = *(volatile INT32 *)pTmpData;
                pTmpData += 4U;

                // DriveCommand (16 bits Data)
                Node2S0x7004.CommandNode2 = *(volatile UINT16 *)pTmpData;
                pTmpData += 2U;

                // SpeedReference (32 bits Data)
                Node2S0x7004.SpeedRefNode2 = *(volatile INT32 *)pTmpData;
                pTmpData += 4U;

                // PositionReference (32 bits Data)
                Node2S0x7004.PositionRefNode2 = *(volatile INT32 *)pTmpData;
                pTmpData += 4U;

                // DriveCommand (16 bits Data)
                Node3S0x7006.CommandNode3 = *(volatile UINT16 *)pTmpData;
                pTmpData += 2U;

                // SpeedReference (32 bits Data)
                Node3S0x7006.SpeedRefNode3 = *(volatile INT32 *)pTmpData;
                pTmpData += 4U;

                // PositionReference (32 bits Data)
                Node3S0x7006.PositionRefNode3 = *(volatile INT32 *)pTmpData;
                pTmpData += 4U;

                // DriveCommand (16 bits Data)
                Node4S0x7008.CommandNode4 = *(volatile UINT16 *)pTmpData;
                pTmpData += 2U;

                // SpeedReference (32 bits Data)
                Node4S0x7008.SpeedRefNode4 = *(volatile INT32 *)pTmpData;
                pTmpData += 4U;

                // PositionReference (32 bits Data)
                Node4S0x7008.PositionRefNode4 = *(volatile INT32 *)pTmpData;
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
    ipcCMToCPUDataBuffer.ctrlSynchron =
            (CtrlSync_e)NodeMS0x7000.SynchAll;

    ipcCMToCPUDataBuffer.ctrlNode[0].command =
            (CtrlMode_e)NodeMS0x7000.CommandNodeM;
    ipcCMToCPUDataBuffer.ctrlNode[0].speedRef = NodeMS0x7000.SpeedRefNodeM;
    ipcCMToCPUDataBuffer.ctrlNode[0].positionRef = NodeMS0x7000.PositionRefNodeM;

    ipcCMToCPUDataBuffer.ctrlNode[1].command =
            (CtrlMode_e)Node1S0x7002.CommandNode1;
    ipcCMToCPUDataBuffer.ctrlNode[1].speedRef = Node1S0x7002.SpeedRefNode1;
    ipcCMToCPUDataBuffer.ctrlNode[1].positionRef = Node1S0x7002.PositionRefNode1;

    ipcCMToCPUDataBuffer.ctrlNode[2].command =
            (CtrlMode_e)Node2S0x7004.CommandNode2;
    ipcCMToCPUDataBuffer.ctrlNode[2].speedRef = Node2S0x7004.SpeedRefNode2;
    ipcCMToCPUDataBuffer.ctrlNode[2].positionRef = Node2S0x7004.PositionRefNode2;

    ipcCMToCPUDataBuffer.ctrlNode[3].command =
            (CtrlMode_e)Node3S0x7006.CommandNode3;
    ipcCMToCPUDataBuffer.ctrlNode[3].speedRef = Node3S0x7006.SpeedRefNode3;
    ipcCMToCPUDataBuffer.ctrlNode[3].positionRef = Node3S0x7006.PositionRefNode3;

    ipcCMToCPUDataBuffer.ctrlNode[4].command =
            (CtrlMode_e)Node4S0x7008.CommandNode4;
    ipcCMToCPUDataBuffer.ctrlNode[4].speedRef = Node4S0x7008.SpeedRefNode4;
    ipcCMToCPUDataBuffer.ctrlNode[4].positionRef = Node4S0x7008.PositionRefNode4;
    //
    // Read status data to send to EtherCAT Master
    //
    NodeMS0x6000.StatusNodeM = (uint16_t)ipcCPUToCMDataBuffer.statusNode[0].state;
    NodeMS0x6000.SpeedNodeM = ipcCPUToCMDataBuffer.statusNode[0].speed;
    NodeMS0x6000.PositionNodeM = ipcCPUToCMDataBuffer.statusNode[0].position;

    Node1S0x6002.StatusNode1 = (uint16_t)ipcCPUToCMDataBuffer.statusNode[1].state;
    Node1S0x6002.SpeedNode1 = ipcCPUToCMDataBuffer.statusNode[1].speed;
    Node1S0x6002.PositionNode1 = ipcCPUToCMDataBuffer.statusNode[1].position;

    Node2S0x6004.StatusNode2 = (uint16_t)ipcCPUToCMDataBuffer.statusNode[2].state;
    Node2S0x6004.SpeedNode2 = ipcCPUToCMDataBuffer.statusNode[2].speed;
    Node2S0x6004.PositionNode2 = ipcCPUToCMDataBuffer.statusNode[2].position;

    Node3S0x6006.StatusNode3 = (uint16_t)ipcCPUToCMDataBuffer.statusNode[3].state;
    Node3S0x6006.SpeedNode3 = ipcCPUToCMDataBuffer.statusNode[3].speed;
    Node3S0x6006.PositionNode3 = ipcCPUToCMDataBuffer.statusNode[3].position;

    Node4S0x6008.StatusNode4 = (uint16_t)ipcCPUToCMDataBuffer.statusNode[4].state;
    Node4S0x6008.SpeedNode4 = ipcCPUToCMDataBuffer.statusNode[4].speed;
    Node4S0x6008.PositionNode4 = ipcCPUToCMDataBuffer.statusNode[4].position;
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
// CM main() Echoback Function
//
int main(void)
{
    // Waiting for enable flag set
    while(enableECATFlag == false)
    {
        backTicker++;

        #ifdef _FLASH
        if(backTicker > POWER_ON_DELAY_ECAT)
        {
            enableECATFlag = true;
        }
        #else
        if(backTicker > 300000)
        {
            enableECATFlag = true;
        }
        #endif // _FLASH
    }

    //
    // Initialize device hardware and EtherCAT slave controller
    //
    HW_Init();

    //
    // Initialize Buffer
    //
    ipcCMToCPUDataBuffer.ctrlSynchron = CTRL_SYN_DISABLE;
    ipcCMToCPUDataBuffer.ctrlNode[0].command = CTRL_MODE_STOP;
    ipcCMToCPUDataBuffer.ctrlNode[0].speedRef = 0;
    ipcCMToCPUDataBuffer.ctrlNode[0].positionRef = 0;

    ipcCMToCPUDataBuffer.ctrlNode[1].command = CTRL_MODE_STOP;
    ipcCMToCPUDataBuffer.ctrlNode[1].speedRef = 0;
    ipcCMToCPUDataBuffer.ctrlNode[1].positionRef = 0;

    ipcCMToCPUDataBuffer.ctrlNode[2].command = CTRL_MODE_STOP;
    ipcCMToCPUDataBuffer.ctrlNode[2].speedRef = 0;
    ipcCMToCPUDataBuffer.ctrlNode[2].positionRef = 0;

    ipcCMToCPUDataBuffer.ctrlNode[3].command = CTRL_MODE_STOP;
    ipcCMToCPUDataBuffer.ctrlNode[3].speedRef = 0;
    ipcCMToCPUDataBuffer.ctrlNode[3].positionRef = 0;

    ipcCMToCPUDataBuffer.ctrlNode[4].command = CTRL_MODE_STOP;
    ipcCMToCPUDataBuffer.ctrlNode[4].speedRef = 0;
    ipcCMToCPUDataBuffer.ctrlNode[4].positionRef = 0;

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

        mainLoopCntr++;

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
