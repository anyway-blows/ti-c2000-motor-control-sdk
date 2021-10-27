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
//! \file   /solutions/fast_uni_lab/common/source/communication.c
//!
//! \brief  This project is used to implement motor control with FAST, eSMO
//!         Encoder, and Hall sensors based sensored/sensorless-FOC.
//!         Supports multiple TI EVM board with F28002x
//!
// *****************************************************************************

//
// include the related header files
//
#include "sys_main.h"

#include "communication.h"

//
// Defines
//

#if defined(CMD_CAN_EN)

#pragma CODE_SECTION(canaISR, ".TI.ramfunc");

// **************************************************************************
// the globals

volatile CANCOM_Obj canComVars;

// **************************************************************************
// the functions
void HAL_setupCANA(HAL_Handle halHandle)
{
    HAL_Obj *obj = (HAL_Obj *)halHandle;

    // Initialize the CAN controller
    CAN_initModule(obj->canHandle);

    // Set up the CAN bus bit rate to 200kHz
    // Refer to the Driver Library User Guide for information on how to set
    // tighter timing control. Additionally, consult the device data sheet
    // for more information about the CAN module clocking.
    CAN_setBitRate(obj->canHandle, DEVICE_SYSCLK_FREQ, 500000, 16);

    // Initialize the transmit message object used for sending CAN messages.
    // Message Object Parameters:
    //      Message Object ID Number: 1
    //      Message Identifier: 0x1
    //      Message Frame: Standard
    //      Message Type: Transmit
    //      Message ID Mask: 0x0
    //      Message Object Flags: Transmit Interrupt
    //      Message Data Length: 8 Bytes
    CAN_setupMessageObject(CANA_BASE, TX_MSG_OBJ_ID, 0x1, CAN_MSG_FRAME_STD,
                           CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_TX_INT_ENABLE,
                           MSG_DATA_LENGTH);

    // Initialize the receive message object used for receiving CAN messages.
    // Message Object Parameters:
    //      Message Object ID Number: 2
    //      Message Identifier: 0x1
    //      Message Frame: Standard
    //      Message Type: Receive
    //      Message ID Mask: 0x0
    //      Message Object Flags: Receive Interrupt
    //      Message Data Length: 8 Bytes
    CAN_setupMessageObject(obj->canHandle, RX_MSG_OBJ_ID, 0x1, CAN_MSG_FRAME_STD,
                           CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE,
                           MSG_DATA_LENGTH);

    // Start CAN module operations
    CAN_startModule(obj->canHandle);

    return;
}  // end of DRV_setupSci() function



void HAL_enableCANInts(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    // Enable CAN test mode with external loopback
//     CAN_enableTestMode(CANA_BASE, CAN_TEST_EXL);    // Only for debug

    // Enable interrupts on the CAN peripheral.
    CAN_enableInterrupt(obj->canHandle, CAN_INT_IE0 | CAN_INT_ERROR |
                        CAN_INT_STATUS);


    // enable the PIE interrupts associated with the CAN interrupts
    Interrupt_enable(INT_CANA0);


    CAN_enableGlobalInterrupt(obj->canHandle, CAN_GLOBAL_INT_CANINT0);

    // enable the cpu interrupt for CAN interrupts
    Interrupt_enableInCPU(INTERRUPT_CPU_INT9);

    return;
} // end of HAL_enableCANInts() function

//! \brief      Initializes CAN
//! \param[in]  N/A
void initCANCOM(void)
{
    GPIO_setPinConfig(COM_CANRX_GPIO_PIN_CONFIG);
    GPIO_setDirectionMode(COM_CANRX_GPIO, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(COM_CANRX_GPIO, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(COM_CANTX_GPIO_PIN_CONFIG);
    GPIO_setDirectionMode(COM_CANTX_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(COM_CANTX_GPIO, GPIO_QUAL_ASYNC);

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    // This registers the interrupt handler in PIE vector table.
    Interrupt_register(INT_CANA0, &canaISR);

    canComVars.speedConv_sf = 0.1f;          // uint16->float(unit=0.1Hz)
    canComVars.speedInv_sf = 10.0f;          // float->uint16(unit=0.1Hz)
    canComVars.currentConv_sf = 0.01f;       // uint16->float(unit=0.01A)
    canComVars.currentInv_sf = 100.0f;       // float->uint16(unit=0.01A)

    canComVars.txMsgCount = 0;                // for debug
    canComVars.rxMsgCount = 0;                // for debug

    canComVars.waitTimeCnt = 1000;            // 1s/1000ms
    canComVars.waitTimeDelay = 2;             // 2ms

    canComVars.flagTxDone = true;             // To enable CAN
    canComVars.flagRxDone = false;

    canComVars.speedRef_Hz = 0.0f;            // 0Hz
    canComVars.speedSet_Hz = 40.0f;           // 40Hz

    canComVars.flagCmdTxRun = false;
    canComVars.flagCmdRxRun = false;

    return;
} // end of HAL_initCANInt() function


void updateCANCmdFreq(MOTOR_Handle handle)
{
    MOTOR_Vars_t *objMtr = (MOTOR_Vars_t *)handle;
    uint16_t canData = 0;

    if(canComVars.flagRxDone == true)
    {
        canComVars.flagCmdRxRun = (bool)(canComVars.rxMsgData[0]);
        canComVars.motorStateRx = (MOTOR_Status_e)(canComVars.rxMsgData[1]);

        canComVars.speedRef_Hz = ((float32_t)((canComVars.rxMsgData[2]<<8) +
                canComVars.rxMsgData[3])) * canComVars.speedConv_sf;

        canComVars.speedRx_Hz = ((float32_t)((canComVars.rxMsgData[4]<<8) +
                canComVars.rxMsgData[5])) * canComVars.speedConv_sf;

        canComVars.IqRx_A = ((float32_t)((canComVars.rxMsgData[6]<<8) +
                canComVars.rxMsgData[7])) * canComVars.currentConv_sf;

        canComVars.flagTxDone = true;
        canComVars.flagRxDone = false;
    }

    if((canComVars.flagTxDone == true) && (canComVars.waitTimeCnt == 0))
    {
        canComVars.txMsgData[0] = (uint16_t)(canComVars.flagCmdTxRun);
        canComVars.txMsgData[1] = (uint16_t)(objMtr->motorState);

        canData = (uint16_t)(canComVars.speedSet_Hz * canComVars.speedInv_sf);
        canComVars.txMsgData[2] = (canData>>8) & 0x00FF;
        canComVars.txMsgData[3] = canData & 0x00FF;


        canData = (uint16_t)(objMtr->speedAbs_Hz * canComVars.speedInv_sf);
        canComVars.txMsgData[4] = (canData>>8) & 0x00FF;
        canComVars.txMsgData[5] = canData & 0x00FF;

        canData = (uint16_t)(objMtr->Idq_in_A.value[1] * canComVars.currentInv_sf);

        canComVars.txMsgData[6] = (canData>>8) & 0x00FF;
        canComVars.txMsgData[7] = canData & 0x00FF;

        CAN_sendMessage(CANA_BASE, TX_MSG_OBJ_ID, MSG_DATA_LENGTH,
                        (uint16_t *)(&canComVars.txMsgData[0]));

        canComVars.waitTimeCnt = canComVars.waitTimeDelay;
        canComVars.flagTxDone = false;
    }

    if(canComVars.waitTimeCnt > 0)
    {
        canComVars.waitTimeCnt--;
    }

    return;
}

__interrupt void canaISR(void)
{
    uint32_t status;

    // Read the CAN interrupt status to find the cause of the interrupt
    status = CAN_getInterruptCause(halHandle->canHandle);


    // If the cause is a controller status interrupt, then get the status
    if(status == CAN_INT_INT0ID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.
        status = CAN_getStatus(halHandle->canHandle);

        // Check to see if an error occurred.
        if(((status  & ~(CAN_STATUS_TXOK | CAN_STATUS_RXOK)) != 7) &&
           ((status  & ~(CAN_STATUS_TXOK | CAN_STATUS_RXOK)) != 0))
        {
            // Set a flag to indicate some errors may have occurred.
            canComVars.errorFlag = 1;
        }

    }
    // Check if the cause is the transmit message object 1
    else if(status == TX_MSG_OBJ_ID)
    {
        //
        // Getting to this point means that the TX interrupt occurred on
        // message object 1, and the message TX is complete.  Clear the
        // message object interrupt.
        //
        CAN_clearInterruptStatus(CANA_BASE, TX_MSG_OBJ_ID);

        // Increment a counter to keep track of how many messages have been
        // sent.  In a real application this could be used to set flags to
        // indicate when a message is sent.
        canComVars.txMsgCount++;

        // Since the message was sent, clear any error flags.
        canComVars.errorFlag = 0;
    }

    // Check if the cause is the receive message object 2
    else if(status == RX_MSG_OBJ_ID)
    {
        //
        // Get the received message
        //
        CAN_readMessage(halHandle->canHandle, RX_MSG_OBJ_ID,
                        (uint16_t *)(&canComVars.rxMsgData[0]));

        // Getting to this point means that the RX interrupt occurred on
        // message object 2, and the message RX is complete.  Clear the
        // message object interrupt.
        CAN_clearInterruptStatus(halHandle->canHandle, RX_MSG_OBJ_ID);

        canComVars.rxMsgCount++;
        canComVars.flagRxDone = true;

        // Since the message was received, clear any error flags.
        canComVars. errorFlag = 0;
    }
    // If something unexpected caused the interrupt, this would handle it.
    else
    {
        // Spurious interrupt handling can go here.
    }

    // Clear the global interrupt flag for the CAN interrupt line
    CAN_clearGlobalInterruptStatus(halHandle->canHandle, CAN_GLOBAL_INT_CANINT0);

    // Acknowledge this interrupt located in group 9
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);

    return;
}

#endif // CMD_CAN_EN
//
//-- end of this file ----------------------------------------------------------
//
