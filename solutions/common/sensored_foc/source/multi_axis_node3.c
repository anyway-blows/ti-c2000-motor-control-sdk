//#############################################################################
//
// FILE:    multi_axis_node1.c
//
// TITLE:   multi-axis servo drive over FSI on the related kits
//
// Group:   C2000
//
// Target Family: F28004x/F28002x
//
//!
//!\b External \b Connections \n
//!  For the FSI daisy-chain topology external connections are required to
//!  be made between the devices in the chain. Each devices FSI TX pins need to
//!  be connected to the FSI RX pins of the next device in the chain (or ring).
//!  See below for external connections to include and GPIOs used:
//!
//!  External Connections Required:
//!     - FSIRX_CLK(F28388D)  to  FSITX_CLK (F280049C/F280025C)
//!     - FSIRX_RX0(F28388D)  to  FSITX_TX0 (F280049C/F280025C)
//!     - FSITX_CLK(F28388D)  to  FSIRX_CLK (F280049C/F280025C)
//!     - FSITX_TX0(F28388D)  to  FSIRX_RX0 (F280049C/F280025C)
//!
//!  F28002x ControlCard FSI Header GPIOs:
//!     - GPIO_27  ->    FSITX_CLK
//!     - GPIO_26  ->    FSITX_TX0
//!     - GPIO_13  ->    FSIRX_CLK
//!     - GPIO_12  ->    FSIRX_RX0
//!
//! \b Watch \b Variables \n
//!  - \b dataFrameCntr  Number of Data frames received back
//!  - \b error          Non zero for transmit/receive data mismatch
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
// includes
//
#include "multi_axis_node_main.h"
#include "multi_axis_fsi_shared.h"
#include "multi_axis_node3.h"

void FSI_setGPIOandNodeNum(HAL_Handle handle)
{
    fsiSlaveNodeUse  = FSI_NODE_USE;
    fsiSlaveNodeNext = FSI_NODE_NEXT;

    clbCounter0Match1Delay = CLB_COUNTER_0_MATCH1_NODE3;
    epwmSyncCmpStop = EPWM_SYNC_STOP_VALUE_NODE3;
    epwmSyncCmpDelay = EPWM_SYNC_SHIFT_VALUE_NODE3;
    epwmSyncPreScale = EPWM_SYNC_EVENT_COUNT_NODE3;

    phaseShiftValueU = EPWM_SYNC_SHIFT_U_NODE3;
    phaseShiftValueV = EPWM_SYNC_SHIFT_V_NODE3;
    phaseShiftValueW = EPWM_SYNC_SHIFT_W_NODE3;

    //
    // NOTE: External loopback, Modify GPIO settings as per setup
    // Only support one lane
    //
    GPIO_setPinConfig(GPIO_CFG_FSI_TXCLK);
    GPIO_setPinConfig(GPIO_CFG_FSI_TX0);

    GPIO_setPinConfig(GPIO_CFG_FSI_RXCLK);
    GPIO_setPinConfig(GPIO_CFG_FSI_RX0);

    //
    // Set RX GPIO to be asynchronous
    // (pass through without delay)
    // Default setting is to have 2 SYS_CLK cycles delay
    //
    GPIO_setQualificationMode(GPIO_PIN_FSI_RX0, GPIO_QUAL_ASYNC);
    GPIO_setQualificationMode(GPIO_PIN_FSI_RXCLK, GPIO_QUAL_ASYNC);


    return;
}

//
// End of Code
//
