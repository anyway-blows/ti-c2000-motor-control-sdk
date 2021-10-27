//#############################################################################
//
// FILE:    multi_axis_node2.h
//
// TITLE:   User settings
//
// Group:   C2000
//
// Target Family: F28004x/F28002x
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

#ifndef MULTI_AXIS_NODE2_H
#define MULTI_AXIS_NODE2_H

//
// Include project specific include files.
//
#include "device.h"
#include <math.h>

#include "multi_axis_nodes.h"

#define  CLB_COUNTER_0_MATCH1_NODE2  150

#define EPWM_SYNC_STOP_VALUE_NODE2      M_INV_PWM_TICKS
#define EPWM_SYNC_SHIFT_VALUE_NODE2     50U
#define EPWM_SYNC_EVENT_COUNT_NODE2     2U

#define EPWM_SYNC_SHIFT_U_NODE2         10U
#define EPWM_SYNC_SHIFT_V_NODE2         10U
#define EPWM_SYNC_SHIFT_W_NODE2         10U

#define EPWM_SYNC_SHIFT_T_NODE2         10U

//
// FSI
//
#ifdef _LAUNCHXL_F280049C
#define GPIO_PIN_FSI_TXCLK  7U                 // GPIO number for FSI TXCLK
#define GPIO_PIN_FSI_TX0    6U                 // GPIO number for FSI TX0
#define GPIO_PIN_FSI_RXCLK  33U                // GPIO number for FSI RXCLK
#define GPIO_PIN_FSI_RX0    12U                // GPIO number for FSI RX0

#define GPIO_CFG_FSI_TXCLK  GPIO_7_FSI_TXCLK   // "pinConfig" for FSI TXCLK
#define GPIO_CFG_FSI_TX0    GPIO_6_FSI_TX0     // "pinConfig" for FSI TX0
#define GPIO_CFG_FSI_RXCLK  GPIO_33_FSI_RXCLK  // "pinConfig" for FSI RXCLK
#define GPIO_CFG_FSI_RX0    GPIO_12_FSI_RX0    // "pinConfig" for FSI RX0
#else // _TMDSCNCD_F280049C
#define GPIO_PIN_FSI_TXCLK  27U                // GPIO number for FSI TXCLK
#define GPIO_PIN_FSI_TX0    26U                // GPIO number for FSI TX0
#define GPIO_PIN_FSI_RXCLK  13U                // GPIO number for FSI RXCLK
#define GPIO_PIN_FSI_RX0    12U                // GPIO number for FSI RX0

#define GPIO_CFG_FSI_TXCLK  GPIO_27_FSI_TXCLK  // "pinConfig" for FSI TXCLK
#define GPIO_CFG_FSI_TX0    GPIO_26_FSI_TX0    // "pinConfig" for FSI TX0
#define GPIO_CFG_FSI_RXCLK  GPIO_13_FSI_RXCLK  // "pinConfig" for FSI RXCLK
#define GPIO_CFG_FSI_RX0    GPIO_12_FSI_RX0    // "pinConfig" for FSI RX0
#endif // _LAUNCHXL_F280049C/

// node number
#define FSI_NODE_USE        FSI_SLAVE_N2
#define FSI_NODE_NEXT		FSI_SLAVE_N3

#endif  // end of MULTI_AXIS_NODE2_H definition

