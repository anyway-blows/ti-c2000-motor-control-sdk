//#############################################################################
//
// FILE:    motor_ctrl_settings.h
//
// TITLE:   User settings
//
// Group:   C2000
//
// Target Family: F2838x
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
//! \file  solutions/multi_axis_drive/f2838x/include/motor_ctrl_settings.h
//! \brief header file to be included in all labs
//!
//

#ifndef MOTOR_CTRL_SETTINGS_H
#define MOTOR_CTRL_SETTINGS_H

//
//! \defgroup MASTER
//! @{
//

//
// Include project specific include files.
//

//
// Following is the list of the Build Level choices.
//
#define  FCL_LEVEL1   1     // Verify HAL
#define  FCL_LEVEL2   2     // Verify IPC for Node_M with speed loop
#define  FCL_LEVEL3   3     // Verify EtherCAT & IPC for Node_M w/o control
#define  FCL_LEVEL4   4     // Verify EtherCAT & IPC with speed loop for Node_M
#define  FCL_LEVEL5   5     // Verify FSI
#define  FCL_LEVEL6   6     // Verify torque current control over FSI (**)
#define  FCL_LEVEL7   7     // Verify speed loop over FSI & IPC
#define  FCL_LEVEL8   8     // Verify position loop over FSI & IPC
#define  FCL_LEVEL9   9     // SFRA verify slaves control bandwidth
#define  FCL_LEVEL10  10    // Verify EtherCAT, IPC & FSI w/o control loop
#define  FCL_LEVEL11  11    // Speed/position loop over EtherCAT, IPC and FSI

//
// Here below, pick position and speed loop controller option
//
#define  SPD_PID_CNTLR      1
#define  SPD_DCL_CNTLR      2

//
// User can select choices from available control configurations
//
#define  BUILDLEVEL          FCL_LEVEL7         // 2, 4, 7 & 11 for demo
#define  SPD_CNTLR           SPD_PID_CNTLR      // SPD_DCL_CNTLR     //

#ifndef BUILDLEVEL
#error  Critical: BUILDLEVEL must be defined !!
#endif  // BUILDLEVEL

#ifndef SPD_CNTLR
#error  Critical: SPD_CNTLR must be defined !!
#endif  // SPD_CNTLR

#ifndef PI
#define PI 3.14159265358979
#endif

//
// Close the Doxygen group.
//! @} //defgroup MASTER
//

#endif  // end of MOTOR_CTRL_SETTINGS_H definition

// end of file






















