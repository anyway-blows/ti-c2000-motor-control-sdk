//#############################################################################
//
// FILE:    fcl_f2837x_enum.h
//
// TITLE:   define enumerations for FCL
//
// Group:   C2000
//
// Target Family: F2837x
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

#ifndef FCL_F2837X_ENUM_H
#define FCL_F2837X_ENUM_H

//
//! \brief Enumeration for State Machine typedef for motor QEP calibration
//
typedef enum
{
    QEP_CALIB_LOOPFLUSH = 0,
    QEP_CALIB_EQP1 = 1,
    QEP_CALIB_QEP2 = 2,
    QEP_CALIB_DONE = 3
} QEPCalibSM_e;

//
//! \brief Enumeration for Motor run/ stop command
//
typedef enum
{
    MOTOR_STOP = 0,
    MOTOR_RUN = 1
} MotorRunStop_e;

//
//! \brief Enumeration for Load motor selection/ reset
//
typedef enum
{
    LOAD_NONE = 0,
    LOAD_MOTOR1 = 1,
    LOAD_MOTOR2 = 2
} LoadMotor_e;

//
//! \brief Enumeration for FCL controller --> PI/ FCL
//
typedef enum
{
    CNTLR_CPI = 0,
    CNTLR_CMPLX = 1
} CurrentCntlr_e;

//
//! \brief Enumeration for SFRA test axis
//
typedef enum
{
    SFRA_TEST_D_AXIS = 0,
    SFRA_TEST_Q_AXIS = 1,
    SFRA_TEST_SPEEDLOOP = 2
} SFRATest_e;

//
//! \brief Enumeration for PWM update mode
//
typedef enum
{
    PWM_UPDATE_IMMEDIATE = 0,
    PWM_UPDATE_SHADOW = 1
} PWMUpdateType_e;

#endif // end of FCL_F2837X_ENUM_H definition
