//#############################################################################
//
// FILE:    motor_drive_settings.h
//
// TITLE:   User settings
//
// Group:   C2000
//
// Target Family: F2838x/F2837x/F28004x/F28002x
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
//! \file  solutions/tmdxiddk379d/f28002x/include/motor_drive_settings.h
//! \brief header file to be included in all labs
//!
//

#ifndef MOTOR_DRIVE_SETTINGS_H
#define MOTOR_DRIVE_SETTINGS_H

//
// Include project specific include files.
//

//
// List of control GND configurations - COLD or HOT
//
#define  COLD  1                // control GND is COLD
#define  HOT   2                // control GND is HOT

//
// Following is the list of the Build Level choices.
//
#define  FCL_LEVEL1  1          // Verify SVGEN module and PWM generation
#define  FCL_LEVEL2  2          // Verify ADC, park/clarke, calibrate the
                                // offset and speed measurement
#define  FCL_LEVEL3  3          // Verify closed current(torque) loop + its PI
#define  FCL_LEVEL4  4          // Verify speed loop and speed PID
#define  FCL_LEVEL5  5          // Verify position loop
#define  FCL_LEVEL6  6          // SFRA integration to verify bandwidth
                                // FCL_LEVEL6 only supports F28002x_Flash build
                                // configuration limit to RAM size

//
// This line sets the SAMPLING FREQUENCY to one of the available choices
//
#define  SINGLE_SAMPLING        1
#define  DOUBLE_SAMPLING        2

//
// Following is the list of Current sense options
//
#define  SHUNT_CURRENT_SENSE    1
#define  LEM_CURRENT_SENSE      2
#define  SD_CURRENT_SENSE       3

//
// Following is the list of Position Encoder options
// Select Position Feedback Option
//
#define  QEP_POS_ENCODER        1
#define  RESOLVER_POS_ENCODER   2
#define  BISS_POS_ENCODER       3
#define  ENDAT_POS_ENCODER      4
#define  SINCOS_POS_ENCODER     5
#define  T_FORMAT_ENCODER       6

#define  ENCODER_MAX_TYPES      7    // for Position Sensor Suite array size

#ifndef  ENCODER_MAX_TYPES
#error  Critical: ENCODER_MAX_TYPES must be defined !!
#endif

#if(ENCODER_MAX_TYPES <= T_FORMAT_ENCODER)
#error  Critical: the Position Sensor Suite array size is too small
#endif

//
// Here below, pick current loop controller option
//
#define  CMPLX_CNTLR        1
#define  PI_CNTLR           2

//
// Here below, pick position and speed loop controller option
//
#define  SPD_PID_CNTLR      1
#define  SPD_DCL_CNTLR      2
#define  SPD_NLPID_CNTLR    3

//
// User can select choices from available control configurations
//
#define  CGND               COLD
#define  BUILDLEVEL         FCL_LEVEL1
#define  SAMPLING_METHOD    SINGLE_SAMPLING        // SINGLE_SAMPLING       //
#define  FCL_CNTLR          PI_CNTLR               // CMPLX_CNTLR           //
#define  SPD_CNTLR          SPD_PID_CNTLR          // SPD_DCL_CNTLR          //
//#define  SPD_CNTLR        SPD_NLPID_CNTLR

#define  CURRENT_SENSE      LEM_CURRENT_SENSE
#define  POSITION_ENCODER   QEP_POS_ENCODER

#ifndef _FLASH
#if(BUILDLEVEL == FCL_LEVEL6)
#error FCL_LEVEL6 is only supported by "F28002x_FLASH" build configurations, \
       limit to the RAM size. Change the build configurations by right click \
       project name , select "Build Configurations->Set Active->F28002x_FLASH"
#endif
#endif  // _FLASH

//
// Select using Bit-Field or Drivelib mode
//
#define BITFIELD_MODE       0
#define DRIVERLIB_MODE      1

#define DRIVER_MODULE       BITFIELD_MODE           // DRIVERLIB_MODE        //

//
// set the motor parameters to the one available
//
#define ESTUN_EMJ04APB222   1
#define TEKNIC_2310PLN04K   2

#define USER_MOTOR          ESTUN_EMJ04APB222   //TEKNIC_2310PLN04K     //

//
// Generate error if no related definition
//
#if(CURRENT_SENSE != LEM_CURRENT_SENSE)
#error  Critical: Only LEM_CURRENT_SENSE is supported in this example
#endif

#if(POSITION_ENCODER != QEP_POS_ENCODER)
#error  Critical: Only QEP_POS_ENCODER is supported in this example
#endif

#ifndef BUILDLEVEL
#error  Critical: BUILDLEVEL must be defined !!
#endif  // BUILDLEVEL

#ifndef PI
#define PI 3.14159265358979
#endif

#endif  // end of MOTOR_DRIVE_SETTINGS_H definition

