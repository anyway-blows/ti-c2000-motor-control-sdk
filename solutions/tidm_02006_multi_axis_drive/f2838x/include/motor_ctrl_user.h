//#############################################################################
//
// FILE:    motor_ctrl_user.h
//
// TITLE:   motor parameters definition
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
//! \file  solutions/multi_axis_drive/f2838x/include/motor_ctrl_user.h
//! \brief header file to be included in all labs
//!
//

#ifndef MOTOR_CTRL_USER_H
#define MOTOR_CTRL_USER_H

//
// Include project specific include files.
//
#include "motor_ctrl_settings.h"

#define M_SYNC_PWM_BASE         EPWM4_BASE          // M: Set up based board
#define M_SYNT_PWM_BASE         EPWM7_BASE          // M: Set up based board
#define M_CISR_PWM_BASE         EPWM8_BASE          // M: Set up based board

#define M_CISR_INT_PWM          INT_EPWM8           // M: Set up based board

#define M_SYNC_NUM              0
#define M_SYNT_NUM              1
#define M_CISR_NUM              2

#define M_PWM_SNYC_SOC          FSI_EXT_TRIGSRC_EPWM4_SOCB

//
// define EN_GATE and SPI_CS pin for DRV device for Motor 1
//
#define M_DRV_SPI_CS            27          // NC: Set up based IDDK board
#define M_EN_GATE_GPIO          10          // NC: Set up based IDDK board

#define M_CLR_FAULT_GPIO        41          // NC: Set up based IDDK board
#define M_nFAULT_GPIO           40          // NC: Set up based IDDK board
#define M_XBAR_INPUT_GPIO       40          // NC: Set up based IDDK board
#define M_TRIP_CC_GPIO          58          // NC: Set up based IDDK board                                         // re-connect to GPIO15(H13-12)

//
// Motor Parameters
//

//
// PWM, SAMPLING FREQUENCY and Current Loop Band width definitions
//
#define M_INV_FREQUENCY           10.0F      // in KHz
#define M_CTRL_FREQUENCY          20.0F      // in KHz

#if(SAMPLING_METHOD == SINGLE_SAMPLING)
#define M_ISR_FREQUENCY           (M_CTRL_FREQUENCY)
#elif(SAMPLING_METHOD == DOUBLE_SAMPLING)
#define M_ISR_FREQUENCY           (2*M_CTRL_FREQUENCY)
#endif

//
// Keep PWM Period same between single sampling and double sampling
//
#define M_INV_PWM_TICKS         (((SYSTEM_FREQUENCY / 2.0F) / M_INV_FREQUENCY) * 1000)
#define M_INV_PWM_TBPRD         (M_INV_PWM_TICKS / 2)
#define M_INV_PWM_HALF_TBPRD    (M_INV_PWM_TBPRD / 2)

#define M_CTRL_PWM_TICKS        (((SYSTEM_FREQUENCY / 2.0F) / M_CTRL_FREQUENCY) * 1000)
#define M_CTRL_PWM_TBPRD        (M_CTRL_PWM_TICKS / 2)
#define M_CTRL_PWM_HALF_TBPRD   (M_CTRL_PWM_TBPRD / 2)
#define M_SAMPLING_FREQ         (M_ISR_FREQUENCY * 1000)
#define M_CUR_LOOP_BANDWIDTH    (2.0F * PI * M_SAMPLING_FREQ / 18)

//
// FCL Computation time predetermined from library
//
#define M_FCL_COMPUTATION_TIME  (1.00F)  //in uS

//
// NOTE:-
// Base voltage and base current information from TIDA-00909 doc is
// based off of an ADC that works at 3.3V reference.
// The base current = 16.5A (for a spread of 3.3V - 1.65V = 1.65V)
// The base voltage  = 81.5 / sqrt(3)=
// Define the base quantites
//
#define M_BASE_VOLTAGE          236.14F // Base peak phase voltage (volt),
                                       // maximum measurable Vdc/sqrt(3)
#define M_BASE_SHUNT_CURRENT    9.95F   // Base peak phase current (amp),
                                        // maximum measurable peak curr.
#define M_BASE_LEM_CURRENT      12.0F   // Base peak phase current (amp),
                                        // maximum measurable peak current
#define M_BASE_CURRENT          M_BASE_LEM_CURRENT
#define M_BASE_TORQUE           NULL     // Base torque (N.m)
#define M_BASE_FLUX             NULL     // Base flux linkage (volt.sec/rad)
#define M_BASE_FREQ             250      // Base electrical frequency (Hz)
#define M_MAXIMUM_CURRENT       8.0F     // Motor maximum torque current (amp)

#define M_SPEED_REF     0.05F            // reference speed (pu)
#define M_ID_START      0.1F             // alignment reference d-axis current
#define M_ID_RUN        0.0F             // running d-axis current
#define M_IQ_START      0.05F            // startup q-axis current

//
// Current sensors scaling
// 1.0pu current ==> 9.95A -> 2048 counts ==> 8A -> 1647
//
#define M_CURRENT_SCALE(A)             (uint16_t)(2048 * A / M_BASE_CURRENT)

//
// Analog scaling with ADC
//
#define M_ADC_PU_SCALE_FACTOR          0.000244140625     // 1/2^12
#define M_ADC_PU_PPB_SCALE_FACTOR      0.000488281250     // 1/2^11

//
// Current Scale
//
#define M_MAXIMUM_SCALE_CURRENT        M_BASE_CURRENT * 2.0
#define M_CURRENT_SENSE_SCALE          (M_MAXIMUM_SCALE_CURRENT / 4096.0)

//
// Voltage Scale
//
#define M_MAXIMUM_SCALE_VOLATGE        M_BASE_VOLTAGE * 1.732050808
#define M_VOLTAGE_SENSE_SCALE          (M_MAXIMUM_SCALE_VOLATGE / 4096.0)

#endif  // end of MOTOR_CTRL_USER_H definition

