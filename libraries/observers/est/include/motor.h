//#############################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:26 CDT 2021 $
// $Copyright:
// Copyright (C) 2017-2018 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef MOTOR_H
#define MOTOR_H

//! \file   libraries/motor/include/motor.h for ROM symbolic library
//! \brief  Contains motor related definitions
//!


//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! \defgroup MOTOR MOTOR
//! @{
//
//*****************************************************************************

// the includes

#include "types.h"


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs


//! \brief Enumeration for the motor types
//!
typedef enum
{
  MOTOR_TYPE_INDUCTION = 0,   //!< induction
  MOTOR_TYPE_PM               //!< permanent magnet
} MOTOR_Type_e;

//
//! \brief Enumeration for the Flying Start mode
//
typedef enum
{
    FLYINGSTART_MODE_HALT    = 0,       //!< Halt Mode
    FLYINGSTART_MODE_STANDBY = 1        //!< Standby Mode
} FlyingStart_Mode_e;

//
//! \brief Enumeration for the braking Mode
//
typedef enum
{
    FREE_STOP_MODE           = 0,       //!< Free stop mode without braking
    HARDSWITCH_BRAKE_MODE    = 1,       //!< Hard switch braking mode
    FORCESTOP_BRAKE_MODE     = 2,       //!< Force alignment braking mode
    DYNAMIC_BRAKE_MODE       = 3,       //!< N/A, Dynamic braking mode
    REGENERATION_BRAKE_MODE  = 4        //!< N/A, Regeneration braking mode
} BRAKE_Mode_e;

//
//! \brief Enumeration for the control mode
//
typedef enum
{
    OPERATE_MODE_SPEED  = 0,           //!< Speed close loop running mode
    OPERATE_MODE_TORQUE = 1            //!< Torque close loop running mode
} OPERATE_Mode_e;

//! \brief Enumeration for the estimator mode
//
typedef enum
{
    ESTIMATOR_MODE_FAST  = 0,             //!< FAST estimator
    ESTIMATOR_MODE_ESMO  = 1,             //!< ESMO estimator
    ESTIMATOR_MODE_APLL  = 2,             //!< APLL estimator
    ESTIMATOR_MODE_ENC   = 3,             //!< Encoder
    ESTIMATOR_MODE_BINT  = 4,             //!< BEMF Int
    ESTIMATOR_MODE_HALL  = 5              //!< Hall sensor
} ESTIMATOR_Mode_e;


//! \brief Enumeration for the motor drive control state
//
typedef enum
{
    SAMPLE_MODE_DCSS2  = 0,             //!< dclink_ss2
    SAMPLE_MODE_DCSS4  = 1,             //!< dclink_ss4
    SAMPLE_MODE_DCLINK = 2,             //!< dclink_ss
    SAMPLE_MODE_3LSR   = 3,             //!< three_shunt
    SAMPLE_MODE_3INL   = 4,             //!< inline
    SAMPLE_MODE_SDFM   = 5              //!< sdfm
} SAMPLE_Mode_e;

//! \brief Enumeration for the motor drive control state
//
typedef enum
{
    MCTRL_ON_IDLE      = 0,         //!< Power on in IDLE
    MCTRL_INIT_SET     = 1,         //!< Initialize run time parameters
    MCTRL_FAULT_STOP   = 2,         //!< motor stop with fault
    MCTRL_FIRST_RUN    = 3,         //!< First start the motor
    MCTRL_NORM_STOP    = 4,         //!< Normal stop
    MCTRL_CONT_RUN     = 5          //!< Continue run the motor
} MCTRL_State_e;


// State machine typedef for motor running status
typedef enum
{
    MOTOR_STOP_IDLE     = 0,
    MOTOR_SEEK_POS      = 1,
    MOTOR_ALIGNMENT     = 2,
    MOTOR_OL_START      = 3,
    MOTOR_CL_RUNNING    = 4,
    MOTOR_CTRL_RUN      = 5
} MOTOR_Status_e;

//! \brief Defines the motor parameters
//!
typedef struct _MOTOR_Params_
{
  MOTOR_Type_e    type;               //!< Defines the motor type

  uint_least16_t  numPolePairs;       //!< Defines the number of pole pairs

  float32_t         Lmag_H;             //!< Defines the magnetizing inductance, H

  float32_t         Ls_d_H;             //!< Defines the direct stator inductance, H
  float32_t         Ls_q_H;             //!< Defines the quadrature stator inductance, H

  float32_t         Rr_d_Ohm;           //!< Defines the direct rotor resistance, Ohm
  float32_t         Rr_q_Ohm;           //!< Defines the quadrature rotor resistance, Ohm

  float32_t         Rs_d_Ohm;           //!< Defines the direct stator resistance, Ohm
  float32_t         Rs_q_Ohm;           //!< Defines the quadrature stator resistance, Ohm

  float32_t         ratedFlux_Wb;       //!< Defines the rated flux, Wb
} MOTOR_Params;


// **************************************************************************
// the functions



//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // end of MOTOR_H definition





