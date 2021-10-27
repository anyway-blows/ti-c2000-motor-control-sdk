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

#ifndef MOTOR1_DRIVE_H
#define MOTOR1_DRIVE_H

//
//! \file  solutions/fast_uni_lab/common/include/motor1_drive.h
//! \brief  header file to be included in all labs
//!
//

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
//! \addtogroup MOTOR DRIVE
//! @{
//
//*****************************************************************************


// includes
#ifdef __TMS320C28XX_CLA__
#include "libraries/math/include/CLAmath.h"
#else
#include "libraries/math/include/math.h"
#include <math.h>
#endif

#include "userParams.h"
#include "motor_common.h"

// *****************************************************************************
// the defines


// *****************************************************************************
// the typedefs


// *****************************************************************************
// the globals
extern volatile MOTOR_Handle motorHandle_M1;

extern volatile MOTOR_Vars_t motorVars_M1;

extern MOTOR_SetVars_t motorSetVars_M1;

extern USER_Params userParams_M1;

//!< the hardware abstraction layer object to motor control
extern HAL_MTR_Obj    halMtr_M1;

//!< the current Clarke transform object
extern CLARKE_Obj    clarke_I_M1;

//!< the voltage Clarke transform object
extern CLARKE_Obj    clarke_V_M1;

//!< the inverse Park transform object
extern IPARK_Obj     ipark_V_M1;

//!< the Park transform object
extern PARK_Obj      park_I_M1;

//!< the Park transform object
extern PARK_Obj      park_V_M1;

//!< the Id PI controller object
extern PI_Obj        pi_Id_M1;

//!< the Iq PI controller object
extern PI_Obj        pi_Iq_M1;

//!< the speed PI controller object
extern PI_Obj        pi_spd_M1;

//!< the space vector generator object
extern SVGEN_Obj     svgen_M1;

#if defined(MOTOR1_OVM)
//!< the handle for the space vector generator current
extern SVGENCURRENT_Obj svgencurrent_M1;
#endif  // MOTOR1_OVM

//!< the speed reference trajectory object
extern TRAJ_Obj     traj_spd_M1;

#if defined(MOTOR1_FWC)
//!< the fwc PI controller object
extern PI_Obj       pi_fwc_M1;
#endif  // MOTOR1_FWC

#if (DMC_BUILDLEVEL <= DMC_LEVEL_3) || defined(MOTOR1_VOLRECT) || \
               defined(MOTOR1_ESMO) || defined(MOTOR1_ENC)
//!< the Angle Generate onject for open loop control
extern ANGLE_GEN_Obj    angleGen_M1;
#endif  // DMC_BUILDLEVEL <= DMC_LEVEL_3 || MOTOR1_ESMO || MOTOR1_VOLRECT || MOTOR1_ENC

#if (DMC_BUILDLEVEL == DMC_LEVEL_2)
//!< the Vs per Freq object for open loop control
extern VS_FREQ_Obj    VsFreq_M1;
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_2)

#if defined(MOTOR1_ISBLDC)
//!< the isbldc object
extern ISBLDC_Obj isbldc_M1;

//!< the rimpulse object
extern RIMPULSE_Obj rimpulse_M1;

//!< the mod6cnt object
extern MOD6CNT_Obj mod6cnt_M1;

//!< the bldc object
extern BLDC_Obj bldc_M1;
#endif  // MOTOR1_ISBLDC

#if defined(MOTOR1_ENC)
//!< the handle for the enc object
extern ENC_Obj enc_M1;

//!< the handle for the speedcalc object
extern SPDCALC_Obj speedcalc_M1;

//!< the handle for the enc object
extern HALL_Obj hall_M1;
#endif  // MOTOR1_ENC


#if defined(MOTOR1_ESMO)
//!< the speedfr object
extern SPDFR_Obj spdfr_M1;

//!< the esmo object
extern ESMO_Obj esmo_M1;
#endif  // MOTOR1_ESMO

#if defined(MOTOR1_MTPA)
//!< the Maximum torque per ampere (MTPA) object
extern MTPA_Obj     mtpa_M1;
#endif  // MOTOR1_MTPA

#if defined(MOTOR1_SSIPD)
extern SSIPD_Obj       ssipd_M1;
#endif  // MOTOR1_SSIPD

#if defined(MOTOR1_DCLINKSS)
//!< the single-shunt current reconstruction object
extern DCLINK_SS_Obj    dclink_M1;
#endif // MOTOR1_DCLINKSS

#ifdef MOTOR1_VOLRECT
//!< the voltage reconstruct object
extern VOLREC_Obj volrec_M1;
#endif  // MOTOR1_VOLRECT

#if defined(BSXL8320RS_REVA) || defined(BSXL8323RS_REVA) || \
    defined(BSXL8353RS_REVA) || defined(BSXL8316RT_REVA)
// Watch window interface to the /drv8323/drv8353/drv8316 SPI
extern DRVIC_VARS_t drvicVars_M1;
#endif  // BSXL8320RS_REVA || BSXL8323RS_REVA || BSXL8353RS_REVA  ||
        // BSXL8316RT_REVA

// *****************************************************************************
// the function prototypes

//! \brief The main interrupt service (ISR) routine
extern __interrupt void motor1CtrlISR(void);

//! \brief initialize motor control handles
extern void initMotor1Handles(MOTOR_Handle handle);

//! \brief initialize motor control parameters
extern void initMotor1CtrlParameters(MOTOR_Handle handle);

//! \brief run motor control in main loop
extern void runMotor1Control(MOTOR_Handle handle);

//! \brief runs offset calculation using filters
extern void runMotor1OffsetsCalculation(MOTOR_Handle handle);

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

#endif // end of MOTOR1_DRIVE_H definition
