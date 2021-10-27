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

#ifndef _QEP_SENSOR_H_
#define _QEP_SENSOR_H_

//! \file   solutions\servo_drive_with_can\common\include\qep_sensor.h
//! \brief  Calculate the rotor angle and speed based on the QEP signals
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
//! \addtogroup QEP_SENSOR
//! @{
//
//*****************************************************************************

#include "qep_defs.h"
#include "math.h"
#include "types.h"
#include "hal.h"

// **************************************************************************
// \brief Defines the QEP_SENSOR object

typedef struct _QEP_SENSOR_Obj_
{
    float32_t lsw1Speed;            // initial force rotation speed in search
                                    // of QEP index pulse
    float32_t CaliAngle;
    float32_t Angle;
    float32_t OldAngle;         // History: Electrical angle at previous step (pu)
    float32_t OldAngleDiff;
    float32_t SpeedK1_filter;   // Parameter: Constant for differentiator
    float32_t SpeedK2_filter;   // Parameter: Constant for low-pass filter (pu)
    float32_t SpeedK3_filter;   // Parameter: Constant for low-pass filter (pu)
    float32_t SpeedKrpm;        // Output : Speed in rpm
    float32_t SpeedHz;
    float32_t SpeedGain;
    float32_t Tmp;              //Variable: Temp variable

    uint32_t      qepHandle;
    QEP qepsensor;

    int16_t AngleDiagnostic[4];
    ENC_Status_e lsw;
} QEP_SENSOR_Obj;


//! \brief Defines the QEP_SENSOR handle
//!
typedef struct _QEP_SENSOR_Obj_ *QEP_SENSOR_Handle;

#define EQEP_POSCNT_INIT_NOTHING             0U //!< No action
#define EQEP_POSCNT_INIT_RISING_EDGE    0x0200U //!< poscnt=posinit @ QEPI rise
#define EQEP_POSCNT_INIT_FALLING_EDGE   0x0300U //!< poscnt=posinit @ QEPI fall

// **************************************************************************
// the globals

// **************************************************************************
// the function prototypes

extern QEP_SENSOR_Handle QEP_SENSOR_init(void *pMemory,const size_t numBytes);

extern void QEP_SENSOR_setParams(QEP_SENSOR_Handle qepsensorinHandle);


extern void QEP_SENSOR_run(QEP_SENSOR_Handle sensorhandle);


static inline void EQEP_resetPoscnt(uint32_t base, uint16_t initMode)
{
    //
    // Check the arguments.
    //
    ASSERT(EQEP_isBaseValid(base));

    //
    // Set the init mode in the QEP Control register.
    //
    HWREGH(base + EQEP_O_QEPCTL) = (HWREGH(base + EQEP_O_QEPCTL) &
                                    ~(EQEP_QEPCTL_IEI_M)) | initMode;
}

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

#endif  //end of _QEP_SENSOR_H_ definition
