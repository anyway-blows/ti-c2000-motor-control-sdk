//#############################################################################
//
// FILE:    dual_axis_servo_drive.h
//
// TITLE:   Include header files used in the project
//
// Group:   C2000
//
// Target Family: F2837x/F28004x
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

#ifndef DUAL_AXIS_SERVO_DRIVE_H
#define DUAL_AXIS_SERVO_DRIVE_H

//
//! \file   solutions/common/sensored_foc/include/dual_axis_servo_drive.h
//! \brief  header file to be included in all labs
//!
//


//
//! \defgroup LABS LABS
//! @{
//

//
// includes
//
#include <math.h>

#include "device.h"

#include "dual_axis_servo_drive_user.h"

#include "clarke.h"
#include "park.h"
#include "ipark.h"              // Include header for the IPARK object
#include "pi.h"                 // Include header for the PI  object
#include "fcl_pi.h"             // Include header for the FCL_PI object
#include "svgen.h"              // Include header for the SVGENDQ object
#include "rampgen.h"            // Include header for the RAMPGEN object
#include "rmp_cntl.h"           // Include header for the RMPCNTL object
#include "volt_calc.h"          // Include header for the PHASEVOLTAGE object
#include "speed_fr.h"           // Include header for the SPEED_MEAS_QEP object
#include "resolver.h"
#include "pid_grando.h"
#include "pid_reg3.h"

#if defined(F28002x_DEVICE)
#include "DCLF32.h"
#include "DCL_NLPID.h"
#include "fcl_foc_cpu_dm.h"

#include "dlog_2ch_f.h"
#elif defined(F28004x_DEVICE) || defined(F2837x_DEVICE)
#include "fcl_enum.h"
#include "fcl_cla_dm.h"
#include "fcl_cpu_cla_dm.h"
#include "cpu_cla_shared_dm.h"

#include "dlog_4ch_f.h"
#endif  // F28004x_DEVICE || F2837x_DEVICE

#if defined(DAC128S_ENABLE)
#include "dac128s085.h"
#endif  // DAC128S_ENABLE

#define DBUFF_SIZE_NUM  200

// Variables for Datalog module
#ifdef DLOG_ENABLE
#if defined (F28004x_DEVICE)
extern float32_t DBUFF_4CH1[DBUFF_SIZE_NUM];
extern float32_t DBUFF_4CH2[DBUFF_SIZE_NUM];
extern float32_t DBUFF_4CH3[DBUFF_SIZE_NUM];
extern float32_t DBUFF_4CH4[DBUFF_SIZE_NUM];

extern float32_t dlogCh1;
extern float32_t dlogCh2;
extern float32_t dlogCh3;
extern float32_t dlogCh4;

// Create an instance of DATALOG Module
extern DLOG_4CH_F dlog_4ch1;
#endif  // F28004x_DEVICE

#if defined(DAC128S_ENABLE)
extern DAC128S_Handle   dac128sHandle;
extern DAC128S_Obj      dac128s;
#endif  // DAC128S_ENABLE

#if defined (F28002x_DEVICE)
extern float32_t DBUFF_4CH1[DBUFF_SIZE_NUM];
extern float32_t DBUFF_4CH2[DBUFF_SIZE_NUM];

extern float32_t dlogCh1;
extern float32_t dlogCh2;

// Create an instance of DATALOG Module
extern DLOG_2CH_F dlog_2ch1;
#endif  // F28002x_DEVICE

#endif  // DLOG_ENABLE

#if defined(DAC128S_ENABLE)
extern DAC128S_Handle   dac128sHandle;
extern DAC128S_Obj      dac128s;
#endif  // DAC128S_ENABLE

extern float32_t posArray[8];
extern float32_t posPtrMax;

// Variables for SFRA module
#if(BUILDLEVEL == FCL_LEVEL6)
extern float32_t    sfraNoiseD;
extern float32_t    sfraNoiseQ;
extern float32_t    sfraNoiseW;
extern SFRATest_e   sfraTestLoop;
extern bool         sfraCollectStart;
#endif

extern float32_t speedRef;
extern float32_t IdRef;
extern float32_t IqRef;

extern CtrlState_e    ctrlState;
extern bool flagSyncRun ;

//
// the function prototypes
//

// slew programmable ramper
static inline float32_t ramper(float32_t in, float32_t out, float32_t rampDelta)
{
    float32_t err;

    err = in - out;

    if(err > rampDelta)
    {
        return(out + rampDelta);
    }
    else if(err < -rampDelta)
    {
        return(out - rampDelta);
    }
    else
    {
        return(in);
    }
}

//
// Reference Position Generator for position loop
//
static inline float32_t refPosGen(float32_t out, MOTOR_Vars_t *pMotor)
{
    float32_t in = posArray[pMotor->posPtr];

    out = ramper(in, out, pMotor->posSlewRate);

    if(in == out)
    {
        pMotor->posCntr++;

        if(pMotor->posCntr > pMotor->posCntrMax)
        {
            pMotor->posCntr = 0;

            pMotor->posPtr++;

            if(pMotor->posPtr >= pMotor->posPtrMax)
            {
                pMotor->posPtr = 0;
            }
        }
    }

    return(out);
}

//
// Current sensors scaling
//
static inline uint16_t scaleCurrentValue(float32_t current, float32_t currentSF)
{
    return((uint16_t)(current * currentSF));
}

//
// Voltage sensors scaling
//
static inline uint16_t scaleVoltageValue(float32_t voltage, float32_t voltageSF)
{
    return((uint16_t)(voltage * voltageSF));
}

//
// Get FCL timing details - time stamp taken in library after PWM update
//
static inline void getFOCCount(MOTOR_Vars_t *pMotor)
{
    pMotor->focCycleCount = EPWM_getTimeBaseCounterValue(pMotor->pwmBaseU);

    return;
}

//
// Get FCL timing details - time stamp taken in library after PWM update
//
static inline void getFOCTime(MOTOR_Vars_t *pMotor)
{
    if(EPWM_getTimeBaseCounterValue(pMotor->pwmBaseU) < pMotor->focCycleCount)
    {
        pMotor->focCycleCount = EPWM_getTimeBasePeriod(pMotor->pwmBaseU) -
                pMotor->focCycleCount;
    }

    if(pMotor->focCycleCountMax < pMotor->focCycleCount)
    {
        pMotor->focCycleCountMax = pMotor->focCycleCount;
    }

    if(pMotor->focClrCntr)
    {
        pMotor->focCycleCountMax = 0;
        pMotor->focClrCntr = 0;
    }

    //for 100MHz PWM clock
    pMotor->focExecutionTime_us = pMotor->focCycleCountMax * 0.01;

    return;
}

// ****************************************************************************
// Get FCL timing details - time stamp taken in library after PWM update
// ****************************************************************************
static inline void getFCLTime(MOTOR_Vars_t *pMotor)
{
    if(EPWM_getTimeBaseCounterValue(pMotor->pwmBaseU) < pMotor->fclCycleCount)
    {
        pMotor->fclCycleCount = EPWM_getTimeBasePeriod(pMotor->pwmBaseU) -
                pMotor->fclCycleCount;
    }

    if(pMotor->fclCycleCountMax < pMotor->fclCycleCount)
    {
        pMotor->fclCycleCountMax = pMotor->fclCycleCount;
    }

    if(pMotor->fclClrCntr)
    {
        pMotor->fclCycleCountMax = 0;
        pMotor->fclClrCntr = 0;
    }

    //for 100MHz PWM clock
    pMotor->fclUpdateLatency_us = pMotor->fclCycleCountMax * 0.01;

    return;
}

//! \brief      Get the dc bus voltage
//! \return     The dc bus voltage
static inline float32_t getVdc(MOTOR_Vars_t *ptrMotor)
{
    float32_t vdc;

    vdc = HWREGH(ptrMotor->volDC_PPBRESULT) * ptrMotor->voltageScale;

    if(vdc < 1.0)
    {
        vdc = 1.0;
    }

    return(vdc);
}

//! \brief      Initializes the parameters of motor
//! \details    Initializes all the parameters for each motor
//! \param[in]  pMotor   A pointer to the motorVars object
extern void initMotorParameters(MOTOR_Vars_t *pMotor, HAL_MTR_Handle mtrHandle);

//! \brief      Initializes the control variables of motor
//! \details    Initializes all the control variables for each motor
//! \param[in]  pMotor   A pointer to the motorVars object
extern void initControlVars(MOTOR_Vars_t *pMotor);

//! \brief      Reset the control variables of motor
//! \details    Reset the control variables for each motor
//! \param[in]  pMotor   A pointer to the motorVars object
extern void resetControlVars(MOTOR_Vars_t *pMotor);

//! \brief      Run offser calibration
//! \details    implements offset calculation using filters
//! \param[in]  pMotor   A pointer to the motorVars object
extern void runOffsetsCalculation(MOTOR_Vars_t *pMotor);

//! \brief      Run motor control
//! \details    Set current limitation, check fault
//! \param[in]  pMotor   A pointer to the motorVars object
extern void
runMotorControl(MOTOR_Vars_t *pMotor, HAL_MTR_Handle mtrHandle);


//! \brief      Run/stop dual motor control at the same time
extern void runSyncControl(void);

//
// SFRA utility functions
//
#if(BUILDLEVEL == FCL_LEVEL6)
extern void injectSFRA(void);
extern void collectSFRA(MOTOR_Vars_t *pMotor);
#endif

//
// Close the Doxygen group.
//! @} //defgroup
//

#endif // end of DUAL_AXIS_SERVO_DRIVE_H definition
