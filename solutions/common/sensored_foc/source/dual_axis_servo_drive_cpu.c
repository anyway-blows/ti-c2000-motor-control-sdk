//#############################################################################
//
// FILE:    dual_axis_servo_dirve.c
//
// TITLE:   dual-axis motor drive on the related kits
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

//
// includes
//
#include "dual_axis_servo_drive_settings.h"
#include "dual_axis_servo_drive_user.h"
#include "dual_axis_servo_drive_hal.h"
#include "dual_axis_servo_drive_cpu.h"

#include "sfra_settings.h"

//
// Functions
//
#ifdef _FLASH
#pragma CODE_SECTION(motor1ControlISR, ".TI.ramfunc");
#pragma CODE_SECTION(motor2ControlISR, ".TI.ramfunc");

#pragma INTERRUPT (motor1ControlISR, HPI)
#pragma INTERRUPT (motor2ControlISR, HPI)
#endif

//
//  Prototype statements for Local Functions
//
extern __interrupt void motor1ControlISR(void);
extern __interrupt void motor2ControlISR(void);


//
// USER Variables
//

//
// Global variables used in this system
//
MOTOR_Vars_t motorVars[2];
#pragma DATA_SECTION(motorVars, "motor_data");

// Variables for Field Oriented Control
float32_t VdTesting = 0.0;          // Vd reference (pu)
float32_t VqTesting = 0.15;         // Vq reference (pu)
#pragma DATA_SECTION(VdTesting, "ramInitVars");
#pragma DATA_SECTION(VqTesting, "ramInitVars");

// Variables for position reference generation and control
float32_t posArray[8] = {2.5, -2.5, 3.5, -3.5, 5.0, -5.0, 8.0, -8.0};
float32_t posPtrMax = 4;
#pragma DATA_SECTION(posArray, "ramInitVars");
#pragma DATA_SECTION(posPtrMax, "ramInitVars");

// control dual motor with the same speed and acceleration at the same time
float32_t speedRef = 0.10f;
float32_t IdRef = 0.0f;
float32_t IqRef = 0.10f;
#pragma DATA_SECTION(speedRef, "ramInitVars");
#pragma DATA_SECTION(IdRef, "ramInitVars");
#pragma DATA_SECTION(IqRef, "ramInitVars");

MotorRunStop_e runMotor;
CtrlState_e    ctrlState;
bool flagSyncRun ;

// Variables for SFRA module
#if(BUILDLEVEL == FCL_LEVEL6)
float32_t   sfraNoiseD;
float32_t   sfraNoiseQ;
float32_t   sfraNoiseW;
SFRATest_e  sfraTestLoop;        //speedLoop;
bool        sfraCollectStart;
#endif

HAL_Handle    halHandle;    //!< the handle for the hardware abstraction layer
HAL_Obj       hal;          //!< the hardware abstraction layer object

HAL_MTR_Handle halMtrHandle[2];   //!< the handle for the hardware abstraction
                                  //!< layer to motor control
HAL_MTR_Obj    halMtr[2];         //!< the hardware abstraction layer object
                                  //!< to motor control
// FCL Latency variables
volatile uint16_t FCL_cycleCount[2];

//
//   Various Incremental Build levels
//

//****************************************************************************
// INCRBUILD 1
//****************************************************************************
//
#if(BUILDLEVEL == FCL_LEVEL1)
// =============================== FCL_LEVEL 1 =================================
// Level 1 verifies
//  - PWM Generation blocks and DACs
// =============================================================================
// build level 1 subroutine for motor_1
#pragma FUNC_ALWAYS_INLINE(buildLevel1)

static inline void buildLevel1(MOTOR_Vars_t *pMotor)
{
    //
    // call QEP module to get the rotor position and speed
    //
    FCL_runQEPPosEst(pMotor);

    //
    // control force angle generation based on 'runMotor'
    //
    if(pMotor->runMotor == MOTOR_RUN)
    {
        pMotor->rc.TargetValue = pMotor->speedRef;
        pMotor->ipark.Ds = VdTesting;
        pMotor->ipark.Qs = VqTesting;
    }
    else
    {
        pMotor->rc.TargetValue = 0;
        pMotor->rc.SetpointValue = 0;
        pMotor->ipark.Ds = 0.0;
        pMotor->ipark.Qs = 0.0;
    }

    //
    // Connect inputs of the RMPCTRL module and call the ramp control module
    //
    fclRampControl(&pMotor->rc);

    //
    // Connect inputs of the RAMPGEN module and call the ramp generator module
    //
    pMotor->rg.Freq = pMotor->rc.SetpointValue;
    fclRampGen((RAMPGEN *)&pMotor->rg);

    //
    // Connect inputs of the INV_PARK module and call the inverse park module
    //
    pMotor->ipark.Sine = __sinpuf32(pMotor->rg.Out);
    pMotor->ipark.Cosine = __cospuf32(pMotor->rg.Out);
    runIPark(&pMotor->ipark);

    //
    // Call QEP module
    //
    FCL_runQEPPosEstWrap(pMotor);

    //
    //  Measure DC Bus voltage
    //
    pMotor->FCL_params.Vdcbus = getVdc(pMotor);

    //
    // Connect inputs of the SVGEN_DQ module and call the svgen module
    //
    pMotor->svgen.Ualpha = pMotor->ipark.Alpha;
    pMotor->svgen.Ubeta  = pMotor->ipark.Beta;
    runSVGenDQ(&pMotor->svgen);

    //
    // Computed Duty and Write to CMPA register
    //
    EPWM_setCounterCompareValue(pMotor->pwmBaseU, EPWM_COUNTER_COMPARE_A,
                    (uint16_t)((pMotor->pwmHalfPeriod * pMotor->svgen.Tc) +
                                pMotor->pwmHalfPeriod));

    EPWM_setCounterCompareValue(pMotor->pwmBaseV, EPWM_COUNTER_COMPARE_A,
                    (uint16_t)((pMotor->pwmHalfPeriod * pMotor->svgen.Ta) +
                                pMotor->pwmHalfPeriod));

    EPWM_setCounterCompareValue(pMotor->pwmBaseW, EPWM_COUNTER_COMPARE_A,
                    (uint16_t)((pMotor->pwmHalfPeriod * pMotor->svgen.Tb) +
                                pMotor->pwmHalfPeriod));

    return;
}

#endif // (BUILDLEVEL==FCL_LEVEL1)

//
//****************************************************************************
// INCRBUILD 2
//****************************************************************************
//
#if(BUILDLEVEL == FCL_LEVEL2)
// =============================== FCL_LEVEL 2 =================================
// Level 2 verifies
//   - verify inline shunt current sense schemes
//     - analog-to-digital conversion
//   - Current Limit Settings for over current protection
//   - Position sensor interface is taken care by FCL lib using QEP
//     - speed estimation
// =============================================================================
#pragma FUNC_ALWAYS_INLINE(buildLevel2)

// build level 2 subroutine for motor_1
static inline void buildLevel2(MOTOR_Vars_t *pMotor)
{
    //
    // call QEP module to get the rotor position and speed
    //
    FCL_runQEPPosEst(pMotor);

    // -------------------------------------------------------------------------
    // Alignment Routine: this routine aligns the motor to zero electrical
    // angle and in case of QEP also finds the index location and initializes
    // the angle w.r.t. the index location
    // -------------------------------------------------------------------------
    if(pMotor->runMotor == MOTOR_RUN)
    {
        // Connect inputs of the RMP module and call the ramp control module
        pMotor->rc.TargetValue = pMotor->speedRef;

        if(pMotor->lsw == ENC_CALIBRATION_DONE)
        {
            pMotor->ipark.Ds = VdTesting;
            pMotor->ipark.Qs = VqTesting;
        }
        else if(pMotor->lsw == ENC_ALIGNMENT)
        {
            // for restarting from (runMotor = STOP)
            pMotor->rc.TargetValue = 0;
            pMotor->rc.SetpointValue = 0;

            // for QEP, spin the motor to find the index pulse
            pMotor->lsw = ENC_WAIT_FOR_INDEX;

            pMotor->ipark.Ds = VdTesting;
            pMotor->ipark.Qs = VqTesting;
        } // (lsw == ENC_ALIGNMENT)
    }
    else
    {
        pMotor->lsw = ENC_ALIGNMENT;
        pMotor->rc.TargetValue = 0.0;
        pMotor->IdRef = 0;
        pMotor->cmplx_Id.ref = pMotor->IdRef;

        FCL_resetController(pMotor);

        pMotor->ipark.Ds = 0.0;
        pMotor->ipark.Qs = 0.0;
    }



    fclRampControl(&pMotor->rc);

    //
    // Connect inputs of the RAMP GEN module and call the ramp generator module
    //
    pMotor->rg.Freq = pMotor->rc.SetpointValue;
    fclRampGen((RAMPGEN *)&pMotor->rg);

// ----------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5)
//  to (-1,+1). Connect inputs of the CLARKE module and call the clarke
//  transformation module
// ----------------------------------------------------------------------------
    //wait on ADC EOC
#if(DRIVER_MODULE == BITFIELD_MODE)
    while(pMotor->pADCIntFlag->bit.ADCINT1 == 0);
#else
    while(ADC_getInterruptStatus(pMotor->adcBaseW, ADC_INT_NUMBER1) == 0);
#endif

    pMotor->clarke.As = pMotor->FCL_params.adcPPBScale *
            ((float32_t)ADC_readPPBResult(pMotor->curB_PPBRESULT,
                                          pMotor->curB_PPBNumber));

    pMotor->clarke.Bs = pMotor->FCL_params.adcPPBScale *
            ((float32_t)ADC_readPPBResult(pMotor->curC_PPBRESULT,
                                          pMotor->curC_PPBNumber));
    runClarke(&pMotor->clarke);

    //
    //  Measure DC Bus voltage
    //
    pMotor->FCL_params.Vdcbus = getVdc(pMotor);

    //
    // Connect inputs of the PARK module and call the park module
    //
    pMotor->park.Alpha  = pMotor->clarke.Alpha;
    pMotor->park.Beta   = pMotor->clarke.Beta;
    pMotor->park.Angle  = pMotor->rg.Out;
    pMotor->park.Sine   = __sinpuf32(pMotor->park.Angle);
    pMotor->park.Cosine = __cospuf32(pMotor->park.Angle);
    runPark(&pMotor->park);

    //
    // Connect inputs of the INV_PARK module and call the inverse park module
    //
    pMotor->ipark.Sine = pMotor->park.Sine;
    pMotor->ipark.Cosine = pMotor->park.Cosine;
    runIPark(&pMotor->ipark);

    //
    // Position encoder suite module
    //
    FCL_runQEPPosEstWrap(pMotor);

    //
    // Connect inputs of the SPEED_FR module and call the speed calculation module
    //
    pMotor->speed.ElecTheta = pMotor->posElecTheta;
    runSpeedFR(&pMotor->speed);

    //
    // Connect output speed of SPEED_FR to speedWe
    //
    pMotor->speedWe = pMotor->speed.Speed;

    //
    // Connect inputs of the SVGEN_DQ module and call the space-vector gen. module
    //
    pMotor->svgen.Ualpha = pMotor->ipark.Alpha;
    pMotor->svgen.Ubeta  = pMotor->ipark.Beta;
    runSVGenDQ(&pMotor->svgen);

// ----------------------------------------------------------------------------
//  Computed Duty and Write to CMPA register
// ----------------------------------------------------------------------------
    EPWM_setCounterCompareValue(pMotor->pwmBaseU, EPWM_COUNTER_COMPARE_A,
                    (uint16_t)((pMotor->pwmHalfPeriod * pMotor->svgen.Tc) +
                                pMotor->pwmHalfPeriod));

    EPWM_setCounterCompareValue(pMotor->pwmBaseV, EPWM_COUNTER_COMPARE_A,
                    (uint16_t)((pMotor->pwmHalfPeriod * pMotor->svgen.Ta) +
                                pMotor->pwmHalfPeriod));

    EPWM_setCounterCompareValue(pMotor->pwmBaseW, EPWM_COUNTER_COMPARE_A,
                    (uint16_t)((pMotor->pwmHalfPeriod * pMotor->svgen.Tb) +
                                pMotor->pwmHalfPeriod));

    return;
}

#endif // (BUILDLEVEL==FCL_LEVEL2)


//
//****************************************************************************
// INCRBUILD 3
//****************************************************************************
//
#if(BUILDLEVEL == FCL_LEVEL3)
// =============================== FCL_LEVEL 3 ================================
//  Level 3 verifies the dq-axis current regulation performed by PID and speed
//  measurement modules
//  lsw = ENC_ALIGNMENT      : lock the rotor of the motor
//  lsw = ENC_WAIT_FOR_INDEX : close the current loop
//  NOTE:-
//      1. Iq loop is closed using actual QEP angle.
//         Therefore, motor speed races to high speed with lighter load. It is
//         better to ensure the motor is loaded during this test. Otherwise,
//         the motor will run at higher speeds where it can saturate.
//         It may be typically around the rated speed of the motor or higher.
//      2. clarke1.As and clarke1.Bs are not brought out from the FCL library
//         as of library release version 0x02
// ============================================================================

// build level 3 subroutine for motor_1
#pragma FUNC_ALWAYS_INLINE(buildLevel3)

static inline void buildLevel3(MOTOR_Vars_t *pMotor)
{
    //
    // call QEP module to get the rotor position and speed
    //
    FCL_runQEPPosEst(pMotor);

    //
    // Fast current loop controller
    //
#if(FCL_CNTLR ==  PI_CNTLR)
    FCL_runPICtrl(pMotor);
#endif

#if(FCL_CNTLR ==  CMPLX_CNTLR)
    FCL_runComplexCtrl(pMotor);
#endif

    //
    // Measure DC Bus voltage using SDFM Filter3
    //
    pMotor->FCL_params.Vdcbus = getVdc(pMotor);

    //
    // Fast current loop controller wrapper
    //
#if(FCL_CNTLR ==  PI_CNTLR)
    FCL_runPICtrlWrap(pMotor);
#endif

#if(FCL_CNTLR ==  CMPLX_CNTLR)
    FCL_runComplexCtrlWrap(pMotor);
#endif

    //
    // Alignment Routine: this routine aligns the motor to zero electrical
    // angle and in case of QEP also finds the index location and initializes
    // the angle w.r.t. the index location
    //
    if(pMotor->runMotor == MOTOR_RUN)
    {
        // Connect inputs of the RMP module and call the ramp control module
        pMotor->rc.TargetValue = pMotor->speedRef;

        if(pMotor->lsw == ENC_CALIBRATION_DONE)
        {
            pMotor->IdRef = pMotor->IdRef_run;
        }
        else if(pMotor->lsw == ENC_ALIGNMENT)
        {
            pMotor->rc.TargetValue = 0;
            pMotor->rc.SetpointValue = 0;

            // alignment current
            pMotor->IdRef = pMotor->IdRef_start;

            // set up an alignment and hold time for shaft to settle down
            if(pMotor->cmplx_Id.ref >= pMotor->IdRef)
            {
                pMotor->alignCntr++;

                if(pMotor->alignCntr >= pMotor->alignCnt)
                {
                    pMotor->alignCntr  = 0;

                    // for QEP, spin the motor to find the index pulse
                    pMotor->lsw = ENC_WAIT_FOR_INDEX;
                }
            }

        } // end else if(lsw == ENC_ALIGNMENT)
    }
    else
    {
        pMotor->lsw = ENC_ALIGNMENT;
        pMotor->cmplx_Id.ref = 0;
        pMotor->IdRef = 0;
        pMotor->rc.TargetValue = 0.0;

        FCL_resetController(pMotor);
    }

    fclRampControl(&pMotor->rc);

    //
    // Connect inputs of the RAMP GEN module and call the ramp generator module
    //
    pMotor->rg.Freq = pMotor->rc.SetpointValue;
    fclRampGen((RAMPGEN *)&pMotor->rg);

    //
    // Position encoder suite module
    //
    FCL_runQEPPosEstWrap(pMotor);

    //
    // Connect inputs of the SPEED_FR module and
    // call the speed calculation module
    //
    pMotor->speed.ElecTheta = pMotor->posElecTheta;
    runSpeedFR(&pMotor->speed);

    //
    // Connect output speed of SPEED_FR to speedWe
    //
    pMotor->speedWe = pMotor->speed.Speed;

    //
    // setup iqref for FCL
    //
    pMotor->cmplx_Iq.ref =
           (pMotor->lsw == ENC_ALIGNMENT) ? 0.0f : pMotor->IqRef;

    //
    // setup idref for FCL
    //
    pMotor->cmplx_Id.ref =
           ramper(pMotor->IdRef, pMotor->cmplx_Id.ref, 0.00001);

    // DEBUG: customer can remove the below code in final implementation
    // get FOC count
    getFOCCount(pMotor);

    // get FCL latency
    getFCLTime(pMotor);

    // get FOC execution time
    getFOCTime(pMotor);
    // DEBUG: customer can remove the above code in final implementation

    return;
}

#endif // (BUILDLEVEL==FCL_LEVEL3)

//
//****************************************************************************
// INCRBUILD 4
//****************************************************************************
//
#if((BUILDLEVEL == FCL_LEVEL4) || (BUILDLEVEL == FCL_LEVEL6) )
// =============================== FCL_LEVEL 4 ================================
// Level 4 verifies the speed regulator performed by PID module.
// The system speed loop is closed by using the measured speed as feedback
//  lsw = ENC_ALIGNMENT      : lock the rotor of the motor
//  lsw = ENC_WAIT_FOR_INDEX : - needed only with QEP encoders until first
//                               index pulse
//                             - Loops shown for 'lsw=ENC_CALIBRATION_DONE' are
//                               closed in this stage
//  lsw = ENC_CALIBRATION_DONE      : close speed loop and current loops Id, Iq
//
//  ****************************************************************
//
//  Level 6 verifies the SFRA functions used to verify bandwidth.
//  This demo code uses Level 4 code to perform SFRA analysis on
//  a current loop inside the speed loop
//
// ============================================================================
// build level 4/6 subroutine for motor_1
#pragma FUNC_ALWAYS_INLINE(buildLevel46)

static inline void buildLevel46(MOTOR_Vars_t *pMotor)
{
    //
    // call QEP module to get the rotor position and speed
    //
    FCL_runQEPPosEst(pMotor);

    //
    // Fast current loop controller
    //
#if(FCL_CNTLR ==  PI_CNTLR)
    FCL_runPICtrl(pMotor);
#endif

#if(FCL_CNTLR ==  CMPLX_CNTLR)
    FCL_runComplexCtrl(pMotor);
#endif

    //
    // Measure DC Bus voltage
    //
    pMotor->FCL_params.Vdcbus = getVdc(pMotor);

    //
    // Fast current loop controller wrapper
    //
#if(FCL_CNTLR ==  PI_CNTLR)
    FCL_runPICtrlWrap(pMotor);
#endif

#if(FCL_CNTLR ==  CMPLX_CNTLR)
    FCL_runComplexCtrlWrap(pMotor);
#endif

    //
    // Alignment Routine: this routine aligns the motor to zero electrical
    // angle and in case of QEP also finds the index location and initializes
    // the angle w.r.t. the index location
    //
    if(pMotor->runMotor == MOTOR_RUN)
    {
        if(pMotor->lsw == ENC_CALIBRATION_DONE)
        {
            pMotor->IdRef = pMotor->IdRef_run;
            pMotor->rc.TargetValue = pMotor->speedRef;
        } // (lsw == ENC_CALIBRATION_DONE)
        else if(pMotor->lsw == ENC_WAIT_FOR_INDEX)
        {
            //
            //  Connect inputs of the RAMP GEN module and call the ramp generator module
            //
            pMotor->rg.Freq = pMotor->rc.SetpointValue;
            fclRampGen((RAMPGEN *)&pMotor->rg);

            pMotor->rc.TargetValue = pMotor->lsw1Speed *
                    (pMotor->speedRef > 0 ? 1 : -1);
        } // (lsw == ENC_WAIT_FOR_INDEX)
        else if(pMotor->lsw == ENC_ALIGNMENT)
        {
            pMotor->rc.TargetValue = 0;
            pMotor->rc.SetpointValue = 0;

            // alignment current
            pMotor->IdRef = pMotor->IdRef_start;

            // set up an alignment and hold time for shaft to settle down
#if(BUILDLEVEL == FCL_LEVEL6)
            if(pMotor->ctrlIdRef >= pMotor->IdRef)
#else   // (BUILDLEVEL == FCL_LEVEL6)
            if(pMotor->cmplx_Id.ref >= pMotor->IdRef)
#endif  // (BUILDLEVEL == FCL_LEVEL4)
            {
                pMotor->alignCntr++;

                if(pMotor->alignCntr >= pMotor->alignCnt)
                {
                    pMotor->alignCntr  = 0;

                    // for QEP, spin the motor to find the index pulse
                    pMotor->lsw = ENC_WAIT_FOR_INDEX;
                    pMotor->IdRef = pMotor->IdRef_run;
                }
            }
        } // (lsw == ENC_ALIGNMENT)
    }
    else
    {
        pMotor->IdRef = 0.0;
        pMotor->rc.TargetValue = 0.0;

        FCL_resetController(pMotor);
    }

    //
    //  Connect inputs of the RMP module and call the ramp control module
    //
    fclRampControl(&pMotor->rc);

    //
    // Position encoder suite module
    //
    FCL_runQEPPosEstWrap(pMotor);

    // Connect inputs of the SPEED_FR module and
    // call the speed calculation module
//    pMotor->speed.ElecTheta = pMotor->posElecTheta;
    pMotor->speed.ElecTheta = pMotor->pangle;
    pMotor->speedWe = runSpeedFR(&pMotor->speed);

    // call the speed observer module
//  pMotor->speedWe = runSpeedObserve(&pMotor->speedObs, pMotor->pangle);

#if(BUILDLEVEL == FCL_LEVEL6)
    if(pMotor->sfraEnableFlag == true)
    {
        if(sfraCollectStart == true)
        {
            //
            // SFRA collect routine,
            // only to be called after SFRA inject has occurred 1st
            //

            collectSFRA(pMotor);    // Collect noise feedback from loop
        }

        //
        //  SFRA injection
        //
        injectSFRA();               // create SFRA Noise per 'sfraTestLoop'

        sfraCollectStart = true;
    }
    else
    {
        sfraNoiseD = 0.0f;
        sfraNoiseQ = 0.0f;
        sfraNoiseW = 0.0f;
    }
#endif

    //
    // Connect inputs of the PI module and call the PID speed controller module
    //
    pMotor->speedLoopCount++;

    if(pMotor->speedLoopCount >= pMotor->speedLoopPrescaler)
    {

#if(BUILDLEVEL == FCL_LEVEL6)
        // SFRA Noise injection in speed loop
        pMotor->pid_spd.term.Ref = pMotor->rc.SetpointValue + sfraNoiseW;
#else   // (BUILDLEVEL == FCL_LEVEL6)
        pMotor->pid_spd.term.Ref = pMotor->rc.SetpointValue;  //speedRef;
#endif  // (BUILDLEVEL == FCL_LEVEL4)

        pMotor->pid_spd.term.Fbk = pMotor->speedWe;
        runPI(&pMotor->pid_spd);

        pMotor->speedLoopCount = 0;
    }

    if((pMotor->lsw != ENC_CALIBRATION_DONE) ||
            (pMotor->runMotor == MOTOR_STOP))
    {
        pMotor->pid_spd.data.d1 = 0;
        pMotor->pid_spd.data.d2 = 0;
        pMotor->pid_spd.data.i1 = 0;
        pMotor->pid_spd.data.ud = 0;
        pMotor->pid_spd.data.ui = 0;
        pMotor->pid_spd.data.up = 0;
    }

    //
    //    setup iqref and idref for FCL
    //
#if(BUILDLEVEL == FCL_LEVEL6)
    // SFRA Noise injection in Q axis, setup iqref
    pMotor->cmplx_Iq.ref =
            (pMotor->lsw == ENC_ALIGNMENT) ? 0 :
                    (pMotor->lsw == ENC_WAIT_FOR_INDEX) ?
                            pMotor->IqRef :
                            (pMotor->pid_spd.term.Out + sfraNoiseQ);

    // SFRA Noise injection in D axis, setup idref
    pMotor->ctrlIdRef =
            ramper(pMotor->IdRef, pMotor->ctrlIdRef, 0.00001);

    pMotor->cmplx_Id.ref = pMotor->ctrlIdRef + sfraNoiseD;
#else   // if(BUILDLEVEL == FCL_LEVEL4)
    //
    // Setup iqref for FCL
    //
    pMotor->cmplx_Iq.ref = (pMotor->lsw == ENC_ALIGNMENT) ? 0.0f :
                               (pMotor->lsw == ENC_WAIT_FOR_INDEX) ?
                                 pMotor->IqRef : pMotor->pid_spd.term.Out;

    //
    // Setup idref for FCL
    //
    pMotor->cmplx_Id.ref = ramper(pMotor->IdRef, pMotor->cmplx_Id.ref, 0.00001);
#endif

    // DEBUG: customer can remove the below code in final implementation
    // get FOC count
    getFOCCount(pMotor);

    // get FCL latency
    getFCLTime(pMotor);

    // get FOC execution time
    getFOCTime(pMotor);
    // DEBUG: customer can remove the above code in final implementation

   return;
}

#endif // ( (BUILDLEVEL==FCL_LEVEL4) || (BUILDLEVEL == FCL_LEVEL6) )

//
//****************************************************************************
// INCRBUILD 5
//****************************************************************************
//
#if(BUILDLEVEL == FCL_LEVEL5)
// =============================== FCL_LEVEL 5 =================================
//  Level 5 verifies the position control
//  Position references generated locally from a posArray
//  lsw = ENC_ALIGNMENT      : lock the rotor of the motor
//  lsw = ENC_WAIT_FOR_INDEX : - needed only with QEP encoders until first
//                               index pulse
//                             - Loops shown for 'lsw=ENC_CALIBRATION_DONE' are
//                               closed in this stage
//  lsw = ENC_CALIBRATION_DONE : close all loops, position/speed/currents(Id/Iq)
//
//    NOTE:-
//       clarke1.As and clarke1.Bs are not brought out from the FCL library
//       as of library release version 0x02
//
// =============================================================================
// build level 5 subroutine for motor_1
#pragma FUNC_ALWAYS_INLINE(buildLevel5)

static inline void buildLevel5(MOTOR_Vars_t *pMotor)
{
    //
    // call QEP module to get the rotor position and speed
    //
    FCL_runQEPPosEst(pMotor);

    //
    // Fast current loop controller
    //
#if(FCL_CNTLR ==  PI_CNTLR)
    FCL_runPICtrl(pMotor);
#endif

#if(FCL_CNTLR ==  CMPLX_CNTLR)
    FCL_runComplexCtrl(pMotor);
#endif

    //
    // Measure DC Bus voltage
    //
    pMotor->FCL_params.Vdcbus = getVdc(pMotor);

    //
    // Fast current loop controller wrapper
    //
#if(FCL_CNTLR ==  PI_CNTLR)
   FCL_runPICtrlWrap(pMotor);
#endif

#if(FCL_CNTLR ==  CMPLX_CNTLR)
   FCL_runComplexCtrlWrap(pMotor);
#endif

   //
   // Alignment Routine: this routine aligns the motor to zero electrical
   // angle and in case of QEP also finds the index location and initializes
   // the angle w.r.t. the index location
   //
    if(pMotor->runMotor == MOTOR_RUN)
    {
        if(pMotor->lsw == ENC_CALIBRATION_DONE)
        {
            pMotor->IdRef = pMotor->IdRef_run;
        }
        else if(pMotor->lsw == ENC_ALIGNMENT)
        {
            // for restarting from (runMotor = STOP)
            pMotor->rc.TargetValue = 0;
            pMotor->rc.SetpointValue = 0;

            // alignment current
            pMotor->IdRef = pMotor->IdRef_start;

            // set up an alignment and hold time for shaft to settle down
            if(pMotor->cmplx_Id.ref >= pMotor->IdRef)
            {
                pMotor->alignCntr++;

                if(pMotor->alignCntr >= pMotor->alignCnt)
                {
                    pMotor->alignCntr  = 0;

                    // for QEP, spin the motor to find the index pulse
                    pMotor->lsw = ENC_WAIT_FOR_INDEX;
                    pMotor->IdRef = pMotor->IdRef_run;
                }
            }
        } // end else if(lsw == ENC_ALIGNMENT)
        else if(pMotor->lsw == ENC_WAIT_FOR_INDEX)
        {
            //
            //  Connect inputs of the RMP module and call the ramp control module
            //
            fclRampControl(&pMotor->rc);

            //
            //  Connect inputs of the RAMP GEN module and call the ramp generator module
            //
            pMotor->rg.Freq = pMotor->rc.SetpointValue;
            fclRampGen((RAMPGEN *)&pMotor->rg);

            pMotor->rc.TargetValue = pMotor->lsw1Speed *
                    (pMotor->speedRef > 0 ? 1 : -1);
        } // (lsw == ENC_WAIT_FOR_INDEX)
    }
    else
    {
        pMotor->lsw2EntryFlag = 0;
        pMotor->posCntr = 0;
        pMotor->posPtr = 0;

        pMotor->IdRef = 0;
        pMotor->rc.TargetValue = 0.0;

        FCL_resetController(pMotor);
    }

    //
    // Position encoder suite module
    //
    FCL_runQEPPosEstWrap(pMotor);

    //
    // Connect inputs of the SPEED_FR module and
    // call the speed calculation module
    //
    pMotor->speed.ElecTheta = pMotor->posElecTheta;
    runSpeedFR(&pMotor->speed);

    //
    // Connect output speed of SPEED_FR to speedWe
    //
    pMotor->speedWe = pMotor->speed.Speed;

    //
    // Connect inputs of the PID module and call the PID speed controller module
    //
    pMotor->speedLoopCount++;

    if(pMotor->speedLoopCount >= pMotor->speedLoopPrescaler)
    {
        if(pMotor->lsw == ENC_CALIBRATION_DONE)
        {
            if(!pMotor->lsw2EntryFlag)
            {
                pMotor->lsw2EntryFlag = 1;
                pMotor->rc.SetpointValue = pMotor->posMechTheta;
            }
            else
            {
                // ========== reference position setting =========
                // choose between 1 of 2 position commands
                // The user can choose between a position reference table
                // used within refPosGen() or feed it in from rg1.Out
                // Position command read from a table
                pMotor->rc.TargetValue =
                        refPosGen(pMotor->rc.TargetValue, pMotor);

                pMotor->rc.SetpointValue = pMotor->rc.TargetValue -
                             (float32_t)((int32_t)pMotor->rc.TargetValue);

                // Rolling in angle within 0 to 1pu
                if(pMotor->rc.SetpointValue < 0)
                {
                    pMotor->rc.SetpointValue += 1.0;
                }

            }

            pMotor->positionRef = pMotor->rc.SetpointValue;
            pMotor->pi_pos.Ref = pMotor->positionRef;
            pMotor->pi_pos.Fbk = pMotor->posMechTheta;
            runPIPos(&pMotor->pi_pos);

            // speed PI regulator
            pMotor->pid_spd.term.Ref = pMotor->pi_pos.Out;
            pMotor->pid_spd.term.Fbk = pMotor->speedWe;
            runPID(&pMotor->pid_spd);
        }

        pMotor->speedLoopCount = 0;
    }

    if(pMotor->lsw == ENC_ALIGNMENT)
    {
        pMotor->rc.SetpointValue = 0;  // position = 0 deg
        pMotor->pid_spd.data.d1 = 0;
        pMotor->pid_spd.data.d2 = 0;
        pMotor->pid_spd.data.i1 = 0;
        pMotor->pid_spd.data.ud = 0;
        pMotor->pid_spd.data.ui = 0;
        pMotor->pid_spd.data.up = 0;

        pMotor->pi_pos.ui = 0;
        pMotor->pi_pos.i1 = 0;

        pMotor->rg.Out = 0;
        pMotor->lsw2EntryFlag = 0;
    }

    //
    //  Setup iqref for FCL
    //
    pMotor->cmplx_Iq.ref = (pMotor->lsw == ENC_ALIGNMENT) ? 0 :
                               (pMotor->lsw == ENC_WAIT_FOR_INDEX) ?
                                 pMotor->IqRef : pMotor->pid_spd.term.Out;

    //
    //  Setup idref for FCL
    //
    pMotor->cmplx_Id.ref = ramper(pMotor->IdRef, pMotor->cmplx_Id.ref, 0.00001);

    // DEBUG: customer can remove the below code in final implementation
    // get FOC count
    getFOCCount(pMotor);

    // get FCL latency
    getFCLTime(pMotor);

    // get FOC execution time
    getFOCTime(pMotor);
    // DEBUG: customer can remove the above code in final implementation

    return;
}
#endif // (BUILDLEVEL==FCL_LEVEL5)


__interrupt void motor1ControlISR(void)
{

#if(BUILDLEVEL == FCL_LEVEL1)
    buildLevel1(&motorVars[MTR_1]);

// -----------------------------------------------------------------------------
// Connect inputs of the DATALOG module
// -----------------------------------------------------------------------------
#ifdef DLOG_ENABLE
#if defined(F28004x_DEVICE)
    dlogCh1 = motorVars[0].rg.Out;
    dlogCh2 = motorVars[0].svgen.Ta;
    dlogCh3 = motorVars[0].svgen.Tb;
    dlogCh4 = motorVars[0].svgen.Tc;

    //    Call the DATALOG update function.
    DLOG_4CH_F_FUNC(&dlog_4ch1);
#endif  // F28004x_DEVICE

#if defined(F28002x_DEVICE)
    dlogCh1 = motorVars[0].rg.Out;
    dlogCh2 = motorVars[0].svgen.Ta;

    //    Call the DATALOG update function.
    DLOG_2CH_F_FUNC(&dlog_2ch1);
#endif  // F28002x_DEVICE
#endif  // DLOG_ENABLE

#elif(BUILDLEVEL == FCL_LEVEL2)
    buildLevel2(&motorVars[MTR_1]);

// ----------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// ----------------------------------------------------------------------------
#ifdef DLOG_ENABLE
#if defined(F28004x_DEVICE)
    dlogCh1 = motorVars[0].rg.Out;
    dlogCh2 = motorVars[0].speed.ElecTheta;
    dlogCh3 = motorVars[0].clarke.As;
    dlogCh4 = motorVars[0].clarke.Bs;


    //    Call the DATALOG update function.
    DLOG_4CH_F_FUNC(&dlog_4ch1);
#endif  // F28004x_DEVICE

#if defined(F28002x_DEVICE)
    dlogCh1 = motorVars[0].rg.Out;
    dlogCh2 = motorVars[0].speed.ElecTheta;

    //    Call the DATALOG update function.
    DLOG_2CH_F_FUNC(&dlog_2ch1);
#endif  // F28002x_DEVICE
#endif  // DLOG_ENABLE

#elif(BUILDLEVEL == FCL_LEVEL3)
    buildLevel3(&motorVars[MTR_1]);

// ----------------------------------------------------------------------------
// Connect inputs of the DATALOG module
// ----------------------------------------------------------------------------
#ifdef DLOG_ENABLE
#if defined(F28004x_DEVICE)
    dlogCh1 = motorVars[0].posElecTheta;
    dlogCh2 = motorVars[0].rg.Out;
    dlogCh3 = motorVars[0].pi_iq.ref;
    dlogCh4 = motorVars[0].pi_iq.fbk;

    //    Call the DATALOG update function.
    DLOG_4CH_F_FUNC(&dlog_4ch1);
#endif  // F28004x_DEVICE

#if defined(F28002x_DEVICE)
    dlogCh1 = motorVars[0].posElecTheta;
    dlogCh2 = motorVars[0].rg.Out;

    //    Call the DATALOG update function.
    DLOG_2CH_F_FUNC(&dlog_2ch1);
#endif  // F28002x_DEVICE
#endif  // DLOG_ENABLE

#elif(BUILDLEVEL == FCL_LEVEL4)
    buildLevel46(&motorVars[MTR_1]);

// -----------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// -----------------------------------------------------------------------------
#ifdef DLOG_ENABLE
#if defined(F28004x_DEVICE)
    dlogCh1 = motorVars[0].posElecTheta;
    dlogCh2 = motorVars[0].speed.Speed;
    dlogCh3 = motorVars[1].posElecTheta;
    dlogCh4 = motorVars[1].speed.Speed;

    //    Call the DATALOG update function.
    DLOG_4CH_F_FUNC(&dlog_4ch1);
#endif  // F28004x_DEVICE

#if defined(F28002x_DEVICE)
    dlogCh1 = motorVars[0].posElecTheta;
    dlogCh2 = motorVars[0].speed.Speed;

    //    Call the DATALOG update function.
    DLOG_2CH_F_FUNC(&dlog_2ch1);
#endif  // F28002x_DEVICE
#endif  // DLOG_ENABLE

#elif(BUILDLEVEL == FCL_LEVEL5)
    buildLevel5(&motorVars[MTR_1]);

// -----------------------------------------------------------------------------
//  Connect inputs of the DATALOG module
// -----------------------------------------------------------------------------
#ifdef DLOG_ENABLE
#if defined(F28004x_DEVICE)
    dlogCh1 = motorVars[0].pi_pos.Ref;
    dlogCh2 = motorVars[0].pi_pos.Fbk;
    dlogCh3 = motorVars[0].pi_id.fbk;
    dlogCh4 = motorVars[0].pi_iq.fbk;

    //    Call the DATALOG update function.
    DLOG_4CH_F_FUNC(&dlog_4ch1);
#endif  // F28004x_DEVICE

#if defined(F28002x_DEVICE)
    dlogCh1 = motorVars[0].pi_pos.Ref;
    dlogCh2 = motorVars[0].pi_pos.Fbk;

    //    Call the DATALOG update function.
    DLOG_2CH_F_FUNC(&dlog_2ch1);
#endif  // F28002x_DEVICE
#endif  // DLOG_ENABLE

#elif(BUILDLEVEL == FCL_LEVEL6)
    buildLevel46(&motorVars[MTR_1]);

// -----------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// -----------------------------------------------------------------------------
#ifdef DLOG_ENABLE
#if defined(F28004x_DEVICE)
    dlogCh1 = motorVars[0].posElecTheta;
    dlogCh2 = motorVars[0].speed.Speed;
    dlogCh3 = motorVars[0].cmplx_Id.fbk;
    dlogCh4 = motorVars[0].cmplx_Iq.fbk;

    //    Call the DATALOG update function.
    DLOG_4CH_F_FUNC(&dlog_4ch1);
#endif  // F28004x_DEVICE

#if defined(F28002x_DEVICE)
    dlogCh1 = motorVars[0].posElecTheta;
    dlogCh2 = motorVars[0].speed.Speed;

    //    Call the DATALOG update function.
    DLOG_2CH_F_FUNC(&dlog_2ch1);
#endif  // F28002x_DEVICE
#endif  // DLOG_ENABLE

#endif

#if defined(DAC128S_ENABLE)
    DAC128S_writeData(dac128sHandle);
#endif  // DAC128S_ENABLE


    // Acknowledges an interrupt
    HAL_ackInt_M1(halMtrHandle[MTR_1]);

    motorVars[MTR_1].isrTicker++;

} // motor1ControlISR Ends Here



//  motor2ControlISR()
__interrupt void motor2ControlISR(void)
{

#if(BUILDLEVEL == FCL_LEVEL1)
    buildLevel1(&motorVars[MTR_2]);

#elif(BUILDLEVEL == FCL_LEVEL2)
    buildLevel2(&motorVars[MTR_2]);

#elif(BUILDLEVEL == FCL_LEVEL3)
    buildLevel3(&motorVars[MTR_2]);

#elif(BUILDLEVEL == FCL_LEVEL4)
    buildLevel46(&motorVars[MTR_2]);

#elif(BUILDLEVEL == FCL_LEVEL5)
    buildLevel5(&motorVars[MTR_2]);

#elif(BUILDLEVEL == FCL_LEVEL6)
    buildLevel46(&motorVars[MTR_2]);
#endif


    // Acknowledges an interrupt
    HAL_ackInt_M2(halMtrHandle[MTR_2]);

    motorVars[MTR_2].isrTicker++;
} // motor1ControlISR Ends Here

//
// run the motor control
//
void runMotorControl(MOTOR_Vars_t *pMotor, HAL_MTR_Handle mtrHandle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)mtrHandle;

    // *******************************************************
    // Current limit setting / tuning in Debug environment
    // *******************************************************
    pMotor->currentThreshHi = 2048 +
            scaleCurrentValue(pMotor->currentLimit, pMotor->currentInvSF);
    pMotor->currentThreshLo = 2048 -
            scaleCurrentValue(pMotor->currentLimit, pMotor->currentInvSF);

    HAL_setupCMPSS_DACValue(mtrHandle,
                            pMotor->currentThreshHi, pMotor->currentThreshLo);

    pMotor->Vdcbus = (pMotor->Vdcbus * 0.8) + (pMotor->FCL_params.Vdcbus * 0.2);

    if( (pMotor->Vdcbus > pMotor->VdcbusMax) ||
            (pMotor->Vdcbus < pMotor->VdcbusMin) )
    {
        pMotor->tripFlagDMC |= 0x0002;
    }
    else
    {
        pMotor->tripFlagDMC &= (0xFFFF - 0x0002);
    }

    // Check for PWM trip due to over current
    if((EPWM_getTripZoneFlagStatus(obj->pwmHandle[0]) & EPWM_TZ_FLAG_OST) ||
       (EPWM_getTripZoneFlagStatus(obj->pwmHandle[1]) & EPWM_TZ_FLAG_OST) ||
       (EPWM_getTripZoneFlagStatus(obj->pwmHandle[2]) & EPWM_TZ_FLAG_OST))
    {
        // if any EPwm's OST is set, force OST on all three to DISABLE inverter
        EPWM_forceTripZoneEvent(obj->pwmHandle[0], EPWM_TZ_FORCE_EVENT_OST);
        EPWM_forceTripZoneEvent(obj->pwmHandle[1], EPWM_TZ_FORCE_EVENT_OST);
        EPWM_forceTripZoneEvent(obj->pwmHandle[2], EPWM_TZ_FORCE_EVENT_OST);

        // Disable Driver Gate
        GPIO_writePin(pMotor->drvEnableGateGPIO, 1);

        pMotor->tripFlagDMC |= 0x0001;      // over current fault trip
    }

    pMotor->tripFlagPrev |= pMotor->tripFlagDMC;

    if(pMotor->tripFlagDMC != 0)
    {
        pMotor->runMotor = MOTOR_STOP;
        pMotor->ctrlState = CTRL_FAULT;

        // Disable Driver Gate
        GPIO_writePin(pMotor->drvEnableGateGPIO, 1);
    }

    if((pMotor->tripFlagDMC != 0) && (pMotor->clearTripFlagDMC == true))
    {
        pMotor->tripCountDMC++;
    }

    // If clear cmd received, reset PWM trip
    if(pMotor->clearTripFlagDMC == true)
    {
        // clear HLATCH - (not in TRIP gen path)
        // clear LLATCH - (not in TRIP gen path)
        CMPSS_clearFilterLatchHigh(obj->cmpssHandle[0]);

        CMPSS_clearFilterLatchHigh(obj->cmpssHandle[1]);
        CMPSS_clearFilterLatchLow(obj->cmpssHandle[1]);

        CMPSS_clearFilterLatchLow(obj->cmpssHandle[2]);

        // clear OST & DCAEVT1 flags
        EPWM_clearTripZoneFlag(obj->pwmHandle[0], HAL_TZFLAG_ALL);

        EPWM_clearTripZoneFlag(obj->pwmHandle[1], HAL_TZFLAG_ALL);

        EPWM_clearTripZoneFlag(obj->pwmHandle[2], HAL_TZFLAG_ALL);

        // clear the ocp
        pMotor->tripFlagDMC = 0;
        pMotor->clearTripFlagDMC = 0;
        pMotor->ctrlState = CTRL_STOP;
        pMotor->lsw = ENC_ALIGNMENT;
    }

    if(pMotor->ctrlState == CTRL_RUN)
    {
        if(pMotor->runMotor == MOTOR_STOP)
        {
            pMotor->runMotor = MOTOR_RUN;

            // Enable Driver Gate
            GPIO_writePin(pMotor->drvEnableGateGPIO, 0);
        }
    }
    else
    {
        if(pMotor->runMotor == MOTOR_RUN)
        {
            pMotor->runMotor = MOTOR_STOP;

            // Disable Driver Gate
            GPIO_writePin(pMotor->drvEnableGateGPIO, 1);
        }
    }

    return;
}

//------------------------------------------------------------------------------
// runSyncControl()
void runSyncControl(void)
{
    if(flagSyncRun == true)
    {
        if((motorVars[0].tripFlagDMC == 0) && (motorVars[1].tripFlagDMC == 0))
        {

#if(BUILDLEVEL != FCL_LEVEL5)
            motorVars[0].speedRef = speedRef;
            motorVars[1].speedRef = speedRef;
#endif

#if(BUILDLEVEL == FCL_LEVEL3)
            motorVars[0].IdRef_run = IdRef;
            motorVars[1].IdRef_run = IdRef;

            motorVars[0].IqRef = IqRef;
            motorVars[1].IqRef = IqRef;
#endif

            motorVars[0].ctrlState = ctrlState;
            motorVars[1].ctrlState = ctrlState;

        }
        else
        {
            motorVars[0].ctrlState = CTRL_STOP;
            motorVars[1].ctrlState = CTRL_STOP;
            motorVars[0].speedRef = 0.0;
            motorVars[1].speedRef = 0.0;
        }

        if((motorVars[0].runMotor == MOTOR_RUN) &&
                (motorVars[1].runMotor == MOTOR_RUN))
        {
            runMotor = MOTOR_RUN;
        }
        else
        {
            runMotor= MOTOR_STOP;
        }
    }

    return;
}

//*****************************************************************************
//*****************************************************************************
// Build level 6 : SFRA support functions
//*****************************************************************************
//*****************************************************************************
#if(BUILDLEVEL == FCL_LEVEL6)
// *************************************************************************
// Using SFRA tool :
// =================
//      - INJECT noise
//      - RUN the controller
//      - CAPTURE or COLLECT the controller output
// From a controller analysis standpoint, this sequence will reveal the
// output of controller for a given input, and therefore, good for analysis
// *************************************************************************
void injectSFRA(void)
{
    sfraNoiseD = 0.0f;
    sfraNoiseQ = 0.0f;
    sfraNoiseW = 0.0f;

    if(sfraTestLoop == SFRA_TEST_D_AXIS)
    {
        sfraNoiseD = SFRA_F32_inject(0.0);
    }
    else if(sfraTestLoop == SFRA_TEST_Q_AXIS)
    {
        sfraNoiseQ = SFRA_F32_inject(0.0);
    }
    else if(sfraTestLoop == SFRA_TEST_SPEEDLOOP)
    {
        sfraNoiseW = SFRA_F32_inject(0.0);
    }

    return;
}

// ****************************************************************************
void collectSFRA(MOTOR_Vars_t *pMotor)
{
    if(sfraTestLoop == SFRA_TEST_D_AXIS)
    {
        SFRA_F32_collect(&pMotor->cmplx_Id.out,
                         &pMotor->cmplx_Id.fbk);
    }
    else if(sfraTestLoop == SFRA_TEST_Q_AXIS)
    {
        SFRA_F32_collect(&pMotor->cmplx_Iq.out,
                         &pMotor->cmplx_Iq.fbk);
    }
    else if(sfraTestLoop == SFRA_TEST_SPEEDLOOP)
    {
        SFRA_F32_collect(&pMotor->pid_spd.term.Out,
                         &pMotor->pid_spd.term.Fbk);
    }

    return;
}
#endif

//
// End of Code
//
