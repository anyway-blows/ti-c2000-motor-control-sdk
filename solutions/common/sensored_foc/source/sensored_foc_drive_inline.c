//#############################################################################
//
// FILE:    multi_axis_slave_drive.c
//
// TITLE:   multi-axis servo drive over FSI on the related kits
//
// Group:   C2000
//
// Target Family: F28004x/F28002x
//
//#############################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:28 CDT 2021 $
// $Copyright:
// Copyright (C) 2017-2020 Texas Instruments Incorporated - http://www.ti.com/
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
#include "motor_drive_settings.h"
#include "sensored_foc_main.h"

//
// Functions
//
#ifdef _FLASH
#pragma CODE_SECTION(motorControlISR, ".TI.ramfunc");
#endif  // _FLASH

//
//  Prototype statements for Local Functions
//
#pragma INTERRUPT (motorControlISR, HPI)
extern __interrupt void motorControlISR(void);

//
// SFRA utility functions
//
#if(BUILDLEVEL == FCL_LEVEL6)
extern void injectSFRA(void);
extern void collectSFRA(MOTOR_Vars_t *pMotor);
#endif  // (BUILDLEVEL == FCL_LEVEL6)

//
// Global variables used in this system
//

// motor drive variables
MOTOR_Vars_t motorVars;

HAL_Handle halHandle;    //!< the handle for the hardware abstraction layer
HAL_Obj    hal;          //!< the hardware abstraction layer object

// flag variables
volatile uint16_t tempPIEIER;

// Variables for Field Oriented Control
float32_t VdTesting = 0.0;          // Vd reference (pu)
float32_t VqTesting = 0.15;         // Vq reference (pu)
#pragma DATA_SECTION(VdTesting, "ramInitVars");
#pragma DATA_SECTION(VqTesting, "ramInitVars");

// Variables for position reference generation and control
float32_t posArray[8] = {6.5, -6.5, 8.5, -8.5, 9.0, -9.0, 8.0, -8.0};
float32_t posPtrMax = 4;
#pragma DATA_SECTION(posArray, "ramInitVars");
#pragma DATA_SECTION(posPtrMax, "ramInitVars");

// Variables for Datalog module
#ifdef DLOG_ENABLE
float32_t DBUFF_4CH1[DBUFF_SIZE_NUM];
float32_t DBUFF_4CH2[DBUFF_SIZE_NUM];
float32_t DBUFF_4CH3[DBUFF_SIZE_NUM];
float32_t DBUFF_4CH4[DBUFF_SIZE_NUM];

float32_t dlogCh1;
float32_t dlogCh2;
float32_t dlogCh3;
float32_t dlogCh4;

// Create an instance of DATALOG Module
DLOG_4CH_F dlog_4ch1;
#endif  // DLOG_ENABLE

// Variables for SFRA module
#if((BUILDLEVEL == FCL_LEVEL6) || (BUILDLEVEL == FCL_LEVEL9))
SFRATest_e  sfraTestLoop;  //speedLoop;
uint32_t    sfraCollectStart;
float32_t   sfraNoiseD;
float32_t   sfraNoiseQ;
float32_t   sfraNoiseW;
#endif


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
// build level 1 subroutine for motor
static inline void buildLevel1(void)
{
    //
    // call QEP module to get the rotor position and speed
    //
    FCL_runQEPPosEst(&motorVars);

    //
    // control force angle generation based on 'runMotor'
    //
    if(motorVars.ctrlStateUse == CTRL_RUN)
    {
        motorVars.rc.TargetValue = motorVars.speedRef;
        motorVars.ipark.Ds = VdTesting;
        motorVars.ipark.Qs = VqTesting;
    }
    else
    {
        motorVars.rc.TargetValue = 0;
        motorVars.rc.SetpointValue = 0;
        motorVars.ipark.Ds = 0.0;
        motorVars.ipark.Qs = 0.0;
    }
    //
    // Connect inputs of the RMPCTRL module and call the ramp control module
    //
    fclRampControl(&motorVars.rc);

    //
    // Connect inputs of the RAMPGEN module and call the ramp generator module
    //
    motorVars.rg.Freq = motorVars.rc.SetpointValue;
    fclRampGen((RAMPGEN *)&motorVars.rg);

    //
    // Connect inputs of the INV_PARK module and call the inverse park module
    //
    motorVars.ipark.Sine = __sinpuf32(motorVars.rg.Out);
    motorVars.ipark.Cosine = __cospuf32(motorVars.rg.Out);
    runIPark(&motorVars.ipark);

    //
    // Call QEP module
    //
    FCL_runQEPPosEstWrap(&motorVars);

    //
    //  Measure DC Bus voltage
    //
    motorVars.FCL_params.Vdcbus = getVdc(&motorVars);

    //
    // Connect inputs of the SVGEN_DQ module and call the svgen module
    //
    motorVars.svgen.Ualpha = motorVars.ipark.Alpha;
    motorVars.svgen.Ubeta  = motorVars.ipark.Beta;
    runSVGenDQ(&motorVars.svgen);

    //
    // Computed Duty and Write to CMPA register
    //
    EPWM_setCounterCompareValue(motorVars.pwmBaseU, EPWM_COUNTER_COMPARE_A,
                   (uint16_t)((M_INV_PWM_HALF_TBPRD * motorVars.svgen.Tc) +
                               M_INV_PWM_HALF_TBPRD));

    EPWM_setCounterCompareValue(motorVars.pwmBaseV, EPWM_COUNTER_COMPARE_A,
                   (uint16_t)((M_INV_PWM_HALF_TBPRD * motorVars.svgen.Ta) +
                               M_INV_PWM_HALF_TBPRD));

    EPWM_setCounterCompareValue(motorVars.pwmBaseW, EPWM_COUNTER_COMPARE_A,
                   (uint16_t)((M_INV_PWM_HALF_TBPRD * motorVars.svgen.Tb) +
                               M_INV_PWM_HALF_TBPRD));
    return;
}
#endif // (BUILDLEVEL==FCL_LEVEL1)

//
//****************************************************************************
// INCRBUILD 3
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
// build level 2 subroutine for motor_1
static inline void buildLevel2(void)
{
    //
    // call QEP module to get the rotor position and speed
    //
    FCL_runQEPPosEst(&motorVars);

    // -------------------------------------------------------------------------
    // Alignment Routine: this routine aligns the motor to zero electrical
    // angle and in case of QEP also finds the index location and initializes
    // the angle w.r.t. the index location
    // -------------------------------------------------------------------------
    if(motorVars.ctrlStateUse == CTRL_RUN)
    {
        // Connect inputs of the RMP module and call the ramp control module
        motorVars.rc.TargetValue = motorVars.speedRef;

        if(motorVars.lsw == ENC_CALIBRATION_DONE)
        {
            motorVars.ipark.Ds = VdTesting;
            motorVars.ipark.Qs = VqTesting;
        }
        else if(motorVars.lsw == ENC_ALIGNMENT)
        {
            // for restarting from (runMotor = STOP)
            motorVars.rc.TargetValue = 0;
            motorVars.rc.SetpointValue = 0;

            // for QEP, spin the motor to find the index pulse
            motorVars.lsw = ENC_WAIT_FOR_INDEX;

            motorVars.ipark.Ds = VdTesting;
            motorVars.ipark.Qs = VqTesting;
        } // (lsw == ENC_ALIGNMENT)
    }
    else
    {
        motorVars.lsw = ENC_ALIGNMENT;
        motorVars.rc.TargetValue = 0.0;
        motorVars.IdRef = 0;
        motorVars.cmplx_Id.ref = motorVars.IdRef;

        FCL_resetController(&motorVars);

        motorVars.ipark.Ds = 0.0;
        motorVars.ipark.Qs = 0.0;
    }


    fclRampControl(&motorVars.rc);

    //
    // Connect inputs of the RAMP GEN module and call the ramp generator module
    //
    motorVars.rg.Freq = motorVars.rc.SetpointValue;
    fclRampGen((RAMPGEN *)&motorVars.rg);

// ----------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5)
//  to (-1,+1). Connect inputs of the CLARKE module and call the clarke
//  transformation module
// ----------------------------------------------------------------------------
    //wait on ADC EOC
#if(DRIVER_MODULE == BITFIELD_MODE)
    while(motorVars.pADCIntFlag->bit.ADCINT1 == 0);
#else
    while(ADC_getInterruptStatus(motorVars.adcBaseW, ADC_INT_NUMBER1) == 0);
#endif

    motorVars.clarke.As = (float32_t)M_IFB_V_PPB *
            motorVars.FCL_params.adcPPBScale;

    motorVars.clarke.Bs = (float32_t)M_IFB_W_PPB *
            motorVars.FCL_params.adcPPBScale;

    runClarke(&motorVars.clarke);

    //
    //  Measure DC Bus voltage
    //
    motorVars.FCL_params.Vdcbus = getVdc(&motorVars);

    //
    // Connect inputs of the PARK module and call the park module
    //
    motorVars.park.Alpha  = motorVars.clarke.Alpha;
    motorVars.park.Beta   = motorVars.clarke.Beta;
    motorVars.park.Angle  = motorVars.rg.Out;
    
    motorVars.park.Sine   = __sinpuf32(motorVars.park.Angle);
    motorVars.park.Cosine = __cospuf32(motorVars.park.Angle);
    
    runPark(&motorVars.park);

    //
    // Connect inputs of the INV_PARK module and call the inverse park module
    //
    motorVars.ipark.Sine = motorVars.park.Sine;
    motorVars.ipark.Cosine = motorVars.park.Cosine;
    runIPark(&motorVars.ipark);

    //
    // Position encoder suite module
    //
    FCL_runQEPPosEstWrap(&motorVars);

    //
    // Connect inputs of the SPEED_FR module and call the speed calculation module
    //
    motorVars.speed.ElecTheta = motorVars.posElecTheta;
    runSpeedFR(&motorVars.speed);

    //
    // Connect output speed of SPEED_FR to speedWe
    //
    motorVars.speedWe = motorVars.speed.Speed;

    //
    // Connect inputs of the SVGEN_DQ module and call the space-vector gen. module
    //
    motorVars.svgen.Ualpha = motorVars.ipark.Alpha;
    motorVars.svgen.Ubeta  = motorVars.ipark.Beta;
    runSVGenDQ(&motorVars.svgen);

// ----------------------------------------------------------------------------
//  Computed Duty and Write to CMPA register
// ----------------------------------------------------------------------------
    EPWM_setCounterCompareValue(motorVars.pwmBaseU, EPWM_COUNTER_COMPARE_A,
                   (uint16_t)((M_INV_PWM_HALF_TBPRD * motorVars.svgen.Tc) +
                               M_INV_PWM_HALF_TBPRD));

    EPWM_setCounterCompareValue(motorVars.pwmBaseV, EPWM_COUNTER_COMPARE_A,
                   (uint16_t)((M_INV_PWM_HALF_TBPRD * motorVars.svgen.Ta) +
                               M_INV_PWM_HALF_TBPRD));

    EPWM_setCounterCompareValue(motorVars.pwmBaseW, EPWM_COUNTER_COMPARE_A,
                   (uint16_t)((M_INV_PWM_HALF_TBPRD * motorVars.svgen.Tb) +
                               M_INV_PWM_HALF_TBPRD));
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

// build level 3 subroutine for motor
static inline void buildLevel3(void)
{
    //
    // call QEP module to get the rotor position and speed
    //
    FCL_runQEPPosEst(&motorVars);

    //
    // Fast current loop controller
    //
#if(FCL_CNTLR ==  PI_CNTLR)
    FCL_runPICtrl(&motorVars);
#endif

#if(FCL_CNTLR ==  CMPLX_CNTLR)
    FCL_runComplexCtrl(&motorVars);
#endif

    //
    // Measure DC Bus voltage using SDFM Filter3
    //
    motorVars.FCL_params.Vdcbus = getVdc(&motorVars);

    //
    // Fast current loop controller wrapper
    //
#if(FCL_CNTLR ==  PI_CNTLR)
    FCL_runPICtrlWrap(&motorVars);
#endif

#if(FCL_CNTLR ==  CMPLX_CNTLR)
    FCL_runComplexCtrlWrap(&motorVars);
#endif

    //
    // Alignment Routine: this routine aligns the motor to zero electrical
    // angle and in case of QEP also finds the index location and initializes
    // the angle w.r.t. the index location
    //
    if(motorVars.ctrlStateUse == CTRL_RUN)
    {
        // Connect inputs of the RMP module and call the ramp control module
        motorVars.rc.TargetValue = motorVars.speedRef;

        if(motorVars.lsw == ENC_CALIBRATION_DONE)
        {
            motorVars.IdRef = motorVars.IdRef_run;
        }
        else if(motorVars.lsw == ENC_ALIGNMENT)
        {
            motorVars.rc.TargetValue = 0;
            motorVars.rc.SetpointValue = 0;

            // alignment current
            motorVars.IdRef = motorVars.IdRef_start;

            // set up an alignment and hold time for shaft to settle down
            if(motorVars.cmplx_Id.ref >= motorVars.IdRef)
            {
                motorVars.alignCntr++;

                if(motorVars.alignCntr >= motorVars.alignCnt)
                {
                    motorVars.alignCntr  = 0;

                    // for QEP, spin the motor to find the index pulse
                    motorVars.lsw = ENC_WAIT_FOR_INDEX;
                    motorVars.IqRef = motorVars.IqRef_start;
                }
            }

        } // end else if(lsw == ENC_ALIGNMENT)
    }
    else
    {
        motorVars.lsw = ENC_ALIGNMENT;
        motorVars.cmplx_Id.ref = 0;
        motorVars.IdRef = 0;
        motorVars.rc.TargetValue = 0.0;

        FCL_resetController(&motorVars);
    }

    fclRampControl(&motorVars.rc);

    //
    // Connect inputs of the RAMP GEN module and call the ramp generator module
    //
    motorVars.rg.Freq = motorVars.rc.SetpointValue;
    fclRampGen((RAMPGEN *)&motorVars.rg);

    //
    // Position encoder suite module
    //
    FCL_runQEPPosEstWrap(&motorVars);

    //
    // Connect inputs of the SPEED_FR module and
    // call the speed calculation module
    //
    motorVars.speed.ElecTheta = motorVars.posElecTheta;
    runSpeedFR(&motorVars.speed);

    //
    // Connect output speed of SPEED_FR to speedWe
    //
    motorVars.speedWe = motorVars.speed.Speed;

    //
    // setup iqref for FCL
    //
    motorVars.cmplx_Iq.ref =
           (motorVars.lsw == ENC_ALIGNMENT) ? 0 : motorVars.IqRef;

    //
    // setup idref for FCL
    //
    motorVars.cmplx_Id.ref =
           ramper(motorVars.IdRef, motorVars.cmplx_Id.ref, 0.00001);

    // DEBUG: customer can remove the below code in final implementation
    // get FOC count
    getFOCCount(&motorVars);

    // get FCL latency
    getFCLTime(&motorVars);

    // get FOC execution time
    getFOCTime(&motorVars);
    // DEBUG: customer can remove the above code in final implementation

    return;
}
#endif // (BUILDLEVEL==FCL_LEVEL3)

//
//****************************************************************************
// INCRBUILD 4
//****************************************************************************
//
#if((BUILDLEVEL == FCL_LEVEL4) || (BUILDLEVEL == FCL_LEVEL6))
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
static inline void buildLevel46(void)
{
    //
    // call QEP module to get the rotor position and speed
    //
    FCL_runQEPPosEst(&motorVars);

    //
    // Fast current loop controller
    //
#if(FCL_CNTLR ==  PI_CNTLR)
    FCL_runPICtrl(&motorVars);
#endif

#if(FCL_CNTLR ==  CMPLX_CNTLR)
    FCL_runComplexCtrl(&motorVars);
#endif

    //
    // Measure DC Bus voltage
    //
    motorVars.FCL_params.Vdcbus = getVdc(&motorVars);

    //
    // Fast current loop controller wrapper
    //
#if(FCL_CNTLR ==  PI_CNTLR)
    FCL_runPICtrlWrap(&motorVars);
#endif

#if(FCL_CNTLR ==  CMPLX_CNTLR)
    FCL_runComplexCtrlWrap(&motorVars);
#endif

    //
    // Alignment Routine: this routine aligns the motor to zero electrical
    // angle and in case of QEP also finds the index location and initializes
    // the angle w.r.t. the index location
    //
    if(motorVars.ctrlStateUse == CTRL_RUN)
    {
        if(motorVars.lsw == ENC_CALIBRATION_DONE)
        {
            motorVars.ctrlIdRef = motorVars.IdRef_run;
            motorVars.rc.TargetValue = motorVars.speedRef;
        } // (lsw == ENC_CALIBRATION_DONE)
        else if(motorVars.lsw == ENC_ALIGNMENT)
        {
            motorVars.rc.TargetValue = 0;
            motorVars.rc.SetpointValue = 0;

            // alignment current
            motorVars.ctrlIdRef = motorVars.IdRef_start;

            // set up an alignment and hold time for shaft to settle down
            if(motorVars.IdRef >= motorVars.ctrlIdRef)
            {
                motorVars.alignCntr++;

                if(motorVars.alignCntr >= motorVars.alignCnt)
                {
                    motorVars.alignCntr  = 0;

                    // for QEP, spin the motor to find the index pulse
                    motorVars.lsw = ENC_WAIT_FOR_INDEX;
                    motorVars.ctrlIdRef = motorVars.IdRef_run;

                    if(motorVars.speedRef >= 0.0)
                    {
                        motorVars.IqRef = motorVars.IqRef_start;
                    }
                    else
                    {
                        motorVars.IqRef = -motorVars.IqRef_start;
                    }
                }
            }
        } // (lsw == ENC_ALIGNMENT)
        else if(motorVars.lsw == ENC_WAIT_FOR_INDEX)
        {
            //
            //  Connect inputs of the RAMP GEN module and call the ramp generator module
            //
            motorVars.rg.Freq = motorVars.rc.SetpointValue;
            fclRampGen((RAMPGEN *)&motorVars.rg);

            motorVars.rc.TargetValue = motorVars.lsw1Speed *
                    (motorVars.speedRef > 0 ? 1 : -1);
        } // (lsw == ENC_WAIT_FOR_INDEX)
    }
    else
    {
        motorVars.IdRef = 0;
        motorVars.IqRef = 0;

        motorVars.ctrlIdRef = 0.0;

        motorVars.rc.TargetValue = 0.0;

        FCL_resetController(&motorVars);

        if(motorVars.lsw != ENC_CALIBRATION_DONE)
        {
            motorVars.lsw = ENC_ALIGNMENT;
        }
    }

    //
    //  Connect inputs of the RMP module and call the ramp control module
    //
    fclRampControl(&motorVars.rc);

    //
    // Position encoder suite module
    //
    FCL_runQEPPosEstWrap(&motorVars);

    //
    // Connect inputs of the SPEED_FR module and
    // call the speed calculation module
    //
    motorVars.speed.ElecTheta = motorVars.posElecTheta;
    runSpeedFR(&motorVars.speed);

    //
    // Connect output speed of SPEED_FR to speedWe
    //
    motorVars.speedWe = motorVars.speed.Speed;

#if(BUILDLEVEL == FCL_LEVEL6)
    //
    // SFRA collect routine,
    // only to be called after SFRA inject has occurred 1st
    //
    if(sfraCollectStart)
    {
        collectSFRA(&motorVars);    // Collect noise feedback from loop
    }

    //
    //  SFRA injection
    //
    injectSFRA();               // create SFRA Noise per 'sfraTestLoop'

    sfraCollectStart = 1;       // enable SFRA data collection
#endif

    //
    // Connect inputs of the PI module and call the PID speed controller module
    //
    motorVars.speedLoopCount++;

#if(SPD_CNTLR == SPD_PID_CNTLR)
    if(motorVars.speedLoopCount >= motorVars.speedLoopPrescaler)
    {

#if(BUILDLEVEL == FCL_LEVEL6)
        // SFRA Noise injection in speed loop
        motorVars.ctrlSpeedRef = motorVars.rc.SetpointValue + sfraNoiseW;
#else   // (BUILDLEVEL == FCL_LEVEL6)
        motorVars.ctrlSpeedRef = motorVars.rc.SetpointValue;  //speedRef;
#endif  // (BUILDLEVEL == FCL_LEVEL4)

        motorVars.pid_spd.term.Ref = motorVars.ctrlSpeedRef;
        motorVars.pid_spd.term.Fbk = motorVars.speedWe;
        runPID(&motorVars.pid_spd);

        motorVars.speedLoopCount = 0;
    }

    if((motorVars.lsw != ENC_CALIBRATION_DONE) ||
            (motorVars.ctrlStateUse != CTRL_RUN))
    {
        motorVars.pid_spd.data.d1 = 0;
        motorVars.pid_spd.data.d2 = 0;
        motorVars.pid_spd.data.i1 = 0;
        motorVars.pid_spd.data.ud = 0;
        motorVars.pid_spd.data.ui = 0;
        motorVars.pid_spd.data.up = 0;
        motorVars.pid_spd.term.Out = 0;
    }

    motorVars.ctrlSpdOut = motorVars.pid_spd.term.Out;
#endif  // (SPD_CNTLR == SPD_PID_CNTLR)

#if(SPD_CNTLR == SPD_DCL_CNTLR)
    if(motorVars.speedLoopCount >= motorVars.speedLoopPrescaler)
    {

#if(BUILDLEVEL == FCL_LEVEL6)
        // SFRA Noise injection in speed loop
        motorVars.ctrlSpeedRef = motorVars.rc.SetpointValue + sfraNoiseW;
#else  // (BUILDLEVEL == FCL_LEVEL6)
        motorVars.ctrlSpeedRef = motorVars.rc.SetpointValue;
#endif // (BUILDLEVEL == FCL_LEVEL4)
        motorVars.ctrlSpdOut = DCL_runPI_C1(&motorVars.dcl_spd,
                                            motorVars.ctrlSpeedRef,
                                            motorVars.speedWe);

        motorVars.speedLoopCount = 0;
    }

    if((motorVars.lsw != ENC_CALIBRATION_DONE) ||
            (motorVars.ctrlStateUse != CTRL_RUN))
    {
        DCL_resetPI(&motorVars.dcl_spd);
    }

#endif  // (SPD_CNTLR == SPD_DCL_CNTLR)

#if(SPD_CNTLR == SPD_NLPID_CNTLR)
    if(motorVars.speedLoopCount >= motorVars.speedLoopPrescaler)
    {

#if(BUILDLEVEL == FCL_LEVEL6)
        // SFRA Noise injection in speed loop
        motorVars.ctrlSpeedRef = motorVars.rc.SetpointValue + sfraNoiseW;
#else   // (BUILDLEVEL == FCL_LEVEL6)
        motorVars.ctrlSpeedRef = motorVars.rc.SetpointValue;
#endif  // (BUILDLEVEL == FCL_LEVEL4)
        motorVars.ctrlSpdOut = DCL_runNLPID_C3(&motorVars.nlpid_spd,
                                              motorVars.ctrlSpeedRef,
                                              motorVars.speedWe, 1.0f);

        motorVars.speedLoopCount = 0;
    }

    if((motorVars.lsw != ENC_CALIBRATION_DONE) ||
            (motorVars.ctrlStateUse != CTRL_RUN))
    {
        motorVars.nlpid_spd.d2 = 0.0;
        motorVars.nlpid_spd.d3 = 0.0;
        motorVars.nlpid_spd.i7 = 0.0;
        motorVars.nlpid_spd.i16 = 1.0f;
    }

#endif  // (SPD_CNTLR == SPD_NLPID_CNTLR)

    //
    //    setup iqref and idref for FCL
    //
#if(BUILDLEVEL == FCL_LEVEL6)
    // SFRA Noise injection in Q axis, setup iqref
    motorVars.cmplx_Iq.ref =
            (motorVars.lsw == ENC_ALIGNMENT) ? 0 :
                    (motorVars.lsw == ENC_WAIT_FOR_INDEX) ?
                            motorVars.IqRef :
                            (motorVars.ctrlSpdOut + sfraNoiseQ);

    // SFRA Noise injection in D axis, setup idref
    motorVars.IdRef =
            ramper(motorVars.ctrlIdRef, motorVars.IdRef, 0.00001);

    motorVars.cmplx_Id.ref = motorVars.IdRef + sfraNoiseD;
#else   // if(BUILDLEVEL == FCL_LEVEL4)
    //
    // Setup iqref for FCL
    //
    motorVars.cmplx_Iq.ref = (motorVars.lsw == ENC_ALIGNMENT) ? 0 :
                               (motorVars.lsw == ENC_WAIT_FOR_INDEX) ?
                                 motorVars.IqRef : motorVars.ctrlSpdOut;

    //
    // Setup idref for FCL
    //
    motorVars.IdRef = ramper(motorVars.ctrlIdRef,
                                    motorVars.IdRef, 0.00001);

    motorVars.cmplx_Id.ref = motorVars.IdRef;
#endif

    // DEBUG: customer can remove the below code in final implementation
    // get FOC count
    getFOCCount(&motorVars);

    // get FCL latency
    getFCLTime(&motorVars);

    // get FOC execution time
    getFOCTime(&motorVars);
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
// build level 5 subroutine for motor
static inline void buildLevel5(void)
{
    //
    // call QEP module to get the rotor position and speed
    //
    FCL_runQEPPosEst(&motorVars);

    //
    // Fast current loop controller
    //
#if(FCL_CNTLR ==  PI_CNTLR)
    FCL_runPICtrl(&motorVars);
#endif

#if(FCL_CNTLR ==  CMPLX_CNTLR)
    FCL_runComplexCtrl(&motorVars);
#endif

    //
    // Measure DC Bus voltage
    //
    motorVars.FCL_params.Vdcbus = getVdc(&motorVars);

    //
    // Fast current loop controller wrapper
    //
#if(FCL_CNTLR ==  PI_CNTLR)
   FCL_runPICtrlWrap(&motorVars);
#endif

#if(FCL_CNTLR ==  CMPLX_CNTLR)
   FCL_runComplexCtrlWrap(&motorVars);
#endif

   //
   // Alignment Routine: this routine aligns the motor to zero electrical
   // angle and in case of QEP also finds the index location and initializes
   // the angle w.r.t. the index location
   //
    if(motorVars.ctrlStateUse == CTRL_RUN)
    {
        if(motorVars.lsw == ENC_CALIBRATION_DONE)
        {
            motorVars.ctrlIdRef = motorVars.IdRef_run;
        }
        else if(motorVars.lsw == ENC_ALIGNMENT)
        {
            // for restarting from (runMotor = STOP)
            motorVars.rc.TargetValue = 0;
            motorVars.rc.SetpointValue = 0;

            // alignment current
            motorVars.ctrlIdRef = motorVars.IdRef_start;

            // set up an alignment and hold time for shaft to settle down
            if(motorVars.cmplx_Id.ref >= motorVars.ctrlIdRef)
            {
                motorVars.alignCntr++;

                if(motorVars.alignCntr >= motorVars.alignCnt)
                {
                    motorVars.alignCntr  = 0;

                    // for QEP, spin the motor to find the index pulse
                    motorVars.lsw = ENC_WAIT_FOR_INDEX;
                    motorVars.ctrlIdRef = motorVars.IdRef_run;
                }
            }
        } // end else if(lsw == ENC_ALIGNMENT)
        else if(motorVars.lsw == ENC_WAIT_FOR_INDEX)
        {
            //
            //  Connect inputs of the RMP module and call the ramp control module
            //
            fclRampControl(&motorVars.rc);

            //
            //  Connect inputs of the RAMP GEN module and call the ramp generator module
            //
            motorVars.rg.Freq = motorVars.rc.SetpointValue;
            fclRampGen((RAMPGEN *)&motorVars.rg);

            motorVars.rc.TargetValue = motorVars.lsw1Speed *
                    (motorVars.speedRef > 0 ? 1 : -1);
        } // (lsw == ENC_WAIT_FOR_INDEX)
    }
    else
    {
        motorVars.lsw2EntryFlag = 0;
        motorVars.posCntr = 0;
        motorVars.posPtr = 0;

        motorVars.ctrlIdRef = 0;
        motorVars.rc.TargetValue = 0.0;

        FCL_resetController(&motorVars);
    }

    //
    // Position encoder suite module
    //
    FCL_runQEPPosEstWrap(&motorVars);

    //
    // Connect inputs of the SPEED_FR module and
    // call the speed calculation module
    //
    motorVars.speed.ElecTheta = motorVars.posElecTheta;
    runSpeedFR(&motorVars.speed);

    //
    // Connect output speed of SPEED_FR to speedWe
    //
    motorVars.speedWe = motorVars.speed.Speed;

    //
    // Connect inputs of the PID module and call the PID speed controller module
    //
    motorVars.speedLoopCount++;

#if(SPD_CNTLR == SPD_PID_CNTLR)
    if(motorVars.speedLoopCount >= motorVars.speedLoopPrescaler)
    {
        if(motorVars.lsw == ENC_CALIBRATION_DONE)
        {
            if(!motorVars.lsw2EntryFlag)
            {
                motorVars.lsw2EntryFlag = 1;
                motorVars.rc.SetpointValue = motorVars.posMechTheta;
            }
            else
            {
                // ========== reference position setting =========
                // choose between 1 of 2 position commands
                // The user can choose between a position reference table
                // used within refPosGen() or feed it in from rg1.Out
                // Position command read from a table
                motorVars.rc.TargetValue =
                        refPosGen(motorVars.rc.TargetValue, &motorVars);

                motorVars.rc.SetpointValue = motorVars.rc.TargetValue -
                             (float32_t)((int32_t)motorVars.rc.TargetValue);

                // Rolling in angle within 0 to 1pu
                if(motorVars.rc.SetpointValue < 0)
                {
                    motorVars.rc.SetpointValue += 1.0;
                }

            }

            motorVars.ctrlPosRef = motorVars.rc.SetpointValue;
            motorVars.pi_pos.Ref = motorVars.ctrlPosRef;
            motorVars.pi_pos.Fbk = motorVars.posMechTheta;
            runPIPos(&motorVars.pi_pos);

            // speed PI regulator
            motorVars.pid_spd.term.Ref = motorVars.pi_pos.Out;
            motorVars.pid_spd.term.Fbk = motorVars.speedWe;
            runPID(&motorVars.pid_spd);
        }

        motorVars.speedLoopCount = 0;
    }

    if(motorVars.lsw == ENC_ALIGNMENT)
    {
        motorVars.rc.SetpointValue = 0;  // position = 0 deg
        motorVars.pid_spd.data.d1 = 0;
        motorVars.pid_spd.data.d2 = 0;
        motorVars.pid_spd.data.i1 = 0;
        motorVars.pid_spd.data.ud = 0;
        motorVars.pid_spd.data.ui = 0;
        motorVars.pid_spd.data.up = 0;

        motorVars.pi_pos.ui = 0;
        motorVars.pi_pos.i1 = 0;

        motorVars.rg.Out = 0;
        motorVars.lsw2EntryFlag = 0;
    }

    motorVars.ctrlSpdOut = motorVars.pid_spd.term.Out;
#endif  // (SPD_CNTLR == SPD_PID_CNTLR)

#if(SPD_CNTLR == SPD_DCL_CNTLR)
    if(motorVars.speedLoopCount >= motorVars.speedLoopPrescaler)
    {
        if(motorVars.lsw == ENC_CALIBRATION_DONE)
        {
            if(!motorVars.lsw2EntryFlag)
            {
                motorVars.lsw2EntryFlag = 1;
                motorVars.rc.SetpointValue = motorVars.posMechTheta;
            }
            else
            {
                // ========== reference position setting =========
                // choose between 1 of 2 position commands
                // The user can choose between a position reference table
                // used within refPosGen() or feed it in from rg1.Out
                // Position command read from a table
                motorVars.rc.TargetValue =
                        refPosGen(motorVars.rc.TargetValue, &motorVars);

                motorVars.rc.SetpointValue = motorVars.rc.TargetValue -
                             (float32_t)((int32_t)motorVars.rc.TargetValue);

                // Rolling in angle within 0.0 to 1.0pu
                if(motorVars.rc.SetpointValue < 0.0)
                {
                    motorVars.rc.SetpointValue += 1.0;
                }

                motorVars.ctrlPosRef = motorVars.rc.SetpointValue;
            }

            motorVars.ctrlPosRef = motorVars.rc.SetpointValue;

            // speed PI regulator
            motorVars.ctrlSpeedRef = DCL_runPI_C1(&motorVars.dcl_pos,
                                                  motorVars.ctrlPosRef,
                                                  motorVars.posMechTheta);

            motorVars.ctrlSpdOut = DCL_runPI_C1(&motorVars.dcl_spd,
                                                motorVars.ctrlSpeedRef,
                                                motorVars.speedWe);
        }

        motorVars.speedLoopCount = 0;
    }

    if(motorVars.lsw == ENC_ALIGNMENT)
    {
        motorVars.rc.SetpointValue = 0;  // position = 0 deg

        motorVars.dcl_spd.i6 = 0.0;
        motorVars.dcl_spd.i10 = 0.0;
        motorVars.dcl_spd.i11 = 0.0;

        motorVars.dcl_pos.i6 = 0.0;
        motorVars.dcl_pos.i10 = 0.0;
        motorVars.dcl_pos.i11 = 0.0;

        motorVars.rg.Out = 0;
        motorVars.lsw2EntryFlag = 0;
    }

#endif  // (SPD_CNTLR == SPD_DCL_CNTLR)

#if(SPD_CNTLR == SPD_NLPID_CNTLR)
    if(motorVars.speedLoopCount >= motorVars.speedLoopPrescaler)
    {
        if(motorVars.lsw == ENC_CALIBRATION_DONE)
        {
            if(!motorVars.lsw2EntryFlag)
            {
                motorVars.lsw2EntryFlag = 1;
                motorVars.rc.TargetValue = motorVars.posMechTheta;
                motorVars.pi_pos.Fbk = motorVars.rc.TargetValue;
                motorVars.pi_pos.Ref = motorVars.pi_pos.Fbk;
            }
            else
            {
                // ========== reference position setting =========
                // choose between 1 of 2 position commands
                // The user can choose between a position reference table
                // used within refPosGen() or feed it in from rg1.Out
                // Position command read from a table
                motorVars.rc.TargetValue =
                        refPosGen(motorVars.rc.TargetValue, &motorVars);

                motorVars.rc.SetpointValue = motorVars.rc.TargetValue -
                             (float32_t)((int32_t)motorVars.rc.TargetValue);

                // Rolling in angle within 0 to 1pu
                if(motorVars.rc.SetpointValue < 0)
                {
                    motorVars.rc.SetpointValue += 1.0;
                }

                motorVars.pi_pos.Ref = motorVars.rc.SetpointValue;
                motorVars.pi_pos.Fbk = motorVars.posMechTheta;
            }

            runPIPos(&motorVars.pi_pos);

            // speed PI regulator
            motorVars.pid_spd.term.Ref = motorVars.pi_pos.Out;
            motorVars.pid_spd.term.Fbk = motorVars.speedWe;
            runPID(&motorVars.pid_spd);
        }

        motorVars.speedLoopCount = 0;
    }

    if(motorVars.lsw == ENC_ALIGNMENT)
    {
        motorVars.rc.SetpointValue = 0;  // position = 0 deg
        motorVars.pid_spd.data.d1 = 0;
        motorVars.pid_spd.data.d2 = 0;
        motorVars.pid_spd.data.i1 = 0;
        motorVars.pid_spd.data.ud = 0;
        motorVars.pid_spd.data.ui = 0;
        motorVars.pid_spd.data.up = 0;

        motorVars.pi_pos.ui = 0;
        motorVars.pi_pos.i1 = 0;

        motorVars.rg.Out = 0;
        motorVars.lsw2EntryFlag = 0;
    }

    motorVars.ctrlSpdOut = motorVars.pid_spd.term.Out;
#endif  // (SPD_CNTLR == SPD_NLPID_CNTLR)

    //
    //  Setup iqref for FCL
    //
    motorVars.cmplx_Iq.ref = (motorVars.lsw == ENC_ALIGNMENT) ? 0 :
                               (motorVars.lsw == ENC_WAIT_FOR_INDEX) ?
                                 motorVars.IqRef : motorVars.ctrlSpdOut;

    //
    //  Setup idref for FCL
    //
    motorVars.IdRef = ramper(motorVars.ctrlIdRef,
                             motorVars.IdRef, 0.00001);

    motorVars.cmplx_Id.ref = motorVars.IdRef;

    // DEBUG: customer can remove the below code in final implementation
    // get FOC count
    getFOCCount(&motorVars);

    // get FCL latency
    getFCLTime(&motorVars);

    // get FOC execution time
    getFOCTime(&motorVars);
    // DEBUG: customer can remove the above code in final implementation

    return;
}
#endif // (BUILDLEVEL==FCL_LEVEL5)

__interrupt void motorControlISR(void)
{

#if(BUILDLEVEL == FCL_LEVEL1)
    buildLevel1();

    //
    // Connect inputs of the DATALOG module
    //
#ifdef DLOG_ENABLE
    dlogCh1 = motorVars.rg.Out;
    dlogCh2 = motorVars.svgen.Ta;
    dlogCh3 = motorVars.svgen.Tb;
    dlogCh4 = motorVars.svgen.Tc;
#endif  // DLOG_ENABLE

#ifdef DACOUT_EN
//------------------------------------------------------------------------------
// Variable display on DACs
//------------------------------------------------------------------------------
    DAC_setShadowValue(hal.dacHandle[0],
                       DAC_MACRO_PU(motorVars.IdRef));
    DAC_setShadowValue(hal.dacHandle[1],
                       DAC_MACRO_PU(motorVars.IqRef));
#endif   // DACOUT_EN

#elif(BUILDLEVEL == FCL_LEVEL2)
    buildLevel2();

    //
    // Connect inputs of the DATALOG module
    //
#ifdef DLOG_ENABLE
    dlogCh1 = motorVars.rg.Out;
    dlogCh2 = motorVars.speed.ElecTheta;
    dlogCh3 = motorVars.clarke.As;
    dlogCh4 = motorVars.clarke.Bs;
#endif  // DLOG_ENABLE

#elif(BUILDLEVEL == FCL_LEVEL3)
    buildLevel3();

    //
    // Connect inputs of the DATALOG module
    //
#ifdef DLOG_ENABLE
    dlogCh1 = motorVars.posElecTheta;
    dlogCh2 = motorVars.rg.Out;
    dlogCh3 = motorVars.cmplx_Iq.ref;
    dlogCh4 = motorVars.cmplx_Iq.fbk;
#endif  // DLOG_ENABLE

#elif(BUILDLEVEL == FCL_LEVEL4)
    buildLevel46();

    //
    // Connect inputs of the DATALOG module
    //
#ifdef DLOG_ENABLE
    dlogCh1 = motorVars.posElecTheta;
    dlogCh2 = motorVars.speedWe;
    dlogCh3 = motorVars.cmplx_Id.fbk;
    dlogCh4 = motorVars.cmplx_Iq.fbk;
#endif  // DLOG_ENABLE

#elif(BUILDLEVEL == FCL_LEVEL5)
    buildLevel5();

    //
    // Connect inputs of the DATALOG module
    //
#ifdef DLOG_ENABLE
    dlogCh1 = motorVars.ctrlPosRef;
    dlogCh2 = motorVars.posMechTheta;
    dlogCh3 = motorVars.cmplx_Id.fbk;
    dlogCh4 = motorVars.cmplx_Iq.fbk;
#endif  // DLOG_ENABLE

#elif(BUILDLEVEL == FCL_LEVEL6)
    buildLevel46();

    //
    // Connect inputs of the DATALOG module
    //
#ifdef DLOG_ENABLE
    dlogCh1 = motorVars.posElecTheta;
    dlogCh2 = motorVars.speedWe;
    dlogCh3 = motorVars.cmplx_Id.fbk;
    dlogCh4 = motorVars.cmplx_Iq.fbk;
#endif  // DLOG_ENABLE
#endif // (BUILDLEVEL)


    //
    //    Call the DATALOG update function.
    //
#ifdef DLOG_ENABLE
    DLOG_4CH_F_FUNC(&dlog_4ch1);
#endif  // DLOG_ENABLE

    motorVars.isrTicker++;

    // Acknowledges an interrupt
    HAL_ackInt(halHandle);

    return;
} // motor1ControlISR Ends Here

//
// run the motor control
//
void runMotorControl(MOTOR_Vars_t *pMotor, HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    // *******************************************************
    // Current limit setting / tuning in Debug environment
    // *******************************************************
    pMotor->currentThreshHi = 2048 +
            scaleCurrentValue(pMotor->currentLimit);
    pMotor->currentThreshLo = 2048 -
            scaleCurrentValue(pMotor->currentLimit);

    HAL_setupCMPSS_DACValue(handle,
                            pMotor->currentThreshHi, pMotor->currentThreshLo);

#ifdef F28004x_DEVICE
    // comparator references
    // Set DAC-H to allowed maximum voltage
    pMotor->voltageThreshHi = scaleVoltageValue(pMotor->voltageLimit);
    CMPSS_setDACValueHigh(obj->cmpssHandle[3], pMotor->voltageThreshHi);
#endif  // F28004x_DEVICE

#ifdef F28002x_DEVICE
    // Not connect to CMPSS for voltage protection
#endif  // F28002x_DEVICE

    pMotor->Vdcbus = (pMotor->Vdcbus * 0.8) + (pMotor->FCL_params.Vdcbus * 0.2);

    if( (pMotor->Vdcbus > pMotor->VdcbusMax) ||
            (pMotor->Vdcbus < pMotor->VdcbusMin) )
    {
        pMotor->faultStatusFlag |= 0x0002;
    }
    else
    {
        pMotor->faultStatusFlag &= (0xFFFF - 0x0002);
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

        pMotor->faultStatusFlag |= 0x0001;      // over current fault trip
    }

    if(pMotor->faultStatusFlag != 0)
    {
        pMotor->ctrlStateUse = CTRL_STOP;
        pMotor->ctrlStateFdb = CTRL_FAULT;

        // Disable Driver Gate
        GPIO_writePin(pMotor->drvEnableGateGPIO, 1);
    }

    if((pMotor->faultStatusFlag != 0) && (motorVars.ctrlStateCom == CTRL_RESET))
    {
        pMotor->clearFaultFlag = 1;
    }

    pMotor->faultStatusPrev |= pMotor->faultStatusFlag;

    if((pMotor->faultStatusFlag != 0) && (pMotor->clearFaultFlag == 1))
    {
        pMotor->faultTimesCount++;
    }

    // If clear cmd received, reset PWM trip
    if(pMotor->clearFaultFlag)
    {
        // clear HLATCH - (not in TRIP gen path)
        CMPSS_clearFilterLatchHigh(obj->cmpssHandle[0]);
        CMPSS_clearFilterLatchHigh(obj->cmpssHandle[1]);
        CMPSS_clearFilterLatchHigh(obj->cmpssHandle[2]);
        CMPSS_clearFilterLatchHigh(obj->cmpssHandle[3]);

        // clear LLATCH - (not in TRIP gen path)
        CMPSS_clearFilterLatchLow(obj->cmpssHandle[0]);
        CMPSS_clearFilterLatchLow(obj->cmpssHandle[1]);
        CMPSS_clearFilterLatchLow(obj->cmpssHandle[2]);
        CMPSS_clearFilterLatchLow(obj->cmpssHandle[3]);

        // clear Latch fault
        GPIO_writePin(pMotor->drvClearFaultGPIO, 0);

        // clear EPWM trip flags
        DEVICE_DELAY_US(1L);

        // clear Latch fault
        GPIO_writePin(pMotor->drvClearFaultGPIO, 1);

        // clear OST & DCAEVT1 flags
        EPWM_clearTripZoneFlag(obj->pwmHandle[0],
                               (EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1));

        EPWM_clearTripZoneFlag(obj->pwmHandle[1],
                               (EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1));

        EPWM_clearTripZoneFlag(obj->pwmHandle[2],
                               (EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1));

        // clear HLATCH - (not in TRIP gen path)
        CMPSS_clearFilterLatchHigh(obj->cmpssHandle[0]);
        CMPSS_clearFilterLatchHigh(obj->cmpssHandle[1]);
        CMPSS_clearFilterLatchHigh(obj->cmpssHandle[2]);
        CMPSS_clearFilterLatchHigh(obj->cmpssHandle[3]);

        // clear LLATCH - (not in TRIP gen path)
        CMPSS_clearFilterLatchLow(obj->cmpssHandle[0]);
        CMPSS_clearFilterLatchLow(obj->cmpssHandle[1]);
        CMPSS_clearFilterLatchLow(obj->cmpssHandle[2]);
        CMPSS_clearFilterLatchLow(obj->cmpssHandle[3]);

        // clear Latch fault
        GPIO_writePin(pMotor->drvClearFaultGPIO, 0);

        // clear the ocp
        pMotor->faultStatusFlag = 0;
        pMotor->clearFaultFlag = 0;
        pMotor->ctrlStateFdb = CTRL_STOP;
        pMotor->lsw = ENC_ALIGNMENT;

        // clear Latch fault
        GPIO_writePin(pMotor->drvClearFaultGPIO, 1);
    }

    if(pMotor->ctrlStateUse == CTRL_RUN)
    {
        // Enable Driver Gate
        GPIO_writePin(pMotor->drvEnableGateGPIO, 0);
    }
    else
    {
        // Disable Driver Gate
        GPIO_writePin(pMotor->drvEnableGateGPIO, 1);
    }

    return;
}

//
// Build level 6: SFRA support functions
//
#if(BUILDLEVEL == FCL_LEVEL6)
//
// Using SFRA tool :
//      - INJECT noise
//      - RUN the controller
//      - CAPTURE or COLLECT the controller output
// From a controller analysis standpoint, this sequence will reveal the
// output of controller for a given input, and therefore, good for analysis
//
void injectSFRA(void)
{
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

//
// collectSFRA:
//
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
        SFRA_F32_collect(&pMotor->ctrlSpdOut,
                         &pMotor->speedWe);
    }

    return;
}
#endif  // (BUILDLEVEL == FCL_LEVEL6)

//
// End of Code in this file
//
