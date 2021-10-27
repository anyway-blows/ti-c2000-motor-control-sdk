//#############################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:28 CDT 2021 $
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

//
//! \file   /solutions/fast_uni_lab/common/source/motor1_drive.c
//!
//! \brief  This project is used to implement motor control with FAST, eSMO
//!         Encoder, and Hall sensors based sensored/sensorless-FOC.
//!         Supports multiple TI EVM board with F28002x
//!
//


//
// include the related header files
//
#include "sys_settings.h"
#include "sys_main.h"
#include "motor1_drive.h"

#pragma CODE_SECTION(motor1CtrlISR, ".TI.ramfunc");

// the globals

//!< the hardware abstraction layer object to motor control
HAL_MTR_Obj    halMtr_M1;
#pragma DATA_SECTION(halMtr_M1, "user_data");

USER_Params userParams_M1;
#pragma DATA_SECTION(userParams_M1,"user_data");

volatile MOTOR_Handle motorHandle_M1;
#pragma DATA_SECTION(motorHandle_M1,"motor_data");

volatile MOTOR_Vars_t motorVars_M1;
#pragma DATA_SECTION(motorVars_M1, "motor_data");

MOTOR_SetVars_t motorSetVars_M1;
#pragma DATA_SECTION(motorSetVars_M1, "foc_data");

#if defined(MOTOR1_FAST)
//!< the voltage Clarke transform object
CLARKE_Obj    clarke_V_M1;
#pragma DATA_SECTION(clarke_V_M1, "foc_data");
#endif  // MOTOR1_FAST

//!< the current Clarke transform object
CLARKE_Obj    clarke_I_M1;
#pragma DATA_SECTION(clarke_I_M1, "foc_data");

//!< the inverse Park transform object
IPARK_Obj     ipark_V_M1;
#pragma DATA_SECTION(ipark_V_M1, "foc_data");

//!< the Park transform object
PARK_Obj      park_I_M1;
#pragma DATA_SECTION(park_I_M1, "foc_data");

//!< the Park transform object
PARK_Obj      park_V_M1;
#pragma DATA_SECTION(park_V_M1, "foc_data");

//!< the Id PI controller object
PI_Obj        pi_Id_M1;
#pragma DATA_SECTION(pi_Id_M1, "foc_data");

//!< the Iq PI controller object
PI_Obj        pi_Iq_M1;
#pragma DATA_SECTION(pi_Iq_M1, "foc_data");

//!< the speed PI controller object
PI_Obj        pi_spd_M1;
#pragma DATA_SECTION(pi_spd_M1, "foc_data");

//!< the space vector generator object
SVGEN_Obj     svgen_M1;
#pragma DATA_SECTION(svgen_M1, "foc_data");

#if defined(MOTOR1_OVM)
//!< the handle for the space vector generator current
SVGENCURRENT_Obj svgencurrent_M1;
#pragma DATA_SECTION(svgencurrent_M1, "foc_data");
#endif  // MOTOR1_OVM

//!< the speed reference trajectory object
TRAJ_Obj     traj_spd_M1;
#pragma DATA_SECTION(traj_spd_M1, "foc_data");

#if defined(MOTOR1_FWC)
//!< the fwc PI controller object
PI_Obj       pi_fwc_M1;
#pragma DATA_SECTION(pi_fwc_M1, "foc_data");
#endif  // MOTOR1_FWC

#if defined(MOTOR1_ISBLDC)
//!< the isbldc object
ISBLDC_Obj isbldc_M1;

//!< the rimpulse object
RIMPULSE_Obj rimpulse_M1;

//!< the mod6cnt object
MOD6CNT_Obj mod6cnt_M1;

//!< the bldc object
BLDC_Obj bldc_M1;
#else // !MOTOR1_ISBLDC

#if (DMC_BUILDLEVEL <= DMC_LEVEL_3) || defined(MOTOR1_VOLRECT) || \
               defined(MOTOR1_ESMO) || defined(MOTOR1_ENC)
//!< the Angle Generate onject for open loop control
ANGLE_GEN_Obj    angleGen_M1;
#pragma DATA_SECTION(angleGen_M1, "foc_data");
#endif  // DMC_BUILDLEVEL <= DMC_LEVEL_3 || MOTOR1_ESMO || MOTOR1_VOLRECT

#if(DMC_BUILDLEVEL == DMC_LEVEL_2)
//!< the Vs per Freq object for open loop control
VS_FREQ_Obj    VsFreq_M1;
#pragma DATA_SECTION(VsFreq_M1, "foc_data");
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_2)

#endif  // !MOTOR1_ISBLDC

#if defined(MOTOR1_ENC)
//!< the handle for the enc object
ENC_Obj enc_M1;

//!< the handle for the speedcalc object
SPDCALC_Obj speedcalc_M1;
#endif  // MOTOR1_ENC

#if defined(MOTOR1_HALL)
//!< the handle for the enc object
HALL_Obj hall_M1;
// Enable MOTOR1_HALL_CAL pre-defined sybmols, run the motor with FAST for angle
// calibration
// Copy hall_M1.thetaCalBuf[] to hallAngleBuf[]
// 1->1, 2->2, 3->3, 4->4, 5->5, 6->6, 6->0
// Disable MOTOR1_HALL_CAL pre-defined sybmols after calibration for normal operation
                                    // 6           1              2
                                    // 3           4              5
                                    // 6
#if (USER_MOTOR1 == Teknic_M2310PLN04K)
const float32_t hallAngleBuf[7] = { 2.73360014f,  -0.468535542f, -2.48626161f, \
                                    -1.47709608f, 1.68133402f , 0.614400506f,
                                    2.73360014f };

#elif (USER_MOTOR1 == Anaheim_BLY172S_24V)
const float32_t hallAngleBuf[7] = { -1.41421735f,  1.75656128f, -2.48391223f, \
                                    2.76515913f,  -0.460148782f, 0.606459916f,
                                    -1.41421735f };
#elif (USER_MOTOR1 == Anaheim_BLWS235D)
const float32_t hallAngleBuf[7] = { 1.64448488f,  -1.54361129f,  0.548367858f, \
                                   -0.390248626f,  2.67842388f, -2.52673817f,
                                    1.64448488f };
#else   // !Teknic_M2310PLN04K || !Anaheim_BLY172S_24V || !Anaheim_BLWS235D
#error Not a right hall angle buffer for this project, need to do hall calibration
#endif  //

#endif  // MOTOR1_HALL

#if defined(MOTOR1_ESMO)
//!< the speedfr object
SPDFR_Obj spdfr_M1;
#pragma DATA_SECTION(spdfr_M1, "foc_data");

//!< the esmo object
ESMO_Obj   esmo_M1;
#pragma DATA_SECTION(esmo_M1, "foc_data");
#endif  // MOTOR1_ESMO

#if defined(MOTOR1_MTPA)
//!< the handle for the Maximum torque per ampere (MTPA)
MTPA_Handle  mtpaHandle;

//!< the Maximum torque per ampere (MTPA) object
MTPA_Obj     mtpa_M1;
#pragma DATA_SECTION(mtpa_M1, "foc_data");
#endif  // MOTOR1_MTPA

#if defined(MOTOR1_SSIPD)
SSIPD_Obj       ssipd_M1;
#pragma DATA_SECTION(ssipd_M1, "foc_data");
#endif  // MOTOR1_SSIPD

#if defined(MOTOR1_DCLINKSS)
//!< the single-shunt current reconstruction object
DCLINK_SS_Obj    dclink_M1;

#pragma DATA_SECTION(dclink_M1, "foc_data");
#endif // MOTOR1_DCLINKSS

#if defined(MOTOR1_VOLRECT)
//!< the voltage reconstruct object
VOLREC_Obj volrec_M1;
#pragma DATA_SECTION(volrec_M1, "foc_data");
#endif  // MOTOR1_VOLRECT

#if defined(BSXL8320RS_REVA) || defined(BSXL8323RS_REVA) || \
    defined(BSXL8353RS_REVA) || defined(BSXL8316RT_REVA)
// Watch window interface to the drv8323/drv8353/drv8316 SPI
DRVIC_VARS_t drvicVars_M1;
#pragma DATA_SECTION(drvicVars_M1, "foc_data");
#endif  // BSXL8320RS_REVA || BSXL8323RS_REVA ||
        // BSXL8353RS_REVA || BSXL8316RT_REVA

// the control handles for motor 1
void initMotor1Handles(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;

    obj->motorNum = MTR_1;

    obj->motorSetsHandle = &motorSetVars_M1;
    obj->userParamsHandle = &userParams_M1;

    // initialize the driver
    obj->halMtrHandle = HAL_MTR1_init(&halMtr_M1, sizeof(halMtr_M1));

#if defined(BSXL8320RS_REVA) || defined(BSXL8323RS_REVA) || \
    defined(BSXL8353RS_REVA) || defined(BSXL8316RT_REVA)
    obj->drvVarsHandle = &drvicVars_M1;
#endif  // BSXL8320RS_REVA || BSXL8323RS_REVA ||
        // BSXL8353RS_REVA || BSXL8316RT_REVA

    return;
}

// initialize control parameters for motor 1
void initMotor1CtrlParameters(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(handle->motorSetsHandle);
    USER_Params *objUser = (USER_Params *)(handle->userParamsHandle);

    // initialize the user parameters
    USER_setParams_priv(obj->userParamsHandle);

    // initialize the user parameters
    USER_setMotor1Params(obj->userParamsHandle);

    // set the driver parameters
    HAL_MTR_setParams(obj->halMtrHandle, obj->userParamsHandle);

    objSets->Kp_spd = 0.05f;
    objSets->Ki_spd = 0.005f;

    objSets->Kp_fwc = USER_M1_FWC_KP;
    objSets->Ki_fwc = USER_M1_FWC_KI;
    objSets->angleFWCMax_rad = USER_M1_FWC_MAX_ANGLE_RAD;

    objSets->overModulation = USER_M1_MAX_VS_MAG_PU;
    objSets->RsOnLineCurrent_A = 0.1f * USER_MOTOR1_MAX_CURRENT_A;

    objSets->lostPhaseSet_A = USER_M1_LOST_PHASE_CURRENT_A;
    objSets->unbalanceRatioSet = USER_M1_UNBALANCE_RATIO;
    objSets->overLoadSet_W = USER_M1_OVER_LOAD_POWER_W;
    objSets->stallCurrentSet_A = USER_M1_STALL_CURRENT_A;

    objSets->IsFailedChekSet_A = USER_M1_FAULT_CHECK_CURRENT_A;

    objSets->speedFailMaxSet_Hz = USER_M1_FAIL_SPEED_MAX_HZ;
    objSets->speedFailMinSet_Hz = USER_M1_FAIL_SPEED_MIN_HZ;

    objSets->toqueFailMinSet_Nm = USER_M1_TORQUE_FAILED_SET;

    objSets->maxPeakCurrent_A = USER_M1_ADC_FULL_SCALE_CURRENT_A * 0.495f;
    objSets->overCurrent_A = USER_MOTOR1_OVER_CURRENT_A;
    objSets->currentInv_sf = USER_M1_CURRENT_INV_SF;

    objSets->overVoltageFault_V = USER_M1_OVER_VOLTAGE_FAULT_V;
    objSets->overVoltageNorm_V = USER_M1_OVER_VOLTAGE_NORM_V;
    objSets->underVoltageFault_V = USER_M1_UNDER_VOLTAGE_FAULT_V;
    objSets->underVoltageNorm_V = USER_M1_UNDER_VOLTAGE_NORM_V;

    objSets->voltageFaultTimeSet = USER_M1_VOLTAGE_FAULT_TIME_SET;

    objSets->overLoadTimeSet = USER_M1_OVER_LOAD_TIME_SET;
    objSets->motorStallTimeSet = USER_M1_STALL_TIME_SET;
    objSets->unbalanceTimeSet = USER_M1_UNBALANCE_TIME_SET;
    objSets->lostPhaseTimeSet = USER_M1_LOST_PHASE_TIME_SET;
    objSets->overSpeedTimeSet = USER_M1_OVER_SPEED_TIME_SET;
    objSets->startupFailTimeSet = USER_M1_STARTUP_FAIL_TIME_SET;

    objSets->overCurrentTimesSet = USER_M1_OVER_CURRENT_TIMES_SET;

    objSets->stopWaitTimeSet = USER_M1_STOP_WAIT_TIME_SET;
    objSets->restartWaitTimeSet = USER_M1_RESTART_WAIT_TIME_SET;


    objSets->restartTimesSet = USER_M1_START_TIMES_SET;

    objSets->dacCMPValH = 2048U + 1024U;    // set default positive peak value
    objSets->dacCMPValL = 2048U - 1024U;    // set default negative peak value

    obj->stopWaitTimeCnt = 0;

    obj->adcData.voltage_sf = objUser->voltage_sf;
    obj->adcData.dcBusvoltage_sf = objUser->voltage_sf;

#if defined(MOTOR1_ISBLDC) || defined(MOTOR1_DCLINKSS)
#if defined(BSXL8320RS_REVA)
    obj->adcData.current_sf = -objUser->current_sf;
#elif defined(BSXL8323RS_REVA) || defined(BSXL8323RH_REVB)
    obj->adcData.current_sf = -objUser->current_sf;
#else   // No Hardware
#error Not select a right current scale factor based on hardware board
#endif  // No Hardware
#else   // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)
#if defined(BSXL8320RS_REVA) || defined(BSXL3PHGAN_REVA)
    obj->adcData.current_sf = -objUser->current_sf;
#elif defined(BSXL8323RS_REVA) || defined(BSXL8323RH_REVB) || \
      defined(BSXL8353RS_REVA) || defined(BSXL8316RT_REVA)
    obj->adcData.current_sf = objUser->current_sf;
#elif defined(HVMTRPFC_REV1P1)
    obj->adcData.current_sf = objUser->current_sf;
#else   // No Hardware
#error Not select a right current scale factor based on hardware board
#endif  // No Hardware
#endif  // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)

    obj->flagEnableRestart = false;

    obj->operateMode = OPERATE_MODE_SPEED;
    obj->faultMtrMask.all = MTR1_FAULT_MASK_SET;

    obj->speedStart_Hz = USER_MOTOR1_SPEED_START_Hz;
    obj->speedForce_Hz = USER_MOTOR1_SPEED_FORCE_Hz;
    obj->speedFlyingStart_Hz = USER_MOTOR1_SPEED_FS_Hz;

    obj->accelerationMax_Hzps = USER_MOTOR1_ACCEL_MAX_Hzps;
    obj->accelerationStart_Hzps = USER_MOTOR1_ACCEL_START_Hzps;

    obj->VsRef_pu = 0.98f * USER_M1_MAX_VS_MAG_PU;
    obj->VsRef_V =
            0.98f * USER_M1_MAX_VS_MAG_PU * USER_M1_NOMINAL_DC_BUS_VOLTAGE_V;

    obj->fluxCurrent_A = USER_MOTOR1_FLUX_CURRENT_A;
    obj->alignCurrent_A = USER_MOTOR1_ALIGN_CURRENT_A;
    obj->startCurrent_A = USER_MOTOR1_STARTUP_CURRENT_A;
    obj->maxCurrent_A = USER_MOTOR1_MAX_CURRENT_A;
    obj->IsSet_A = USER_MOTOR1_TORQUE_CURRENT_A;

    obj->power_sf = MATH_TWO_PI / USER_MOTOR1_NUM_POLE_PAIRS;
    obj->VIrmsIsrScale = objUser->ctrlFreq_Hz;

    obj->angleDelayed_sf = 0.5f * MATH_TWO_PI * USER_M1_CTRL_PERIOD_sec;

    obj->flyingStartTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 0.5f); // 0.5s

    obj->flyingStartMode = FLYINGSTART_MODE_HALT;

    if(objUser->flag_bypassMotorId == true)
    {
#if defined(MOTOR1_DCLINKSS)
        obj->svmMode = SVM_COM_C;
#else  // !(MOTOR1_DCLINKSS)
        obj->svmMode = SVM_MIN_C;
#endif  // !(MOTOR1_DCLINKSS)
        obj->flagEnableFWC = true;
    }
    else
    {
        obj->svmMode = SVM_COM_C;
        obj->flagEnableFWC = false;
    }

    obj->flagEnableForceAngle = true;
    obj->flagEnableFlyingStart = false;
    obj->flagEnableIPD = false;

    obj->flagEnableSpeedCtrl = true;
    obj->flagEnableCurrentCtrl = true;

#if defined(MOTOR1_ISBLDC)
    obj->estimatorMode = ESTIMATOR_MODE_BINT;

    obj->flagEnableAlignment = true;
    obj->alignTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 1.0f);          // 1.0s
    obj->forceRunTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 1.0f);       // 1.0s
    obj->fwcTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 2.0f);        // 2.0s
#elif defined(MOTOR1_FAST) && defined(MOTOR1_ENC)
    obj->estState = EST_STATE_IDLE;
    obj->trajState = EST_TRAJ_STATE_IDLE;

    obj->estimatorMode = ESTIMATOR_MODE_FAST;

    obj->flagEnableAlignment = true;
    obj->alignTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 0.1f);      // 0.1s
    obj->forceRunTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 1.0f);   // 1.0s
    obj->fwcTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 2.0f);        // 2.0s
#elif defined(MOTOR1_FAST) && defined(MOTOR1_ESMO)
    obj->estState = EST_STATE_IDLE;
    obj->trajState = EST_TRAJ_STATE_IDLE;

    obj->estimatorMode = ESTIMATOR_MODE_FAST;

    obj->flagEnableAlignment = true;
    obj->alignTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 0.1f);          // 0.1s
    obj->forceRunTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 1.0f);       // 1.0s
    obj->fwcTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 2.0f);        // 2.0s
#elif defined(MOTOR1_FAST) && defined(MOTOR1_HALL)
    obj->estState = EST_STATE_IDLE;
    obj->trajState = EST_TRAJ_STATE_IDLE;

    obj->estimatorMode = ESTIMATOR_MODE_FAST;

    obj->flagEnableAlignment = true;
    obj->alignTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 0.1f);          // 0.1s
    obj->forceRunTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 1.0f);       // 1.0s

    obj->fwcTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 2.0f);        // 2.0s
#elif defined(MOTOR1_FAST)
    obj->estState = EST_STATE_IDLE;
    obj->trajState = EST_TRAJ_STATE_IDLE;

    obj->estimatorMode = ESTIMATOR_MODE_FAST;

    obj->flagEnableAlignment = true;
    obj->alignTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 0.1f);          // 0.1s
    obj->forceRunTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 1.0f);       // 1.0s

    obj->fwcTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 2.0f);        // 2.0s
#elif defined(MOTOR1_ESMO)
    obj->estimatorMode = ESTIMATOR_MODE_ESMO;

    obj->flagEnableAlignment = true;
    obj->alignTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 0.1f);          // 0.1s
    obj->forceRunTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 1.0f);       // 1.0s
    obj->fwcTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 2.0f);        // 2.0s
#elif defined(MOTOR1_ENC)
    obj->estimatorMode = ESTIMATOR_MODE_ENC;

    obj->alignTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 0.1f);          // 0.1s
    obj->forceRunTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 1.0f);       // 1.0s
    obj->fwcTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 2.0f);        // 2.0s
#elif defined(MOTOR1_HALL)
    obj->estimatorMode = ESTIMATOR_MODE_HALL;

    obj->flagEnableAlignment = true;
    obj->alignTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 1.0f);      // 1.0s
    obj->forceRunTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 1.0f);   // 1.0s
    obj->fwcTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 2.0f);        // 2.0s
#else
#error Not select a right estimator for this project
#endif  // !MOTOR1_ISBLDC

#if defined(MOTOR1_FWC)
    obj->piHandle_fwc = PI_init(&pi_fwc_M1, sizeof(pi_fwc_M1));

    // set the FWC controller
    PI_setGains(obj->piHandle_fwc, USER_M1_FWC_KP, USER_M1_FWC_KI);
    PI_setUi(obj->piHandle_fwc, 0.0);
    PI_setMinMax(obj->piHandle_fwc, USER_M1_FWC_MAX_ANGLE_RAD,
                 USER_M1_FWC_MIN_ANGLE_RAD);
#endif  // MOTOR1_FWC

#ifdef MOTOR1_MTPA
    // initialize the Maximum torque per ampere (MTPA)
    obj->mtpaHandle = MTPA_init(&mtpa_M1, sizeof(mtpa_M1));

    // compute the motor constant for MTPA
    MTPA_computeParameters(obj->mtpaHandle,
                           objUser->motor_Ls_d_H,
                           objUser->motor_Ls_q_H,
                           objUser->motor_ratedFlux_Wb);
#endif  // MOTOR1_MTPA

#if defined(MOTOR1_SSIPD)
    obj->ssipdHandle = SSIPD_init(&ssipd_M1, sizeof(ssipd_M1));
    SSIPD_setParams(obj->ssipdHandle, 0.75f, (MATH_TWO_PI / SSIPD_DETECT_NUM), 6);

    obj->angleOffsetIPD_rad = MATH_PI / SSIPD_DETECT_NUM;
#endif  // MOTOR1_SSIPD

#if defined(MOTOR1_DCLINKSS)
    obj->dclinkHandle = DCLINK_SS_init(&dclink_M1, sizeof(dclink_M1));
    DCLINK_SS_setInitialConditions(obj->dclinkHandle,
                                   HAL_getTimeBasePeriod(obj->halMtrHandle), 0.5f);

    //disable full sampling
    DCLINK_SS_setFlag_enableFullSampling(obj->dclinkHandle, false);

    //enable sequence control
    DCLINK_SS_setFlag_enableSequenceControl(obj->dclinkHandle, true);

    // Tdt  =  10 ns (Dead-time between top and bottom switch)
    // Tpd  =  38 ns (Gate driver propagation delay)
    // Tr   = 100 ns (Rise time of amplifier including power switches turn on time)
    // Ts   = 100 ns (Settling time of amplifier)
    // Ts&h = 170 ns (ADC sample&holder = 1+(14)+2 = 17 SYSCLK)
    // -------------------------------------------------------------
    // T_MinAVDuration = Tdt+Tr+Ts+Ts&h
    //                 = 10+100+100+170 = 380ns => 38 SYSCLK cycles
    // -------------------------------------------------------------
    // T_SampleDelay = Tdt+Tpd+Tr+Ts
    //               = 10+38+100+100 = 248ns => 25 SYSCLK cycles
    // -------------------------------------------------------------
#if defined(BSXL8323RS_REVA)
    DCLINK_SS_setMinAVDuration(obj->dclinkHandle, 120);  //T_MinAVDuration
    DCLINK_SS_setSampleDelay(obj->dclinkHandle, 103);    //T_SampleDelay
#elif defined(BSXL8323RH_REVB)
    DCLINK_SS_setMinAVDuration(obj->dclinkHandle, 120);  //T_MinAVDuration
    DCLINK_SS_setSampleDelay(obj->dclinkHandle, 103);    //T_SampleDelay
#elif defined(HVMTRPFC_REV1P1)
    DCLINK_SS_setMinAVDuration(obj->dclinkHandle, 255);  //T_MinAVDuration
    DCLINK_SS_setSampleDelay(obj->dclinkHandle, 233);    //T_SampleDelay
#else   // !(BSXL8323RS_REVA || BSXL8323RH_REVB || HVMTRPFC_REV1P1)
#error Not support single shunt on this kit
#endif  // !(BSXL8323RS_REVA || BSXL8323RH_REVB || HVMTRPFC_REV1P1)
#endif   // MOTOR1_DCLINKSS ||MOTOR1_DCLINKSS

#ifdef MOTOR1_VOLRECT
    // initialize the Voltage reconstruction
    obj->volrecHandle = VOLREC_init(&volrec_M1, sizeof(volrec_M1));

    // configure the Voltage reconstruction
    VOLREC_setParams(obj->volrecHandle,
                     objUser->voltageFilterPole_rps,
                     objUser->ctrlFreq_Hz);

    VOLREC_disableFlagEnableSf(obj->volrecHandle);
#endif  // MOTOR1_VOLRECT

#if defined(MOTOR1_ESMO)
    // initialize the esmo
    obj->esmoHandle = ESMO_init(&esmo_M1, sizeof(esmo_M1));

    // set parameters for ESMO controller
    ESMO_setKslideParams(obj->esmoHandle,
                         USER_MOTOR1_KSLIDE_MAX, USER_MOTOR1_KSLIDE_MIN);

    ESMO_setPLLParams(obj->esmoHandle, USER_MOTOR1_PLL_KP_MAX,
                      USER_MOTOR1_PLL_KP_MIN, USER_MOTOR1_PLL_KP_SF);

    ESMO_setBEMFThreshold(obj->esmoHandle, USER_MOTOR1_BEMF_THRESHOLD);
    ESMO_setOffsetCoef(obj->esmoHandle, USER_MOTOR1_THETA_OFFSET_SF);
    ESMO_setBEMFKslfFreq(obj->esmoHandle, USER_MOTOR1_BEMF_KSLF_FC_Hz);
    ESMO_setSpeedFilterFreq(obj->esmoHandle, USER_MOTOR1_SPEED_LPF_FC_Hz);

    // set the ESMO controller parameters
    ESMO_setParams(obj->esmoHandle, obj->userParamsHandle);

    // initialize the spdfr
    obj->spdfrHandle = SPDFR_init(&spdfr_M1, sizeof(spdfr_M1));

    // set the spdfr parameters
    SPDFR_setParams(obj->spdfrHandle, obj->userParamsHandle);

    obj->frswPos_sf = 0.6f;
#endif  //MOTOR1_ESMO



#if defined(MOTOR1_ISBLDC)
    // initialize the ISBLDC handle
    obj->isbldcHandle = ISBLDC_init(&isbldc_M1, sizeof(isbldc_M1));

    // set the ISBLDC controller parameters
    ISBLDC_setParams(obj->isbldcHandle, obj->userParamsHandle,
                     USER_MOTOR1_ISBLDC_INT_MAX, USER_MOTOR1_ISBLDC_INT_MIN);

    // initialize the RIMPULSE handle
    obj->rimpulseHandle = RIMPULSE_init(&rimpulse_M1, sizeof(rimpulse_M1));

    // set the RIMPULSE controller parameters
    RIMPULSE_setParams(obj->rimpulseHandle, obj->userParamsHandle,
                       USER_MOTOR1_RAMP_START_Hz, USER_MOTOR1_RAMP_END_Hz,
                       USER_MOTOR1_RAMP_DELAY);

    // initialize the MOD6CNT handle
    obj->mod6cntHandle = MOD6CNT_init(&mod6cnt_M1, sizeof(mod6cnt_M1));

    // sets up the MOD6CNT controller parameters
    MOD6CNT_setMaximumCount(obj->mod6cntHandle, 6);

    // initialize the BLDC handle
    obj->bldcHandle = &bldc_M1;

    // sets up initialization value for startup
    obj->bldcHandle->IdcRefSet = 1.0f;           // 1.0A
    obj->bldcHandle->IdcStart = 0.5f;            // 0.5A

    obj->bldcHandle->pwmDutySet = 0.10f;         // 10% duty
    obj->bldcHandle->pwmDutyStart = 0.10f;       // 10% duty

    obj->bldcHandle->commSampleDelay = 3;
#else  // !MOTOR1_ISBLDC
#if (DMC_BUILDLEVEL <= DMC_LEVEL_3) || defined(MOTOR1_VOLRECT) || \
               defined(MOTOR1_ESMO) || defined(MOTOR1_ENC)
    // initialize the angle generate module
    obj->angleGenHandle = ANGLE_GEN_init(&angleGen_M1, sizeof(angleGen_M1));

    ANGLE_GEN_setParams(obj->angleGenHandle, objUser->ctrlPeriod_sec);
#endif  // DMC_BUILDLEVEL <= DMC_LEVEL_3 || MOTOR1_ESMO || MOTOR1_VOLRECT || MOTOR1_ENC

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    obj->Idq_set_A.value[0] = 0.0f;
    obj->Idq_set_A.value[1] = obj->startCurrent_A;
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_3)

#if(DMC_BUILDLEVEL == DMC_LEVEL_2)
    // initialize the Vs per Freq module
    obj->VsFreqHandle = VS_FREQ_init(&VsFreq_M1, sizeof(VsFreq_M1));

    VS_FREQ_setVsMagPu(obj->VsFreqHandle, objUser->maxVsMag_pu);

    VS_FREQ_setMaxFreq(obj->VsFreqHandle, USER_MOTOR1_FREQ_MAX_HZ);

    VS_FREQ_setProfile(obj->VsFreqHandle,
                       USER_MOTOR1_FREQ_LOW_HZ, USER_MOTOR1_FREQ_HIGH_HZ,
                       USER_MOTOR1_VOLT_MIN_V, USER_MOTOR1_VOLT_MAX_V);
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_2)
#endif  // !MOTOR1_ISBLDC

#if defined(MOTOR1_ENC)
    // initialize the enc handle
    obj->encHandle = ENC_init(&enc_M1, sizeof(enc_M1));

    // set the ENC controller parameters
    ENC_setQEPHandle(obj->encHandle, MTR1_QEP_BASE);
    ENC_setParams(obj->encHandle, obj->userParamsHandle);
    ENC_setHallGPIO(obj->encHandle, MTR1_HALL_U_GPIO,
                    MTR1_HALL_V_GPIO, MTR1_HALL_W_GPIO);

    // initialize the apll handle
    obj->spdcalcHandle = SPDCALC_init(&speedcalc_M1, sizeof(speedcalc_M1));

    // set the SPEEDCALC controller parameters
    SPDCALC_setParams(obj->spdcalcHandle, obj->userParamsHandle);

    obj->frswPos_sf = 0.6f;     // Tune this coefficient per the motor/system
#endif  // MOTOR1_ENC

#if defined(MOTOR1_HALL)
    // initialize the hall handle
    obj->hallHandle = HALL_init(&hall_M1, sizeof(hall_M1));

    // set the HALL controller parameters
    HALL_setParams(obj->hallHandle, obj->userParamsHandle);
    HALL_setAngleBuf(obj->hallHandle, &hallAngleBuf[0]);
    HALL_setGPIOs(obj->hallHandle,
                  MTR1_HALL_U_GPIO, MTR1_HALL_V_GPIO, MTR1_HALL_W_GPIO);

    obj->frswPos_sf = 1.0f;     // Tune this coefficient per the motor/system
#endif  // MOTOR1_HALL

#if defined(MOTOR1_FAST)
    // initialize the Clarke modules
    obj->clarkeHandle_V = CLARKE_init(&clarke_V_M1, sizeof(clarke_V_M1));

    // set the Clarke parameters
    setupClarke_V(obj->clarkeHandle_V, objUser->numVoltageSensors);
#endif // MOTOR1_FAST

    // initialize the Clarke modules
    obj->clarkeHandle_I = CLARKE_init(&clarke_I_M1, sizeof(clarke_I_M1));

    // set the Clarke parameters
    setupClarke_I(obj->clarkeHandle_I, objUser->numCurrentSensors);

    // initialize the inverse Park module
    obj->iparkHandle_V = IPARK_init(&ipark_V_M1, sizeof(ipark_V_M1));

    // initialize the Park module
    obj->parkHandle_I = PARK_init(&park_I_M1, sizeof(park_I_M1));

    // initialize the Park module
    obj->parkHandle_V = PARK_init(&park_V_M1, sizeof(park_V_M1));

    // initialize the PI controllers
    obj->piHandle_Id  = PI_init(&pi_Id_M1, sizeof(pi_Id_M1));
    obj->piHandle_Iq  = PI_init(&pi_Iq_M1, sizeof(pi_Iq_M1));
    obj->piHandle_spd = PI_init(&pi_spd_M1, sizeof(pi_spd_M1));

    // initialize the space vector generator module
    obj->svgenHandle = SVGEN_init(&svgen_M1, sizeof(svgen_M1));

#if !defined(MOTOR1_ISBLDC)
    SVGEN_setMode(obj->svgenHandle, SVM_COM_C);
#endif  //! MOTOR1_ISBLDC

    HAL_setTriggerPrams(&obj->pwmData, USER_SYSTEM_FREQ_MHz, 0.2f, 0.1f);

#if defined(MOTOR1_OVM)
    // Initialize and setup the 100% SVM generator
    obj->svgencurrentHandle =
            SVGENCURRENT_init(&svgencurrent_M1, sizeof(svgencurrent_M1));

    SVGENCURRENT_setup(obj->svgencurrentHandle, 1.0f,
                       USER_M1_PWM_FREQ_kHz, USER_SYSTEM_FREQ_MHz);
#endif  // MOTOR1_OVM

    // initialize the speed reference trajectory
    obj->trajHandle_spd = TRAJ_init(&traj_spd_M1, sizeof(traj_spd_M1));

    // configure the speed reference trajectory (Hz)
    TRAJ_setTargetValue(obj->trajHandle_spd, 0.0f);
    TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
    TRAJ_setMinValue(obj->trajHandle_spd, -objUser->maxFrequency_Hz);
    TRAJ_setMaxValue(obj->trajHandle_spd, objUser->maxFrequency_Hz);
    TRAJ_setMaxDelta(obj->trajHandle_spd, (objUser->maxAccel_Hzps / objUser->ctrlFreq_Hz));

#if defined(BSXL8320RS_REVA) || defined(BSXL8323RS_REVA) || \
    defined(BSXL8353RS_REVA) || defined(BSXL8316RT_REVA)
    // turn on the DRV8323/DRV8353/DRV8316 if present
    HAL_enableDRV(obj->halMtrHandle);

    // initialize the DRV8323/DRV8353/DRV8316 interface
    HAL_setupDRVSPI(obj->halMtrHandle, &drvicVars_M1);

    // BSXL8320RS_REVA || BSXL8323RS_REVA || BSXL8353RS_REVA
#elif defined(BSXL8323RH_REVB) || defined(BSXL3PHGAN_REVA)
    // turn on the DRV8323RH/3PhGaN if present
    HAL_enableDRV(obj->halMtrHandle);
#elif defined(HVMTRPFC_REV1P1)
    HAL_enableDRV(obj->halMtrHandle);
#else //No Right Board
#error Not select a right board for this project
#endif // ! ALL Boards

#if defined(BSXL8320RS_REVA)
    drvicVars_M1.ctrlReg05.bit.VDS_LVL = DRV8320_VDS_LEVEL_1P500_V;
    drvicVars_M1.ctrlReg05.bit.OCP_MODE = DRV8320_AUTOMATIC_RETRY;
    drvicVars_M1.ctrlReg05.bit.DEAD_TIME = DRV8320_DEADTIME_100_NS;
#endif  // BSXL8320RS_REVA

#if defined(BSXL8323RS_REVA)
    drvicVars_M1.ctrlReg02.bit.OTW_REP = true;

#if defined(MOTOR1_ISBLDC)
    drvicVars_M1.ctrlReg02.bit.PWM_MODE = DRV8323_PWMMODE_3;
#else  // !MOTOR1_ISBLDC
    drvicVars_M1.ctrlReg02.bit.PWM_MODE = DRV8323_PWMMODE_6;
#endif  // !MOTOR1_ISBLDC

    drvicVars_M1.ctrlReg05.bit.VDS_LVL = DRV8323_VDS_LEVEL_1P700_V;
    drvicVars_M1.ctrlReg05.bit.OCP_MODE = DRV8323_AUTOMATIC_RETRY;
    drvicVars_M1.ctrlReg05.bit.DEAD_TIME = DRV8323_DEADTIME_100_NS;
    drvicVars_M1.ctrlReg06.bit.CSA_GAIN = DRV8323_Gain_10VpV;

    drvicVars_M1.ctrlReg06.bit.LS_REF = false;
    drvicVars_M1.ctrlReg06.bit.VREF_DIV = true;
    drvicVars_M1.ctrlReg06.bit.CSA_FET = false;
#endif  // BSXL8323RS_REVA

#if defined(BSXL8316RT_REVA)
    drvicVars_M1.ctrlReg02.bit.PWM_MODE = DRV8316_PWMMODE_6_N;
    drvicVars_M1.ctrlReg02.bit.SLEW = DRV8316_SLEW_200V;

    drvicVars_M1.ctrlReg05.bit.CSA_GAIN = DRV8316_CSA_GAIN_0p15VpA;

    drvicVars_M1.ctrlReg06.bit.BUCK_DIS = false;
    drvicVars_M1.ctrlReg06.bit.BUCK_SEL = DRV8316_BUCK_SEL_3p3V;
#endif  // BSXL8316RT_REVA

#if defined(BSXL8353RS_REVA)
    drvicVars_M1.ctrlReg03.bit.IDRIVEP_HS = DRV8353_ISOUR_HS_0P820_A;
    drvicVars_M1.ctrlReg03.bit.IDRIVEN_HS = DRV8353_ISINK_HS_1P640_A;

    drvicVars_M1.ctrlReg04.bit.IDRIVEP_LS = DRV8353_ISOUR_LS_0P820_A;
    drvicVars_M1.ctrlReg04.bit.IDRIVEN_LS = DRV8353_ISINK_LS_1P640_A;

    drvicVars_M1.ctrlReg05.bit.VDS_LVL = DRV8353_VDS_LEVEL_1P500_V;
    drvicVars_M1.ctrlReg05.bit.OCP_MODE = DRV8353_LATCHED_SHUTDOWN;
    drvicVars_M1.ctrlReg05.bit.DEAD_TIME = DRV8353_DEADTIME_100_NS;
    drvicVars_M1.ctrlReg06.bit.CSA_GAIN = DRV8353_Gain_10VpV;

    drvicVars_M1.ctrlReg06.bit.LS_REF = false;
    drvicVars_M1.ctrlReg06.bit.VREF_DIV = true;
    drvicVars_M1.ctrlReg06.bit.CSA_FET = false;
#endif  // BSXL8353RS_REVA


#if defined(BSXL8320RS_REVA) || defined(BSXL8323RS_REVA) || \
    defined(BSXL8353RS_REVA) || defined(BSXL8316RT_REVA)
    // disable the PWM
    HAL_disablePWM(obj->halMtrHandle);

    drvicVars_M1.writeCmd = 1;
    HAL_writeDRVData(obj->halMtrHandle, &drvicVars_M1);

    // disable the PWM
    HAL_disablePWM(obj->halMtrHandle);

    drvicVars_M1.writeCmd = 1;
    HAL_writeDRVData(obj->halMtrHandle, &drvicVars_M1);
#endif // BSXL8320RS_REVA || BSXL8323RS_REVA ||
       // BSXL8353RS_REVA || BSXL8316RT_REVA

#if defined(MOTOR1_FAST)
    // initialize the estimator
    obj->estHandle = EST_initEst(MTR_1);

    // set the default estimator parameters
    EST_setParams(obj->estHandle, obj->userParamsHandle);
    EST_setFlag_enableForceAngle(obj->estHandle, obj->flagEnableForceAngle);
    EST_setFlag_enableRsRecalc(obj->estHandle, obj->flagEnableRsRecalc);

    // set the scale factor for high frequency motor
    EST_setOneOverFluxGain_sf(obj->estHandle,
                              obj->userParamsHandle, USER_M1_EST_FLUX_HF_SF);
    EST_setFreqLFP_sf(obj->estHandle,
                      obj->userParamsHandle, USER_M1_EST_FREQ_HF_SF);
    EST_setBemf_sf(obj->estHandle,
                   obj->userParamsHandle, USER_M1_EST_BEMF_HF_SF);


    // if motor is an induction motor, configure default state of PowerWarp
    if(objUser->motor_type == MOTOR_TYPE_INDUCTION)
    {
        EST_setFlag_enablePowerWarp(obj->estHandle, obj->flagEnablePowerWarp);
        EST_setFlag_bypassLockRotor(obj->estHandle, obj->flagBypassLockRotor);
    }

    // for Rs online calibration
    obj->flagRsOnLineContinue = false;
    obj->flagStartRsOnLine = false;

    objSets->RsOnlineWaitTimeSet = USER_MOTOR1_RSONLINE_WAIT_TIME;
    objSets->RsOnlineWorkTimeSet = USER_MOTOR1_RSONLINE_WORK_TIME;
#endif // MOTOR1_FAST

#ifdef BRAKE_ENABLE

    obj->brakingCurrent_A = USER_MOTOR1_BRAKE_CURRENT_A;

    obj->brakingTimeDelay = USER_MOTOR1_BRAKE_TIME_DELAY;

    obj->flagEnableBraking = false;
    obj->brakingMode = HARDSWITCH_BRAKE_MODE;
#endif  // BRAKE_ENABLE

    // setup the controllers, speed, d/q-axis current pid regulator
    setupControllers(handle);

    // setup faults
    HAL_setupMtrFaults(obj->halMtrHandle);

    // disable the PWM
    HAL_disablePWM(obj->halMtrHandle);

    return;
}   // end of initMotor1CtrlParameters() function

void runMotor1OffsetsCalculation(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(handle->motorSetsHandle);

    // calculate motor and pfc protection value
    calcMotorOverCurrentThreshold(handle);

    HAL_setMtrCMPSSDACValue(obj->halMtrHandle,
                            objSets->dacCMPValH, objSets->dacCMPValL);

#if defined(MOTOR1_DCLINKSS)
    HAL_MTR_Obj *objHal = (HAL_MTR_Obj *)(obj->halMtrHandle);

    EPWM_setCounterCompareValue(objHal->pwmHandle[1],
                                EPWM_COUNTER_COMPARE_C, 5);
    EPWM_setCounterCompareValue(objHal->pwmHandle[1],
                                EPWM_COUNTER_COMPARE_D, 5);

    EPWM_setCounterCompareValue(objHal->pwmHandle[2],
                                EPWM_COUNTER_COMPARE_C, 5);
    EPWM_setCounterCompareValue(objHal->pwmHandle[2],
                                EPWM_COUNTER_COMPARE_D, 5);
#endif  // MOTOR1_DCLINKSS

        // Offsets in phase current sensing
#if defined(MOTOR1_ISBLDC)
    ADC_setPPBReferenceOffset(MTR1_IDC1_ADC_BASE, MTR1_IDC1_ADC_PPB_NUM,
                              MTR1_IDC1_ADC_PPB_NUM);

    ADC_setPPBReferenceOffset(MTR1_IDC2_ADC_BASE, MTR1_IDC2_ADC_PPB_NUM,
                              MTR1_IDC2_ADC_PPB_NUM);

    obj->adcData.offset_Idc_ad = USER_M1_IDC_OFFSET_AD;
#elif defined(MOTOR1_DCLINKSS)
    ADC_setPPBReferenceOffset(MTR1_IDC1_ADC_BASE, MTR1_IDC1_ADC_PPB_NUM,
                              MTR1_IDC1_ADC_PPB_NUM);

    ADC_setPPBReferenceOffset(MTR1_IDC2_ADC_BASE, MTR1_IDC2_ADC_PPB_NUM,
                              MTR1_IDC2_ADC_PPB_NUM);

    ADC_setPPBReferenceOffset(MTR1_IDC3_ADC_BASE, MTR1_IDC3_ADC_PPB_NUM,
                              MTR1_IDC3_ADC_PPB_NUM);

    ADC_setPPBReferenceOffset(MTR1_IDC4_ADC_BASE, MTR1_IDC4_ADC_PPB_NUM,
                              MTR1_IDC4_ADC_PPB_NUM);

    obj->adcData.offset_Idc_ad = USER_M1_IDC_OFFSET_AD;
#else // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)
    ADC_setPPBReferenceOffset(MTR1_IU_ADC_BASE, MTR1_IU_ADC_PPB_NUM,
                              USER_M1_IA_OFFSET_AD);

    ADC_setPPBReferenceOffset(MTR1_IV_ADC_BASE, MTR1_IV_ADC_PPB_NUM,
                              USER_M1_IB_OFFSET_AD);

    ADC_setPPBReferenceOffset(MTR1_IW_ADC_BASE, MTR1_IW_ADC_PPB_NUM,
                              USER_M1_IC_OFFSET_AD);

    obj->adcData.offset_I_ad.value[0]  = USER_M1_IA_OFFSET_AD;
    obj->adcData.offset_I_ad.value[1]  = USER_M1_IB_OFFSET_AD;
    obj->adcData.offset_I_ad.value[2]  = USER_M1_IC_OFFSET_AD;
#endif // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)

#if defined(MOTOR1_FAST) || defined(MOTOR1_ISBLDC)
    // Offsets in phase voltage sensing
    obj->adcData.offset_V_sf.value[0]  = USER_M1_VA_OFFSET_SF;
    obj->adcData.offset_V_sf.value[1]  = USER_M1_VB_OFFSET_SF;
    obj->adcData.offset_V_sf.value[2]  = USER_M1_VC_OFFSET_SF;
#endif  // MOTOR1_FAST ||  MOTOR1_ISBLDC

    if(obj->flagEnableOffsetCalc == true)
    {
        float32_t offsetK1 = 0.998001f;  // Offset filter coefficient K1: 0.05/(T+0.05);
        float32_t offsetK2 = 0.001999f;  // Offset filter coefficient K2: T/(T+0.05);
        float32_t invCurrentSf = 1.0f / obj->adcData.current_sf;

#if defined(MOTOR1_FAST) || defined(MOTOR1_ISBLDC)
        float32_t invVdcbus;
#endif  // MOTOR1_FAST ||  MOTOR1_ISBLDC

        uint16_t offsetCnt;

        SysCtl_delay(50U);

#if defined(MOTOR1_ISBLDC)
        uint16_t period = EPWM_getTimeBasePeriod(MTR1_PWM_U_BASE);
        EPWM_setCounterCompareValue(MTR1_PWM_U_BASE,
                                    EPWM_COUNTER_COMPARE_A, (period >> 1));
        EPWM_setCounterCompareValue(MTR1_PWM_V_BASE,
                                    EPWM_COUNTER_COMPARE_A, (period >> 1));
        EPWM_setCounterCompareValue(MTR1_PWM_W_BASE,
                                    EPWM_COUNTER_COMPARE_A, (period >> 1));

        ADC_setPPBReferenceOffset(MTR1_IDC1_ADC_BASE, MTR1_IDC1_ADC_PPB_NUM, 0);
        ADC_setPPBReferenceOffset(MTR1_IDC2_ADC_BASE, MTR1_IDC2_ADC_PPB_NUM, 0);

        // enable the PWM
        HAL_enablePWM(obj->halMtrHandle);
#elif defined(MOTOR1_DCLINKSS)
        HAL_setOffsetTrigger(obj->halMtrHandle);

        ADC_setPPBReferenceOffset(MTR1_IDC1_ADC_BASE, MTR1_IDC1_ADC_PPB_NUM, 0);
        ADC_setPPBReferenceOffset(MTR1_IDC2_ADC_BASE, MTR1_IDC2_ADC_PPB_NUM, 0);
        ADC_setPPBReferenceOffset(MTR1_IDC3_ADC_BASE, MTR1_IDC3_ADC_PPB_NUM, 0);
        ADC_setPPBReferenceOffset(MTR1_IDC4_ADC_BASE, MTR1_IDC4_ADC_PPB_NUM, 0);

        obj->adcData.offset_Idc_ad  = USER_M1_IDC_OFFSET_AD * USER_M1_CURRENT_SF;

        // Set the 3-phase output PWMs to 50% duty cycle
        obj->pwmData.Vabc_pu.value[0] = 0.0f;
        obj->pwmData.Vabc_pu.value[1] = 0.0f;
        obj->pwmData.Vabc_pu.value[2] = 0.0f;

        // write the PWM compare values
        HAL_writePWMData(obj->halMtrHandle, &obj->pwmData);

        // enable the PWM
        HAL_enablePWM(obj->halMtrHandle);
#else  // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)
        ADC_setPPBReferenceOffset(MTR1_IU_ADC_BASE, MTR1_IU_ADC_PPB_NUM, 0);
        ADC_setPPBReferenceOffset(MTR1_IV_ADC_BASE, MTR1_IV_ADC_PPB_NUM, 0);
        ADC_setPPBReferenceOffset(MTR1_IW_ADC_BASE, MTR1_IW_ADC_PPB_NUM, 0);

        obj->adcData.offset_I_ad.value[0] =
                 obj->adcData.offset_I_ad.value[0] * obj->adcData.current_sf;
        obj->adcData.offset_I_ad.value[1] =
                 obj->adcData.offset_I_ad.value[1] * obj->adcData.current_sf;
        obj->adcData.offset_I_ad.value[2] =
                 obj->adcData.offset_I_ad.value[2] * obj->adcData.current_sf;

        // Set the 3-phase output PWMs to 50% duty cycle
        obj->pwmData.Vabc_pu.value[0] = 0.0f;
        obj->pwmData.Vabc_pu.value[1] = 0.0f;
        obj->pwmData.Vabc_pu.value[2] = 0.0f;

        // write the PWM compare values
        HAL_writePWMData(obj->halMtrHandle, &obj->pwmData);

        // enable the PWM
        HAL_enablePWM(obj->halMtrHandle);
#endif // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)

        for(offsetCnt = 0; offsetCnt < 32000; offsetCnt++)
        {
            // clear the ADC interrupt flag
            ADC_clearInterruptStatus(MTR1_ADC_INT_BASE, MTR1_ADC_INT_NUM);

            while(ADC_getInterruptStatus(MTR1_ADC_INT_BASE, MTR1_ADC_INT_NUM) == false);

            HAL_readMtr1ADCData(&obj->adcData);

            if(offsetCnt >= 2000)       // Ignore the first 2000 times
            {
                // Offsets in phase current sensing
#if defined(MOTOR1_ISBLDC)
                obj->adcData.offset_Idc_ad = offsetK1 * obj->adcData.offset_Idc_ad +
                         (obj->adcData.Idc1_A.value[0] + obj->adcData.Idc1_A.value[1]) * 0.5f * offsetK2;
#elif defined(MOTOR1_DCLINKSS)
                obj->adcData.offset_Idc_ad = offsetK1 * obj->adcData.offset_Idc_ad +
                               0.25f * offsetK2 *(obj->adcData.Idc1_A.value[0] +
                                                  obj->adcData.Idc1_A.value[1] +
                                                  obj->adcData.Idc2_A.value[0] +
                                                  obj->adcData.Idc2_A.value[1]);
#else // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)
                obj->adcData.offset_I_ad.value[0] =
                        offsetK1 * obj->adcData.offset_I_ad.value[0] +
                        obj->adcData.I_A.value[0] * offsetK2;

                obj->adcData.offset_I_ad.value[1] =
                        offsetK1 * obj->adcData.offset_I_ad.value[1] +
                        obj->adcData.I_A.value[1] * offsetK2;

                obj->adcData.offset_I_ad.value[2] =
                        offsetK1 * obj->adcData.offset_I_ad.value[2] +
                        obj->adcData.I_A.value[2] * offsetK2;
#endif // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)

#if defined(MOTOR1_FAST) || defined(MOTOR1_ISBLDC)
                invVdcbus = 1.0f / obj->adcData.VdcBus_V;

                // Offsets in phase voltage sensing
                obj->adcData.offset_V_sf.value[0] =
                         offsetK1 * obj->adcData.offset_V_sf.value[0] +
                         (invVdcbus * obj->adcData.V_V.value[0]) * offsetK2;

                obj->adcData.offset_V_sf.value[1] =
                         offsetK1 * obj->adcData.offset_V_sf.value[1] +
                         (invVdcbus * obj->adcData.V_V.value[1]) * offsetK2;

                obj->adcData.offset_V_sf.value[2] =
                         offsetK1 * obj->adcData.offset_V_sf.value[2] +
                         (invVdcbus * obj->adcData.V_V.value[2]) * offsetK2;
#endif  // MOTOR1_FAST ||  MOTOR1_ISBLDC

            }
            else
            {
                // enable the PWM
                HAL_enablePWM(obj->halMtrHandle);
            }
        } // for()

        // disable the PWM
        HAL_disablePWM(obj->halMtrHandle);

        obj->flagEnableOffsetCalc = false;

#if defined(MOTOR1_ISBLDC)
        obj->adcData.offset_Idc_ad = obj->adcData.offset_Idc_ad * invCurrentSf;

        ADC_setPPBReferenceOffset(MTR1_IDC1_ADC_BASE, MTR1_IDC1_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_Idc_ad);

        ADC_setPPBReferenceOffset(MTR1_IDC2_ADC_BASE, MTR1_IDC2_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_Idc_ad);

        ISBLDC_setVabcOffset(obj->isbldcHandle, &obj->adcData.offset_V_sf);
#elif defined(MOTOR1_DCLINKSS)
        obj->adcData.offset_Idc_ad = obj->adcData.offset_Idc_ad * invCurrentSf;

        ADC_setPPBReferenceOffset(MTR1_IDC1_ADC_BASE, MTR1_IDC1_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_Idc_ad);

        ADC_setPPBReferenceOffset(MTR1_IDC2_ADC_BASE, MTR1_IDC2_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_Idc_ad);

        ADC_setPPBReferenceOffset(MTR1_IDC3_ADC_BASE, MTR1_IDC3_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_Idc_ad);

        ADC_setPPBReferenceOffset(MTR1_IDC4_ADC_BASE, MTR1_IDC4_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_Idc_ad);
#else // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)
        obj->adcData.offset_I_ad.value[0] =
                 obj->adcData.offset_I_ad.value[0] * invCurrentSf;
        obj->adcData.offset_I_ad.value[1] =
                 obj->adcData.offset_I_ad.value[1] * invCurrentSf;
        obj->adcData.offset_I_ad.value[2] =
                 obj->adcData.offset_I_ad.value[2] * invCurrentSf;

        ADC_setPPBReferenceOffset(MTR1_IU_ADC_BASE, MTR1_IU_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_I_ad.value[0]);

        ADC_setPPBReferenceOffset(MTR1_IV_ADC_BASE, MTR1_IV_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_I_ad.value[1]);

        ADC_setPPBReferenceOffset(MTR1_IW_ADC_BASE, MTR1_IW_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_I_ad.value[2]);
#endif // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)
    }   // flagEnableOffsetCalc = true

#if defined(MOTOR1_ISBLDC)
    // Check current and voltage offset
    if( (obj->adcData.offset_Idc_ad > USER_M1_IDC_OFFSET_AD_MAX) ||
        (obj->adcData.offset_Idc_ad < USER_M1_IDC_OFFSET_AD_MIN) )
    {
        obj->faultMtrNow.bit.currentOffset = 1;
    }
#elif defined(MOTOR1_DCLINKSS)
    // Check current and voltage offset
    if( (obj->adcData.offset_Idc_ad > USER_M1_IDC_OFFSET_AD_MAX) ||
        (obj->adcData.offset_Idc_ad < USER_M1_IDC_OFFSET_AD_MIN) )
    {
        obj->faultMtrNow.bit.currentOffset = 1;
    }
#else // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)
    // Check current and voltage offset
    if( (obj->adcData.offset_I_ad.value[0] > USER_M1_IA_OFFSET_AD_MAX) ||
        (obj->adcData.offset_I_ad.value[0] < USER_M1_IA_OFFSET_AD_MIN) )
    {
        obj->faultMtrNow.bit.currentOffset = 1;
    }

    if( (obj->adcData.offset_I_ad.value[1] > USER_M1_IB_OFFSET_AD_MAX) ||
        (obj->adcData.offset_I_ad.value[1] < USER_M1_IB_OFFSET_AD_MIN) )
    {
        obj->faultMtrNow.bit.currentOffset = 1;
    }

    if( (obj->adcData.offset_I_ad.value[2] > USER_M1_IC_OFFSET_AD_MAX) ||
        (obj->adcData.offset_I_ad.value[2] < USER_M1_IC_OFFSET_AD_MIN) )
    {
        obj->faultMtrNow.bit.currentOffset = 1;
    }
#endif // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)

#if defined(MOTOR1_FAST) || defined(MOTOR1_ISBLDC)
    if( (obj->adcData.offset_V_sf.value[0] > USER_M1_VA_OFFSET_SF_MAX) ||
        (obj->adcData.offset_V_sf.value[0] < USER_M1_VA_OFFSET_SF_MIN) )
    {
        obj->faultMtrNow.bit.voltageOffset = 1;
    }

    if( (obj->adcData.offset_V_sf.value[1] > USER_M1_VB_OFFSET_SF_MAX) ||
        (obj->adcData.offset_V_sf.value[1] < USER_M1_VB_OFFSET_SF_MIN) )
    {
        obj->faultMtrNow.bit.voltageOffset = 1;
    }

    if( (obj->adcData.offset_V_sf.value[2] > USER_M1_VC_OFFSET_SF_MAX) ||
        (obj->adcData.offset_V_sf.value[2] < USER_M1_VC_OFFSET_SF_MIN) )
    {
        obj->faultMtrNow.bit.voltageOffset = 1;
    }
#endif  // MOTOR1_FAST ||  MOTOR1_ISBLDC

    return;
} // end of runMotor1OffsetsCalculation() function


void runMotor1Control(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(handle->motorSetsHandle);
    USER_Params *objUser = (USER_Params *)(handle->userParamsHandle);

    if(HAL_getPwmEnableStatus(obj->halMtrHandle) == true)
    {
        if(HAL_getMtrTripFaults(obj->halMtrHandle) != 0)
        {
            obj->faultMtrNow.bit.moduleOverCurrent = 1;
        }
    }

    obj->faultMtrPrev.all |= obj->faultMtrNow.all;
    obj->faultMtrUse.all = obj->faultMtrNow.all & obj->faultMtrMask.all;

    HAL_setMtrCMPSSDACValue(obj->halMtrHandle,
                            objSets->dacCMPValH, objSets->dacCMPValL);

    if(obj->flagEnableRunAndIdentify == true)
    {
        // Had some faults to stop the motor
        if(obj->faultMtrUse.all != 0)
        {
            if(obj->flagRunIdentAndOnLine == true)
            {
                obj->flagRunIdentAndOnLine = false;
                obj->mctrlState = MCTRL_FAULT_STOP;

                obj->stopWaitTimeCnt = objSets->restartWaitTimeSet;
                obj->restartTimesCnt++;

                if(obj->flagEnableRestart == false)
                {
                    obj->flagEnableRunAndIdentify = false;
                    obj->stopWaitTimeCnt = 0;
                }
            }
            else if(obj->stopWaitTimeCnt == 0)
            {
                if(obj->restartTimesCnt < objSets->restartTimesSet)
                {
                    obj->flagClearFaults = 1;
                }
                else
                {
                    obj->flagEnableRunAndIdentify = false;
                }
            }
        }
        // Restart
        else if((obj->flagRunIdentAndOnLine == false) &&
                (obj->stopWaitTimeCnt == 0))
        {
#if defined(MOTOR1_FAST)
            if(obj->flagEnableMotorIdentify == true)
            {
                obj->estimatorMode = ESTIMATOR_MODE_FAST;
                obj->flagMotorIdentified = false;
                obj->flagSetupController = false;
                obj->svmMode = SVM_COM_C;
                obj->flagEnableFWC = false;
                obj->flagEnableMTPA = false;

                obj->speedRef_Hz = objUser->fluxExcFreq_Hz;

                objUser->flag_bypassMotorId = false;

                // disable interrupts
                DINT;
                __asm("  NOP");

                // set the default estimator parameters
                EST_setParams(obj->estHandle, obj->userParamsHandle);

                // enable interrupts
                EINT;
                __asm("  NOP");

                obj->flagEnableMotorIdentify = false;
            }
#endif  // MOTOR1_FAST

#if defined(MOTOR1_SSIPD)
            if(obj->flagEnableIPD == true)
            {
                if(SSIPD_getDoneStatus(obj->ssipdHandle) == true)
                {
                    if(obj->speedRef_Hz > 0.0f)
                    {
                        obj->angleDetectIPD_rad =
                                SSIPD_getAngleOut_rad(obj->ssipdHandle) -
                                obj->angleOffsetIPD_rad;

                    }
                    else
                    {
                        obj->angleDetectIPD_rad =
                                SSIPD_getAngleOut_rad(obj->ssipdHandle) +
                                obj->angleOffsetIPD_rad;
                    }

                    if(obj->angleDetectIPD_rad < 0.0f)
                    {
                        obj->angleDetectIPD_rad += MATH_TWO_PI;
                    }
                    else if(obj->angleDetectIPD_rad > MATH_TWO_PI)
                    {
                        obj->angleDetectIPD_rad -= MATH_TWO_PI;
                    }

#if defined(MOTOR1_FAST)
                    EST_setAngle_rad(obj->estHandle,
                                     obj->angleDetectIPD_rad);
#endif // MOTOR1_FAST

                    restartMotorControl(handle);
                }
                else if(SSIPD_getRunState(obj->ssipdHandle) == true)
                {
                    if(SSIPD_getFlagEnablePWM(obj->ssipdHandle) == true)
                    {
                        if(HAL_getPwmEnableStatus(obj->halMtrHandle) == false)
                        {
                            // enable the PWM
                            HAL_enablePWM(obj->halMtrHandle);
                        }
                    }
                    else
                    {
                        if(HAL_getPwmEnableStatus(obj->halMtrHandle) == true)
                        {
                            // disable the PWM
                            HAL_disablePWM(obj->halMtrHandle);
                        }
                    }
                }
                else
                {
                    SSIPD_start(obj->ssipdHandle);
                }
            }
            else    // obj->flagEnableIPD = false
            {
                restartMotorControl(handle);
            }
#else  // !MOTOR1_SSIPD
            restartMotorControl(handle);
#endif  // MOTOR1_SSIPD

#if defined(MOTOR1_ESMO)
            ESMO_resetParams(obj->esmoHandle);
#endif  //MOTOR1_ESMO
        }
    }
    // if(obj->flagEnableRunAndIdentify == false)
    else if(obj->flagRunIdentAndOnLine == true)
    {
        stopMotorControl(handle);

        if(obj->flagEnableFlyingStart == false)
        {
            obj->stopWaitTimeCnt = objSets->stopWaitTimeSet;
        }
        else
        {
            obj->stopWaitTimeCnt = 0;
        }
    }
    else
    {
#if defined(MOTOR1_SSIPD)
        // Reset
        if(SSIPD_getDoneStatus(obj->ssipdHandle) == true)
        {
            SSIPD_reset(obj->ssipdHandle);
        }
#endif // MOTOR1_SSIPD
    }

#if defined(MOTOR1_FAST)
    // enable or disable bypassLockRotor flag
    if(objUser->motor_type == MOTOR_TYPE_INDUCTION)
    {
        EST_setFlag_bypassLockRotor(obj->estHandle,
                                    obj->flagBypassLockRotor);
    }
#endif // MOTOR1_FAST

    if(obj->flagClearFaults == true)
    {
        HAL_clearMtrFaultStatus(obj->halMtrHandle);

        obj->faultMtrNow.all &= MTR_FAULT_CLEAR;
        obj->flagClearFaults = false;
    }

    if(obj->flagRunIdentAndOnLine == true)
    {
        if(HAL_getPwmEnableStatus(obj->halMtrHandle) == false)
        {
#if defined(MOTOR1_FAST)
            // enable the estimator
            EST_enable(obj->estHandle);

            // enable the trajectory generator
            EST_enableTraj(obj->estHandle);
#endif // MOTOR1_FAST

            // enable the PWM
            HAL_enablePWM(obj->halMtrHandle);
        }

        if(obj->flagMotorIdentified == true)
        {
            if(obj->speedRef_Hz > 0.0f)
            {
                obj->direction = 1.0f;
            }
            else
            {
                obj->direction = -1.0f;
            }

            #if defined(MOTOR1_FAST)
            // enable or disable force angle
            EST_setFlag_enableForceAngle(obj->estHandle,
                                         obj->flagEnableForceAngle);

            EST_setFlag_enableRsRecalc(obj->estHandle,
                                       obj->flagEnableRsRecalc);
            #endif  // MOTOR1_FAST

            #if defined(MOTOR1_ESMO)
            if(obj->motorState >= MOTOR_CL_RUNNING)
            {
                TRAJ_setTargetValue(obj->trajHandle_spd, obj->speedRef_Hz);
            }
            else
            {
                TRAJ_setTargetValue(obj->trajHandle_spd,
                                    (obj->speedForce_Hz * obj->direction));
            }
            #elif defined(MOTOR1_ISBLDC)
            if(obj->motorState >= MOTOR_CL_RUNNING)
            {
                TRAJ_setTargetValue(obj->trajHandle_spd, obj->speedRef_Hz);
            }
            else
            {
                obj->speed_int_Hz = obj->speedForce_Hz * obj->direction;
                TRAJ_setTargetValue(obj->trajHandle_spd,
                                    (obj->speedForce_Hz * obj->direction));
            }
            #elif defined(MOTOR1_ENC)
            if(obj->motorState >= MOTOR_CL_RUNNING)
            {
                TRAJ_setTargetValue(obj->trajHandle_spd, obj->speedRef_Hz);
            }
            else
            {
                TRAJ_setTargetValue(obj->trajHandle_spd,
                                    (obj->speedForce_Hz * obj->direction));
            }
            #elif defined(MOTOR1_HALL)
            if(obj->motorState >= MOTOR_CL_RUNNING)
            {
                TRAJ_setTargetValue(obj->trajHandle_spd, obj->speedRef_Hz);
            }
            else
            {
                TRAJ_setTargetValue(obj->trajHandle_spd,
                                    (obj->speedForce_Hz * obj->direction));
            }
            #elif defined(MOTOR1_FAST)
            TRAJ_setTargetValue(obj->trajHandle_spd, obj->speedRef_Hz);
            #else   // !MOTOR1_ESMO && !MOTOR1_FAST
            #error No select a right estimator for motor_1 control
            #endif  // MOTOR1_ESMO || MOTOR1_FAST

            if(fabsf(obj->speed_Hz) > obj->speedStart_Hz)
            {
                TRAJ_setMaxDelta(obj->trajHandle_spd,
                  (obj->accelerationMax_Hzps / objUser->ctrlFreq_Hz));

            #if defined(MOTOR1_ISBLDC)
                ISBLDC_updateThresholdInt(obj->isbldcHandle, obj->speed_int_Hz);
                #if (DMC_BUILDLEVEL >= DMC_LEVEL_4)
                PI_setMinMax(obj->piHandle_spd, -obj->maxCurrent_A, obj->maxCurrent_A);
                #else
                PI_setMinMax(obj->piHandle_spd, -1.0f, 1.0f);
                #endif  // DMC_BUILDLEVEL <= DMC_LEVEL_3
            #else  // !MOTOR1_ISBLDC
                PI_setMinMax(obj->piHandle_spd, -obj->maxCurrent_A, obj->maxCurrent_A);

                SVGEN_setMode(obj->svgenHandle, obj->svmMode);
            #endif  // !MOTOR1_ISBLDC

                if(obj->motorState == MOTOR_CL_RUNNING)
                {
                    obj->stateRunTimeCnt++;

                    if(obj->stateRunTimeCnt == obj->fwcTimeDelay)
                    {
                        obj->Idq_out_A.value[0] = 0.0f;
                        obj->motorState = MOTOR_CTRL_RUN;
                        obj->mctrlState = MCTRL_CONT_RUN;
                    }
                }
            }
            else
            {
                TRAJ_setMaxDelta(obj->trajHandle_spd,
                  (obj->accelerationStart_Hzps / objUser->ctrlFreq_Hz));

            #if defined(MOTOR1_ISBLDC)
                #if (DMC_BUILDLEVEL >= DMC_LEVEL_4)
                if(obj->speed_int_Hz > 0.0f)
                {
                    PI_setMinMax(obj->piHandle_spd, 0.0f, obj->startCurrent_A);
                }
                else
                {
                    PI_setMinMax(obj->piHandle_spd, -obj->startCurrent_A, 0.0f);
                }
                #else   // (DMC_BUILDLEVEL < DMC_LEVEL_3)
                PI_setMinMax(obj->piHandle_spd, -1.0f, 1.0f);
                #endif  // DMC_BUILDLEVEL < DMC_LEVEL_3
            #else  // !MOTOR1_ISBLDC
                if(obj->speed_int_Hz >= 0.0f)
                {
                    PI_setMinMax(obj->piHandle_spd, 0.0f, obj->startCurrent_A);
                }
                else
                {
                    PI_setMinMax(obj->piHandle_spd, -obj->startCurrent_A, 0.0f);
                }
            #endif  // !MOTOR1_ISBLDC
            }
        }

        // Identification
#if(DMC_BUILDLEVEL == DMC_LEVEL_3)
        obj->Idq_out_A.value[0] = obj->Idq_set_A.value[0];
        obj->Idq_out_A.value[1] = obj->Idq_set_A.value[1] * obj->direction;

#endif // (DMC_BUILDLEVEL == DMC_LEVEL_3)
    }
#if defined(MOTOR1_SSIPD)
    else if(SSIPD_getRunState(obj->ssipdHandle) == false)
#else
    else
#endif  // MOTOR1_SSIPD
    {
        // reset motor control parameters
        resetMotorControl(handle);
    }

#if defined(MOTOR1_FAST)
    // check the trajectory generator
    if(EST_isTrajError(obj->estHandle) == true)
    {
        // disable the PWM
        HAL_disablePWM(obj->halMtrHandle);
    }
    else
    {
        // update the trajectory generator state
        EST_updateTrajState(obj->estHandle);
    }

    // check the estimator
    if(EST_isError(obj->estHandle) == true)
    {
        // disable the PWM
        HAL_disablePWM(obj->halMtrHandle);
    }
    else        // No any estimator error
    {
        bool flagEstStateChanged = false;

        float32_t Id_target_A = EST_getIntValue_Id_A(obj->estHandle);

        if(obj->flagMotorIdentified == true)
        {
            flagEstStateChanged = EST_updateState(obj->estHandle, 0.0f);
        }
        else
        {
            flagEstStateChanged = EST_updateState(obj->estHandle, Id_target_A);
        }

        if(flagEstStateChanged == true)
        {
            // configure the trajectory generator
            EST_configureTraj(obj->estHandle);

            if(obj->flagMotorIdentified == false)
            {
                // configure the controllers
                EST_configureTrajState(obj->estHandle, obj->userParamsHandle,
                                       obj->piHandle_spd,
                                       obj->piHandle_Id, obj->piHandle_Iq);
            }

            if(objUser->flag_bypassMotorId == false)
            {
                if((EST_isLockRotor(obj->estHandle) == true) ||
                        ( (EST_isMotorIdentified(obj->estHandle) == true)
                                  && (EST_isIdle(obj->estHandle) == true) ) )
                {
                    if(EST_isMotorIdentified(obj->estHandle) == true)
                    {
                        obj->flagMotorIdentified = true;

                        // clear the flag
                        obj->flagRunIdentAndOnLine = false;
                        obj->flagEnableRunAndIdentify = false;
                        objUser->flag_bypassMotorId = true;

                        // disable the estimator
                        EST_disable(obj->estHandle);

                        // enable the trajectory generator
                        EST_disableTraj(obj->estHandle);
                    }

                    if(objUser->motor_type == MOTOR_TYPE_INDUCTION)
                    {
                        // clear the flag
                        obj->flagRunIdentAndOnLine = false;
                        obj->flagEnableRunAndIdentify = false;
                    }
                }
            }
        }
    }

    obj->flagMotorIdentified = EST_isMotorIdentified(obj->estHandle);
#else
    obj->flagMotorIdentified = true;
#endif // MOTOR1_FAST

    if(obj->flagMotorIdentified == true)
    {
        if(obj->flagSetupController == true)
        {
            // update the controller
            updateControllers(handle);
        }
        else
        {
            obj->flagSetupController = true;

            setupControllers(handle);
        }
    }

#if defined(MOTOR1_FAST)
    // run Rs online
    runRsOnLine(handle);

    // update the global variables
    updateGlobalVariables(handle);
#endif // MOTOR1_FAST

#if defined(MOTOR1_ESMO)
    if(obj->motorState >= MOTOR_CTRL_RUN)
    {
        ESMO_updateFilterParams(obj->esmoHandle);
        ESMO_updatePLLParams(obj->esmoHandle);
    }
#endif  // MOTOR2_ESMO

    return;
}   // end of the runMotor1Control() function


__interrupt void motor1CtrlISR(void)
{


    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)motorHandle_M1;
    USER_Params *objUser = (USER_Params *)(obj->userParamsHandle);

#if defined(MOTOR1_DECOUP)
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(obj->motorSetsHandle);
#endif  // MOTOR1_DECOUP

    // acknowledge the ADC interrupt
    HAL_ackMtr1ADCInt();

    // read the ADC data with offsets
    HAL_readMtr1ADCData(&obj->adcData);

//------------------------------------------------------------------------------
//**!!! ISBLDC only supports one direction rotation (speed_ref = positive) !!
#if defined(MOTOR1_ISBLDC)
    obj->bldcHandle->IdcIn =
            (obj->adcData.Idc1_A.value[0] + obj->adcData.Idc1_A.value[1]) * 0.5f;

    if(obj->bldcHandle->commSampleCount >= obj->bldcHandle->commSampleDelay)
    {
        obj->bldcHandle->IdcInFilter = (obj->bldcHandle->IdcInBuff[2] + \
                                        obj->bldcHandle->IdcInBuff[1] + \
                                        obj->bldcHandle->IdcInBuff[0] + \
                                        obj->bldcHandle->IdcIn) * 0.25f;

        obj->bldcHandle->IdcInBuff[2] = obj->bldcHandle->IdcInBuff[1];
        obj->bldcHandle->IdcInBuff[1] = obj->bldcHandle->IdcInBuff[0];
        obj->bldcHandle->IdcInBuff[0] = obj->bldcHandle->IdcIn;
    }

    if(obj->bldcHandle->commTrigFlag == true)
    {
        obj->bldcHandle->commSampleCount = 0;
    }

    obj->bldcHandle->commSampleCount++;

    if((obj->flagRunIdentAndOnLine == true) && (obj->motorState >= MOTOR_CL_RUNNING))
    {
        obj->counterTrajSpeed++;

        if(obj->counterTrajSpeed >= objUser->numIsrTicksPerTrajTick)
        {
            // clear counter
            obj->counterTrajSpeed = 0;

            // run a trajectory for speed reference,
            // so the reference changes with a ramp instead of a step
            TRAJ_run(obj->trajHandle_spd);
        }

        obj->speed_int_Hz = TRAJ_getIntValue(obj->trajHandle_spd);
    }

    MOD6CNT_setFlagCount(obj->mod6cntHandle, obj->bldcHandle->commTrigFlag);

    MOD6CNT_run(obj->mod6cntHandle, obj->speed_int_Hz);

    obj->bldcHandle->commState = MOD6CNT_getCounter(obj->mod6cntHandle);

    ISBLDC_setCommState(obj->isbldcHandle, obj->bldcHandle->commState);

    ISBLDC_run(obj->isbldcHandle, &obj->adcData.V_V);

    obj->speedINT_Hz = ISBLDC_getSpeedINT(obj->isbldcHandle);
    obj->speed_Hz = obj->speedINT_Hz * obj->direction;

#if (DMC_BUILDLEVEL == DMC_LEVEL_1)
    RIMPULSE_run(obj->rimpulseHandle);
    obj->bldcHandle->commTrigFlag = RIMPULSE_getTrigFlag(obj->rimpulseHandle);

    obj->bldcHandle->pwmDuty = obj->bldcHandle->pwmDutySet;
#endif  // DMC_LEVEL_1

#if (DMC_BUILDLEVEL == DMC_LEVEL_2)
    if(obj->motorState >= MOTOR_CL_RUNNING)
    {
        obj->bldcHandle->commTrigFlag = ISBLDC_getCommTrigFlag(obj->isbldcHandle);
        obj->bldcHandle->pwmDuty = obj->bldcHandle->pwmDutySet;
    }
    else if(obj->motorState == MOTOR_OL_START)
    {
        RIMPULSE_run(obj->rimpulseHandle);

        obj->bldcHandle->commTrigFlag = RIMPULSE_getTrigFlag(obj->rimpulseHandle);

        obj->bldcHandle->pwmDuty = obj->bldcHandle->pwmDutyStart;
        obj->enableSpeedCtrl = false;

        if(RIMPULSE_getRmpDoneFlag(obj->rimpulseHandle) == true)
        {
            obj->stateRunTimeCnt++;

            if(obj->stateRunTimeCnt > obj->forceRunTimeDelay)
            {
                obj->stateRunTimeCnt = 0;
                obj->motorState = MOTOR_CL_RUNNING;
            }
        }
        else
        {
            obj->bldcHandle->commTrigFlag = RIMPULSE_getTrigFlag(obj->rimpulseHandle);
            obj->stateRunTimeCnt = 0;
        }
    }
    else if(obj->motorState == MOTOR_ALIGNMENT)
    {
        obj->bldcHandle->commTrigFlag = false;
        obj->enableSpeedCtrl = false;

        MOD6CNT_setCounter(obj->mod6cntHandle, 0);

        obj->bldcHandle->pwmDuty = obj->bldcHandle->pwmDutyStart;

        obj->stateRunTimeCnt++;

        if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                 (obj->flagEnableAlignment == false))
        {
            obj->motorState = MOTOR_OL_START;
            obj->stateRunTimeCnt = 0;
        }
    }
#endif  // DMC_LEVEL_2

#if (DMC_BUILDLEVEL == DMC_LEVEL_3)
    if(obj->motorState >= MOTOR_CL_RUNNING)
    {
        obj->bldcHandle->commTrigFlag = ISBLDC_getCommTrigFlag(obj->isbldcHandle);
    }
    else if(obj->motorState == MOTOR_OL_START)
    {
        if(RIMPULSE_getRmpDoneFlag(obj->rimpulseHandle) == true)
        {
            obj->bldcHandle->commTrigFlag = ISBLDC_getCommTrigFlag(obj->isbldcHandle);

            obj->stateRunTimeCnt++;

            if(obj->stateRunTimeCnt > obj->forceRunTimeDelay)
            {
                obj->motorState = MOTOR_CL_RUNNING;
                obj->stateRunTimeCnt = 0;

                obj->enableSpeedCtrl = obj->flagEnableSpeedCtrl;
                obj->speed_int_Hz = obj->speedINT_Hz;
                TRAJ_setIntValue(obj->trajHandle_spd, obj->speedINT_Hz);
                PI_setUi(obj->piHandle_spd, obj->bldcHandle->pwmDuty);
            }
        }
        else
        {
            RIMPULSE_run(obj->rimpulseHandle);

            obj->bldcHandle->commTrigFlag = RIMPULSE_getTrigFlag(obj->rimpulseHandle);
            obj->bldcHandle->pwmDuty = obj->bldcHandle->pwmDutyStart;

            obj->enableSpeedCtrl = false;
        }
    }
    else if(obj->motorState == MOTOR_ALIGNMENT)
    {
        obj->bldcHandle->commTrigFlag = false;
        obj->enableSpeedCtrl = false;

        MOD6CNT_setCounter(obj->mod6cntHandle, 0);

        obj->bldcHandle->pwmDuty = obj->bldcHandle->pwmDutyStart;

        obj->stateRunTimeCnt++;

        if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                 (obj->flagEnableAlignment == false))
        {
            obj->motorState = MOTOR_OL_START;
            obj->stateRunTimeCnt = 0;
        }
    }

    // run the speed controller
    if(obj->enableSpeedCtrl == true)
    {
        obj->counterSpeed++;

        if(obj->counterSpeed >= objUser->numCtrlTicksPerSpeedTick)
        {
            obj->counterSpeed = 0;

            PI_run(obj->piHandle_spd, obj->speed_int_Hz,
                   obj->speed_Hz, (float32_t*)&obj->bldcHandle->pwmDuty);
        }
    }
#endif  // DMC_LEVEL_3

#if (DMC_BUILDLEVEL == DMC_LEVEL_4)
    if(obj->motorState >= MOTOR_CL_RUNNING)
    {
        obj->bldcHandle->commTrigFlag = ISBLDC_getCommTrigFlag(obj->isbldcHandle);
    }
    else if(obj->motorState == MOTOR_OL_START)
    {
        if(RIMPULSE_getRmpDoneFlag(obj->rimpulseHandle) == true)
        {
            obj->bldcHandle->commTrigFlag = ISBLDC_getCommTrigFlag(obj->isbldcHandle);
            obj->bldcHandle->IdcRef = obj->bldcHandle->IdcStart;

            obj->stateRunTimeCnt++;

            if(obj->stateRunTimeCnt > obj->forceRunTimeDelay)
            {
                obj->stateRunTimeCnt = 0;

                obj->motorState = MOTOR_CL_RUNNING;
                obj->enableSpeedCtrl = obj->flagEnableSpeedCtrl;
                obj->speed_int_Hz = obj->speedINT_Hz;

                TRAJ_setIntValue(obj->trajHandle_spd, obj->speedINT_Hz);
                PI_setUi(obj->piHandle_spd, obj->bldcHandle->IdcRef);
            }
        }
        else
        {
            RIMPULSE_run(obj->rimpulseHandle);

            obj->bldcHandle->commTrigFlag = RIMPULSE_getTrigFlag(obj->rimpulseHandle);
            obj->bldcHandle->IdcRef = obj->bldcHandle->IdcStart;

            obj->enableSpeedCtrl = false;
            obj->stateRunTimeCnt = 0;
        }
    }
    else if(obj->motorState == MOTOR_ALIGNMENT)
    {
        obj->bldcHandle->commTrigFlag = false;
        obj->enableSpeedCtrl = false;

        MOD6CNT_setCounter(obj->mod6cntHandle, 0);

        obj->bldcHandle->IdcRef = obj->bldcHandle->IdcStart;
        obj->enableCurrentCtrl = obj->flagEnableCurrentCtrl;

        obj->stateRunTimeCnt++;

        if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                 (obj->flagEnableAlignment == false))
        {
            obj->motorState = MOTOR_OL_START;
            obj->stateRunTimeCnt = 0;
        }
    }

    // run the speed controller
    if(obj->enableSpeedCtrl == true)
    {
        obj->counterSpeed++;

        if(obj->counterSpeed >= objUser->numCtrlTicksPerSpeedTick)
        {
            obj->counterSpeed = 0;

            PI_run(obj->piHandle_spd, fabsf(obj->speed_int_Hz),
                   fabsf(obj->speed_Hz), (float32_t*)&obj->bldcHandle->IdcRef);
        }
    }

    if(obj->enableCurrentCtrl == true)
    {
        PI_run(obj->piHandle_Iq, obj->bldcHandle->IdcRef,
               obj->bldcHandle->IdcInFilter, &obj->bldcHandle->pwmDuty);
    }
#endif  // DMC_LEVEL_4

    if(HAL_getPwmEnableStatus(obj->halMtrHandle) == true)
    {
        HAL_writePWMDataBLDC(obj->halMtrHandle,
                         obj->bldcHandle->pwmDuty, obj->bldcHandle->commState);
    }
    // defined(MOTOR1_ISBLDC)
#else  // !MOTOR1_ISBLDC
    //------------------------------------------------------------------------------

#if defined(MOTOR1_DCLINKSS)
    // run single-shunt current reconstruction
    DCLINK_SS_runCurrentReconstruction(obj->dclinkHandle,
                                     &obj->adcData.Idc1_A, &obj->adcData.Idc2_A);

    obj->adcData.I_A.value[0] = DCLINK_SS_getIa(obj->dclinkHandle);
    obj->adcData.I_A.value[1] = DCLINK_SS_getIb(obj->dclinkHandle);
    obj->adcData.I_A.value[2] = DCLINK_SS_getIc(obj->dclinkHandle);
#else // !(MOTOR1_DCLINKSS)
#if defined(MOTOR1_OVM)
    // Over Modulation Supporting, run the current reconstruction algorithm
    SVGENCURRENT_RunRegenCurrent(obj->svgencurrentHandle,
                                 &obj->adcData.I_A, &obj->adcDataPrev);
#endif  // MOTOR1_OVM
#endif // !(MOTOR1_DCLINKSS)

#if defined(MOTOR1_FAST) && defined(MOTOR1_ESMO)    // (OK<->OK)
    // sensorless-FOC
    MATH_Vec2 phasor;

    ANGLE_GEN_run(obj->angleGenHandle, obj->speed_int_Hz);
    obj->angleGen_rad = ANGLE_GEN_getAngle(obj->angleGenHandle);

#if defined(MOTOR1_VOLRECT)
    VOLREC_run(obj->volrecHandle, obj->adcData.VdcBus_V,
               &(obj->pwmData.Vabc_pu), &(obj->estInputData.Vab_V));
#else  // !MOTOR1_VOLRECT
    // remove offsets
    obj->adcData.V_V.value[0] -=
            obj->adcData.offset_V_sf.value[0] * obj->adcData.VdcBus_V;

    obj->adcData.V_V.value[1] -=
            obj->adcData.offset_V_sf.value[1] * obj->adcData.VdcBus_V;

    obj->adcData.V_V.value[2] -=
            obj->adcData.offset_V_sf.value[2] * obj->adcData.VdcBus_V;

    // run Clarke transform on voltage
    CLARKE_run(obj->clarkeHandle_V,
               &obj->adcData.V_V, &obj->estInputData.Vab_V);
#endif  // MOTOR1_VOLRECT

    // run Clarke transform on current
    CLARKE_run(obj->clarkeHandle_I, &obj->adcData.I_A, &obj->estInputData.Iab_A);

    if(((EST_isMotorIdentified(obj->estHandle) == false) ||
            (EST_getState(obj->estHandle) == EST_STATE_RS)) &&
            (EST_isEnabled(obj->estHandle) == true))
    {   // run identification or Rs Recalibration
        // setup the trajectory generator
        EST_setupTrajState(obj->estHandle,
                           obj->Idq_out_A.value[1],
                           obj->speedRef_Hz,
                           0.0f);

        // run the trajectories
        EST_runTraj(obj->estHandle);

        obj->IdRated_A = EST_getIntValue_Id_A(obj->estHandle);

        // store the input data into a buffer
        obj->estInputData.speed_ref_Hz = EST_getIntValue_spd_Hz(obj->estHandle);

        obj->speed_int_Hz = obj->estInputData.speed_ref_Hz;

        obj->enableSpeedCtrl = EST_doSpeedCtrl(obj->estHandle);
        obj->enableCurrentCtrl = EST_doCurrentCtrl(obj->estHandle);

        obj->motorState = MOTOR_CTRL_RUN;
    }
    else if(obj->flagMotorIdentified == true)
    {
        if(obj->flagRunIdentAndOnLine == true)
        {
            obj->counterTrajSpeed++;

            if(obj->counterTrajSpeed >= objUser->numIsrTicksPerTrajTick)
            {
                // clear counter
                obj->counterTrajSpeed = 0;

                // run a trajectory for speed reference,
                // so the reference changes with a ramp instead of a step
                TRAJ_run(obj->trajHandle_spd);
            }

            obj->enableCurrentCtrl = obj->flagEnableCurrentCtrl;
            obj->enableSpeedCtrl = obj->flagEnableSpeedCtrl;


            // get Id reference for Rs OnLine
            obj->IdRated_A = EST_getIdRated_A(obj->estHandle);
        }
        else
        {
            obj->enableSpeedCtrl = false;
            obj->enableCurrentCtrl = false;
        }

        obj->estInputData.speed_ref_Hz = TRAJ_getIntValue(obj->trajHandle_spd);
        obj->speed_int_Hz = obj->estInputData.speed_ref_Hz;
    }

    // store the input data into a buffer
    obj->estInputData.dcBus_V = obj->adcData.VdcBus_V;


    // run the FAST estimator
    EST_run(obj->estHandle, &obj->estInputData, &obj->estOutputData);

    // compute angle with delay compensation
    obj->angleDelta_rad =
            objUser->angleDelayed_sf_sec * obj->estOutputData.fm_lp_rps;

    obj->angleEST_rad =
            MATH_incrAngle(obj->estOutputData.angle_rad, obj->angleDelta_rad);

    obj->speedEST_Hz = EST_getFm_lp_Hz(obj->estHandle);


    obj->oneOverDcBus_invV = obj->estOutputData.oneOverDcBus_invV;


    // run the eSMO
    ESMO_setSpeedRef(obj->esmoHandle, obj->speed_int_Hz);
    ESMO_run(obj->esmoHandle, obj->adcData.VdcBus_V,
                    &(obj->pwmData.Vabc_pu), &(obj->estInputData.Iab_A));

    obj->angleComp_rad = obj->speedPLL_Hz * obj->angleDelayed_sf;
    obj->anglePLL_rad = MATH_incrAngle(ESMO_getAnglePLL(obj->esmoHandle), obj->angleComp_rad);

    SPDFR_run(obj->spdfrHandle, obj->anglePLL_rad);
    obj->speedPLL_Hz = SPDFR_getSpeedHz(obj->spdfrHandle);


    obj->speedFilter_Hz = obj->speedFilter_Hz *0.875f + obj->speed_Hz * 0.125f;
    obj->speedAbs_Hz = fabsf(obj->speedFilter_Hz);

    // Running state
    if(obj->estimatorMode == ESTIMATOR_MODE_FAST)
    {
        obj->speed_Hz = obj->speedEST_Hz;

        if(obj->motorState >= MOTOR_CTRL_RUN)
        {
            obj->angleFOC_rad = obj->angleEST_rad;

            ESMO_updateKslide(obj->esmoHandle);
        }
        else if(obj->motorState == MOTOR_CL_RUNNING)
        {
            obj->angleFOC_rad = obj->angleEST_rad;
            ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);
        }
        else if(obj->motorState == MOTOR_OL_START)
        {
            obj->angleFOC_rad = obj->angleEST_rad;
            obj->motorState = MOTOR_CL_RUNNING;

            ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);
        }
        else if(obj->motorState == MOTOR_ALIGNMENT)
        {
            obj->angleFOC_rad = 0.0f;
            obj->enableSpeedCtrl = false;

            obj->stateRunTimeCnt++;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = obj->alignCurrent_A;
            obj->Idq_out_A.value[1] = 0.0f;

            TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
            EST_setAngle_rad(obj->estHandle, obj->angleFOC_rad);
            ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);

            if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                     (obj->flagEnableAlignment == false))
            {
                obj->stateRunTimeCnt = 0;
                obj->motorState = MOTOR_OL_START;

                obj->Idq_out_A.value[0] = obj->fluxCurrent_A;
                PI_setUi(obj->piHandle_spd, 0.0);
            }
        }
        else if(obj->motorState == MOTOR_SEEK_POS)
        {
            obj->enableSpeedCtrl = false;

            obj->stateRunTimeCnt++;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = 0.0f;
            obj->Idq_out_A.value[1] = 0.0f;

            if(obj->stateRunTimeCnt > obj->flyingStartTimeDelay)
            {
                obj->stateRunTimeCnt = 0;

                if(obj->speedAbs_Hz > obj->speedFlyingStart_Hz)
                {
                    obj->speed_int_Hz = obj->speedFilter_Hz;
                    TRAJ_setIntValue(obj->trajHandle_spd, obj->speedFilter_Hz);
                    PI_setUi(obj->piHandle_spd, 0.0f);

                    obj->motorState = MOTOR_CL_RUNNING;

                    ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);
                }
                else
                {
                    obj->angleFOC_rad = 0.0f;
                    obj->motorState = MOTOR_ALIGNMENT;
                }
            }
        }
    }
    else    // if(obj->estimatorMode == ESTIMATOR_MODE_ESMO)
    {
        obj->speed_Hz = obj->speedPLL_Hz;

        if(obj->motorState >= MOTOR_CL_RUNNING)
        {
            obj->angleFOC_rad = obj->anglePLL_rad;

            ESMO_updateKslide(obj->esmoHandle);
        }
        else if(obj->motorState == MOTOR_OL_START)
        {
            obj->angleFOC_rad = obj->angleGen_rad;
            obj->enableSpeedCtrl = false;

            obj->Idq_out_A.value[0] = 0.0f;

            if(obj->speed_int_Hz > 0.0f)
            {
                obj->IsRef_A = obj->startCurrent_A;
                obj->Idq_out_A.value[1] = obj->startCurrent_A;
            }
            else
            {
                obj->IsRef_A = -obj->startCurrent_A;
                obj->Idq_out_A.value[1] = -obj->startCurrent_A;
            }

            if(fabsf(obj->estInputData.speed_ref_Hz) >= obj->speedForce_Hz)
            {
                obj->stateRunTimeCnt++;

                TRAJ_setIntValue(obj->trajHandle_spd, obj->estInputData.speed_ref_Hz);

                if(obj->stateRunTimeCnt > obj->forceRunTimeDelay)
                {
                    obj->motorState = MOTOR_CL_RUNNING;

                    EST_setAngle_rad(obj->estHandle, obj->angleFOC_rad);
                    ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);

                    PI_setUi(obj->piHandle_spd, (obj->frswPos_sf * obj->Idq_out_A.value[1]));
                }
            }
        }
        else if(obj->motorState == MOTOR_ALIGNMENT)
        {
            obj->angleFOC_rad = 0.0f;
            obj->enableSpeedCtrl = false;

            obj->stateRunTimeCnt++;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = obj->alignCurrent_A;
            obj->Idq_out_A.value[1] = 0.0f;

            TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
            EST_setAngle_rad(obj->estHandle, obj->angleFOC_rad);
            ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);
            ANGLE_GEN_setAngle(obj->angleGenHandle, obj->angleFOC_rad);

            if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                     (obj->flagEnableAlignment == false))
            {
                obj->stateRunTimeCnt = 0;
                obj->motorState = MOTOR_OL_START;

                obj->Idq_out_A.value[0] = obj->fluxCurrent_A;

                PI_setUi(obj->piHandle_spd, 0.0f);
            }
        }
        else if(obj->motorState == MOTOR_SEEK_POS)
        {
            obj->enableSpeedCtrl = false;

            obj->stateRunTimeCnt++;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = 0.0f;
            obj->Idq_out_A.value[1] = 0.0f;

            if(obj->stateRunTimeCnt > obj->flyingStartTimeDelay)
            {
                obj->stateRunTimeCnt = 0;

                if(obj->speedAbs_Hz > obj->speedFlyingStart_Hz)
                {
                    obj->speed_int_Hz = obj->speedFilter_Hz;
                    TRAJ_setIntValue(obj->trajHandle_spd, obj->speedFilter_Hz);
                    PI_setUi(obj->piHandle_spd, 0.0f);

                    obj->motorState = MOTOR_CL_RUNNING;
                }
                else
                {
                    obj->angleFOC_rad = 0.0f;
                    obj->motorState = MOTOR_ALIGNMENT;
                }
            }
        }
    }

#ifdef MOTOR1_VOLRECT
    if(obj->motorState == MOTOR_CL_RUNNING)
    {
        obj->angleFOC_rad = obj->angleEST_rad;
    }
    else if(obj->motorState == MOTOR_OL_START)
    {
        obj->angleFOC_rad = obj->angleGen_rad;
        obj->enableSpeedCtrl = false;

        Idq_out_A.value[0] = 0.0f;

        if(obj->speed_int_Hz > 0.0f)
        {
            obj->IsRef_A = obj->startCurrent_A;
            obj->Idq_out_A.value[1] = obj->startCurrent_A;
        }
        else
        {
            obj->IsRef_A = -obj->startCurrent_A;
            obj->Idq_out_A.value[1] = -obj->startCurrent_A;
        }

        if(fabsf(estInputData.speed_ref_Hz) >= obj->speedForce_Hz)
        {
            obj->motorState = MOTOR_CL_RUNNING;

            EST_setAngle_rad(obj->estHandle, angleFOC_rad);

            PI_setUi(obj->piHandle_spd, obj->startCurrent_A * 0.5f);
        }
    }
#endif  // MOTOR1_VOLRECT

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    obj->angleFOC_rad = obj->angleGen_rad;
#endif

    // compute the sin/cos phasor
    phasor.value[0] = __cos(obj->angleFOC_rad);
    phasor.value[1] = __sin(obj->angleFOC_rad);

    // set the phasor in the Park transform
    PARK_setPhasor(obj->parkHandle_I, &phasor);

    // run the Park transform
    PARK_run(obj->parkHandle_I, &(obj->estInputData.Iab_A),
             (MATH_vec2 *)&(obj->Idq_in_A));

    // End of MOTOR1_FAST && MOTOR1_ESMO
//------------------------------------------------------------------------------
#elif defined(MOTOR1_FAST) && defined(MOTOR1_ENC)
    // sensorless-FOC
    MATH_Vec2 phasor;

    ANGLE_GEN_run(obj->angleGenHandle, obj->speed_int_Hz);
    obj->angleGen_rad = ANGLE_GEN_getAngle(obj->angleGenHandle);

    // remove offsets
    obj->adcData.V_V.value[0] -=
            obj->adcData.offset_V_sf.value[0] * obj->adcData.VdcBus_V;

    obj->adcData.V_V.value[1] -=
            obj->adcData.offset_V_sf.value[1] * obj->adcData.VdcBus_V;

    obj->adcData.V_V.value[2] -=
            obj->adcData.offset_V_sf.value[2] * obj->adcData.VdcBus_V;

    // run Clarke transform on voltage
    CLARKE_run(obj->clarkeHandle_V,
               &obj->adcData.V_V, &obj->estInputData.Vab_V);

    // run Clarke transform on current
    CLARKE_run(obj->clarkeHandle_I, &obj->adcData.I_A, &obj->estInputData.Iab_A);

    if(obj->flagMotorIdentified == true)
    {
        if(obj->flagRunIdentAndOnLine == true)
        {
            obj->counterTrajSpeed++;

            if(obj->counterTrajSpeed >= objUser->numIsrTicksPerTrajTick)
            {
                // clear counter
                obj->counterTrajSpeed = 0;

                // run a trajectory for speed reference,
                // so the reference changes with a ramp instead of a step
                TRAJ_run(obj->trajHandle_spd);
            }

            obj->enableCurrentCtrl = obj->flagEnableCurrentCtrl;
            obj->enableSpeedCtrl = obj->flagEnableSpeedCtrl;

            // get Id reference for Rs OnLine
            obj->IdRated_A = EST_getIdRated_A(obj->estHandle);
        }
        else
        {
            obj->enableSpeedCtrl = false;
            obj->enableCurrentCtrl = false;
        }

        obj->estInputData.speed_ref_Hz = TRAJ_getIntValue(obj->trajHandle_spd);
        obj->speed_int_Hz = obj->estInputData.speed_ref_Hz;

    }
    else if(EST_isEnabled(obj->estHandle) == true)  // run identification
    {
        // setup the trajectory generator
        EST_setupTrajState(obj->estHandle,
                           obj->Idq_out_A.value[1],
                           obj->speedRef_Hz,
                           0.0f);

        // run the trajectories
        EST_runTraj(obj->estHandle);

        obj->IdRated_A = EST_getIntValue_Id_A(obj->estHandle);

        // store the input data into a buffer
        obj->estInputData.speed_ref_Hz = EST_getIntValue_spd_Hz(obj->estHandle);

        obj->speed_int_Hz = obj->estInputData.speed_ref_Hz;

        obj->enableSpeedCtrl = EST_doSpeedCtrl(obj->estHandle);
        obj->enableCurrentCtrl = EST_doCurrentCtrl(obj->estHandle);

        obj->motorState = MOTOR_CTRL_RUN;
    }

    // store the input data into a buffer
    obj->estInputData.dcBus_V = obj->adcData.VdcBus_V;


    // Runs the FAST estimator
    EST_run(obj->estHandle, &obj->estInputData, &obj->estOutputData);

    // compute angle with delay compensation
    obj->angleDelta_rad =
            objUser->angleDelayed_sf_sec * obj->estOutputData.fm_lp_rps;

    obj->angleEST_rad =
            MATH_incrAngle(obj->estOutputData.angle_rad, obj->angleDelta_rad);

    obj->speedEST_Hz = EST_getFm_lp_Hz(obj->estHandle);


    obj->oneOverDcBus_invV = obj->estOutputData.oneOverDcBus_invV;


    // Runs encoder
    ENC_run(obj->encHandle);
    obj->angleENC_rad = ENC_getElecAngle(obj->encHandle);

    SPDCALC_run(obj->spdcalcHandle, obj->angleENC_rad);
    obj->speedENC_Hz = SPDCALC_getSpeedHz(obj->spdcalcHandle);


    obj->speedFilter_Hz = obj->speedFilter_Hz *0.875f + obj->speed_Hz * 0.125f;
    obj->speedAbs_Hz = fabsf(obj->speedFilter_Hz);

    // Running state
    if(obj->estimatorMode == ESTIMATOR_MODE_FAST)
    {
        obj->speed_Hz = obj->speedEST_Hz;

        if(obj->motorState >= MOTOR_CL_RUNNING)
        {
            obj->angleFOC_rad = obj->angleEST_rad;
        }
        else if(obj->motorState == MOTOR_OL_START)
        {
            obj->angleFOC_rad = obj->angleEST_rad;

            obj->stateRunTimeCnt++;

            if((ENC_getState(obj->encHandle) == ENC_CALIBRATION_DONE) ||
                    ((obj->stateRunTimeCnt > obj->forceRunTimeDelay)))
            {
                obj->stateRunTimeCnt = 0;
                obj->motorState = MOTOR_CL_RUNNING;
            }
        }
        else if(obj->motorState == MOTOR_ALIGNMENT)
        {
            obj->angleFOC_rad = 0.0f;
            obj->enableSpeedCtrl = false;

            obj->stateRunTimeCnt++;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = obj->alignCurrent_A;
            obj->Idq_out_A.value[1] = 0.0f;

            TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
            EST_setAngle_rad(obj->estHandle, obj->angleFOC_rad);

            if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                    (obj->flagEnableAlignment == false))
            {
                obj->stateRunTimeCnt = 0;
                obj->motorState = MOTOR_OL_START;

                obj->Idq_out_A.value[0] = obj->fluxCurrent_A;

                ENC_setState(obj->encHandle, ENC_WAIT_FOR_INDEX);

                PI_setUi(obj->piHandle_spd, 0.0);
            }
        }
        else if(obj->motorState == MOTOR_SEEK_POS)
        {
            obj->enableSpeedCtrl = false;

            obj->stateRunTimeCnt++;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = 0.0f;
            obj->Idq_out_A.value[1] = 0.0f;

            if(obj->stateRunTimeCnt > obj->flyingStartTimeDelay)
            {
                obj->stateRunTimeCnt = 0;

                if(obj->speedAbs_Hz > obj->speedFlyingStart_Hz)
                {
                    obj->speed_int_Hz = obj->speedFilter_Hz;
                    TRAJ_setIntValue(obj->trajHandle_spd, obj->speedFilter_Hz);
                    PI_setUi(obj->piHandle_spd, 0.0f);

                    obj->motorState = MOTOR_CL_RUNNING;
                }
                else
                {
                    obj->angleFOC_rad = 0.0f;
                    obj->motorState = MOTOR_ALIGNMENT;
                }
            }
        }
    }
    else    // if(obj->estimatorMode == ESTIMATOR_MODE_ENC)
    {
        obj->speed_Hz = obj->speedENC_Hz;

        if(obj->motorState >= MOTOR_CL_RUNNING)
        {
            obj->angleFOC_rad = obj->angleENC_rad;
        }
        else if(obj->motorState == MOTOR_OL_START)
        {
            obj->angleFOC_rad = obj->angleGen_rad;
            obj->enableSpeedCtrl = false;

            obj->Idq_out_A.value[0] = 0.0f;

            if(obj->speed_int_Hz > 0.0f)
            {
                obj->IsRef_A = obj->startCurrent_A;
                obj->Idq_out_A.value[1] = obj->startCurrent_A;
            }
            else
            {
                obj->IsRef_A = -obj->startCurrent_A;
                obj->Idq_out_A.value[1] = -obj->startCurrent_A;
            }

            TRAJ_setIntValue(obj->trajHandle_spd, obj->speed_int_Hz);
            EST_setAngle_rad(obj->estHandle, obj->angleFOC_rad);

            if(ENC_getState(obj->encHandle) == ENC_CALIBRATION_DONE)
            {
                obj->motorState = MOTOR_CL_RUNNING;
            }
        }
        else if(obj->motorState == MOTOR_ALIGNMENT)
        {
            obj->angleFOC_rad = 0.0f;
            obj->enableSpeedCtrl = false;

            obj->stateRunTimeCnt++;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = obj->alignCurrent_A;
            obj->Idq_out_A.value[1] = 0.0f;

            TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
            EST_setAngle_rad(obj->estHandle, obj->angleFOC_rad);
            ANGLE_GEN_setAngle(obj->angleGenHandle, obj->angleFOC_rad);

            if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                     (obj->flagEnableAlignment == false))
            {
                obj->stateRunTimeCnt = 0;
                obj->motorState = MOTOR_OL_START;

                obj->Idq_out_A.value[0] = obj->fluxCurrent_A;

                ENC_setState(obj->encHandle, ENC_WAIT_FOR_INDEX);
                PI_setUi(obj->piHandle_spd, 0.0);
            }
        }
        else if(obj->motorState == MOTOR_SEEK_POS)
        {
            obj->enableSpeedCtrl = false;

            obj->stateRunTimeCnt++;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = 0.0f;
            obj->Idq_out_A.value[1] = 0.0f;

            if(obj->stateRunTimeCnt > obj->flyingStartTimeDelay)
            {
                obj->stateRunTimeCnt = 0;

                if(obj->speedAbs_Hz > obj->speedFlyingStart_Hz)
                {
                    obj->speed_int_Hz = obj->speedFilter_Hz;
                    TRAJ_setIntValue(obj->trajHandle_spd, obj->speedFilter_Hz);
                    PI_setUi(obj->piHandle_spd, 0.0f);

                    obj->motorState = MOTOR_CL_RUNNING;
                }
                else
                {
                    obj->angleFOC_rad = 0.0f;
                    obj->motorState = MOTOR_ALIGNMENT;
                }
            }
        }
    }

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    obj->angleFOC_rad = obj->angleGen_rad;
#endif

#ifdef BRAKE_ENABLE
    if(obj->flagEnableBraking == true)
    {
        if(obj->motorState != MOTOR_BRAKE_STOP)
        {
            obj->motorState = MOTOR_BRAKE_STOP;

            if(obj->brakingMode == HARDSWITCH_BRAKE_MODE)
            {
                // enable the braking mode PWM with
                // turning-on three low side, turn off three high side
                HAL_enableBrakePWM(obj->halMtrHandle);
            }
            else if(obj->brakingMode == FORCESTOP_BRAKE_MODE)
            {
                obj->angleBrake_rad = obj->angleFOC_rad;
                PI_setRefValue(obj->piHandle_spd, 0.0f);
                PI_setUi(obj->piHandle_spd, 0.0f);
            }
        }

        if(obj->brakingMode == FORCESTOP_BRAKE_MODE)
        {
            // compute the sin/cos phasor
            obj->angleBrake_rad = obj->angleBrake_rad;

            obj->IsRef_A = obj->brakingCurrent_A;
            obj->Idq_out_A.value[1] = obj->brakingCurrent_A;

            obj->enableSpeedCtrl = false;
            obj->enableCurrentCtrl = true;
        }
        else
        {
            obj->enableSpeedCtrl = false;
            obj->enableCurrentCtrl = false;
        }
    }
#endif  // BRAKE_ENABLE

    // compute the sin/cos phasor
    phasor.value[0] = __cos(obj->angleFOC_rad);
    phasor.value[1] = __sin(obj->angleFOC_rad);

    // set the phasor in the Park transform
    PARK_setPhasor(obj->parkHandle_I, &phasor);

    // run the Park transform
    PARK_run(obj->parkHandle_I, &(obj->estInputData.Iab_A),
             (MATH_vec2 *)&(obj->Idq_in_A));

// End of MOTOR1_FAST && MOTOR1_ENC

//------------------------------------------------------------------------------
#elif defined(MOTOR1_ESMO) && defined(MOTOR1_ENC)
    // sensorless-FOC
    MATH_Vec2 phasor;

    ANGLE_GEN_run(obj->angleGenHandle, obj->speed_int_Hz);
    obj->angleGen_rad = ANGLE_GEN_getAngle(obj->angleGenHandle);

    // run Clarke transform on current
    CLARKE_run(obj->clarkeHandle_I, &obj->adcData.I_A, &obj->Iab_A);

    if(obj->flagRunIdentAndOnLine == true)
    {
        obj->counterTrajSpeed++;

        if(obj->counterTrajSpeed >= objUser->numIsrTicksPerTrajTick)
        {
            // clear counter
            obj->counterTrajSpeed = 0;

            // run a trajectory for speed reference,
            // so the reference changes with a ramp instead of a step
            TRAJ_run(obj->trajHandle_spd);
        }

        obj->enableCurrentCtrl = obj->flagEnableCurrentCtrl;
        obj->enableSpeedCtrl = obj->flagEnableSpeedCtrl;
    }
    else
    {
        obj->enableSpeedCtrl = false;
        obj->enableCurrentCtrl = false;
    }

    obj->speed_int_Hz = TRAJ_getIntValue(obj->trajHandle_spd);
    obj->oneOverDcBus_invV = 1.0f / obj->adcData.VdcBus_V;

    // run the eSMO
    ESMO_setSpeedRef(obj->esmoHandle, obj->speed_int_Hz);
    ESMO_run(obj->esmoHandle, obj->adcData.VdcBus_V,
                    &(obj->pwmData.Vabc_pu), &(obj->Iab_A));

    obj->angleComp_rad = obj->speedPLL_Hz * obj->angleDelayed_sf;
    obj->anglePLL_rad = MATH_incrAngle(ESMO_getAnglePLL(obj->esmoHandle), obj->angleComp_rad);

    SPDFR_run(obj->spdfrHandle, obj->anglePLL_rad);
    obj->speedPLL_Hz = SPDFR_getSpeedHz(obj->spdfrHandle);

    // run the encoder
    ENC_inline_run(obj->encHandle);
    obj->angleENC_rad = ENC_getElecAngle(obj->encHandle);

    SPDCALC_run(obj->spdcalcHandle, obj->angleENC_rad);
    obj->speedENC_Hz = SPDCALC_getSpeedHz(obj->spdcalcHandle);

    obj->speedFilter_Hz = obj->speedFilter_Hz *0.875f + obj->speed_Hz * 0.125f;
    obj->speedAbs_Hz = fabsf(obj->speedFilter_Hz);

    // Running state
    if(obj->estimatorMode == ESTIMATOR_MODE_ESMO)
    {
        obj->speed_Hz = obj->speedPLL_Hz;

        if(obj->motorState >= MOTOR_CL_RUNNING)
        {
            obj->angleFOC_rad = obj->anglePLL_rad;

            ESMO_updateKslide(obj->esmoHandle);
        }
        else if(obj->motorState == MOTOR_OL_START)
        {
            obj->angleFOC_rad = obj->angleGen_rad;
            obj->enableSpeedCtrl = false;

            obj->Idq_out_A.value[0] = 0.0f;

            if(obj->speed_int_Hz > 0.0f)
            {
                obj->IsRef_A = obj->startCurrent_A;
                obj->Idq_out_A.value[1] = obj->startCurrent_A;
            }
            else
            {
                obj->IsRef_A = -obj->startCurrent_A;
                obj->Idq_out_A.value[1] = -obj->startCurrent_A;
            }

            if(fabsf(obj->speed_int_Hz) >= obj->speedForce_Hz)
            {
                obj->stateRunTimeCnt++;

                TRAJ_setIntValue(obj->trajHandle_spd, obj->speed_int_Hz);

                if(obj->stateRunTimeCnt > obj->forceRunTimeDelay)
                {
                    obj->motorState = MOTOR_CL_RUNNING;
                    obj->stateRunTimeCnt = 0;

                    ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);

                    PI_setUi(obj->piHandle_spd, (obj->frswPos_sf * obj->Idq_out_A.value[1]));
                }
            }
        }
        else if(obj->motorState == MOTOR_ALIGNMENT)
        {
            obj->angleFOC_rad = 0.0f;
            obj->enableSpeedCtrl = false;

            obj->stateRunTimeCnt++;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = obj->alignCurrent_A;
            obj->Idq_out_A.value[1] = 0.0f;

            TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
            ESMO_setAnglePu(obj->esmoHandle, 0.0f);
            ANGLE_GEN_setAngle(obj->angleGenHandle, 0.0f);

            if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                     (obj->flagEnableAlignment == false))
            {
                obj->motorState = MOTOR_OL_START;
                obj->stateRunTimeCnt = 0;

                obj->Idq_out_A.value[0] = obj->fluxCurrent_A;

                ENC_setState(obj->encHandle, ENC_WAIT_FOR_INDEX);

                PI_setUi(obj->piHandle_spd, 0.0f);
            }
        }
        else if(obj->motorState == MOTOR_SEEK_POS)
        {
            obj->enableSpeedCtrl = false;

            obj->stateRunTimeCnt++;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = 0.0f;
            obj->Idq_out_A.value[1] = 0.0f;

            if(obj->stateRunTimeCnt > obj->flyingStartTimeDelay)
            {
                obj->stateRunTimeCnt = 0;

                if(obj->speedAbs_Hz > obj->speedFlyingStart_Hz)
                {
                    obj->speed_int_Hz = obj->speedFilter_Hz;
                    TRAJ_setIntValue(obj->trajHandle_spd, obj->speedFilter_Hz);
                    PI_setUi(obj->piHandle_spd, 0.0f);

                    obj->motorState = MOTOR_CL_RUNNING;
                }
                else
                {
                    obj->angleFOC_rad = 0.0f;
                    obj->motorState = MOTOR_ALIGNMENT;
                }
            }
        }
    }
    else    // if(obj->estimatorMode == ESTIMATOR_MODE_ENC)
    {
        obj->speed_Hz = obj->speedENC_Hz;

        if(obj->motorState >= MOTOR_CTRL_RUN)
        {
            obj->angleFOC_rad = obj->angleENC_rad;

            ESMO_updateKslide(obj->esmoHandle);
        }
        else if(obj->motorState == MOTOR_CL_RUNNING)
        {
            obj->angleFOC_rad = obj->angleENC_rad;
            ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);
        }
        else if(obj->motorState == MOTOR_OL_START)
        {
            obj->angleFOC_rad = obj->angleGen_rad;
            obj->enableSpeedCtrl = false;

            obj->Idq_out_A.value[0] = 0.0f;

            if(obj->speed_int_Hz > 0.0f)
            {
                obj->IsRef_A = obj->startCurrent_A;
                obj->Idq_out_A.value[1] = obj->startCurrent_A;
            }
            else
            {
                obj->IsRef_A = -obj->startCurrent_A;
                obj->Idq_out_A.value[1] = -obj->startCurrent_A;
            }

            if(ENC_getState(obj->encHandle) == ENC_CALIBRATION_DONE)
            {
                obj->motorState = MOTOR_CL_RUNNING;

                ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);
            }
        }
        else if(obj->motorState == MOTOR_ALIGNMENT)
        {
            obj->angleFOC_rad = 0.0f;
            obj->enableSpeedCtrl = false;

            obj->stateRunTimeCnt++;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = obj->alignCurrent_A;
            obj->Idq_out_A.value[1] = 0.0f;

            TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
            ANGLE_GEN_setAngle(obj->angleGenHandle, 0.0f);

            ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);

            if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                     (obj->flagEnableAlignment == false))
            {
                obj->stateRunTimeCnt = 0;
                obj->motorState = MOTOR_OL_START;

                obj->Idq_out_A.value[0] = obj->fluxCurrent_A;

                ENC_setState(obj->encHandle, ENC_WAIT_FOR_INDEX);
                PI_setUi(obj->piHandle_spd, 0.0);
            }
        }
        else if(obj->motorState == MOTOR_SEEK_POS)
        {
            obj->enableSpeedCtrl = false;

            obj->stateRunTimeCnt++;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = 0.0f;
            obj->Idq_out_A.value[1] = 0.0f;

            if(obj->stateRunTimeCnt > obj->flyingStartTimeDelay)
            {
                obj->stateRunTimeCnt = 0;

                if(obj->speedAbs_Hz > obj->speedFlyingStart_Hz)
                {
                    obj->speed_int_Hz = obj->speedFilter_Hz;
                    TRAJ_setIntValue(obj->trajHandle_spd, obj->speedFilter_Hz);
                    PI_setUi(obj->piHandle_spd, 0.0f);

                    obj->motorState = MOTOR_CL_RUNNING;

                    ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);
                }
                else
                {
                    obj->angleFOC_rad = 0.0f;
                    obj->motorState = MOTOR_ALIGNMENT;
                }
            }
        }
    }

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    obj->angleFOC_rad = obj->angleGen_rad;
#endif

    // compute the sin/cos phasor
    phasor.value[0] = __cos(obj->angleFOC_rad);
    phasor.value[1] = __sin(obj->angleFOC_rad);

    // set the phasor in the Park transform
    PARK_setPhasor(obj->parkHandle_I, &phasor);

    // run the Park transform
    PARK_run(obj->parkHandle_I, &(obj->Iab_A),
             (MATH_vec2 *)&(obj->Idq_in_A));

// End of MOTOR1_ESMO && MOTOR1_ENC

//------------------------------------------------------------------------------
#elif defined(MOTOR1_FAST) && defined(MOTOR1_HALL)
    // sensorless-FOC
    MATH_Vec2 phasor;

#if (DMC_BUILDLEVEL <= DMC_LEVEL_3)
    ANGLE_GEN_run(obj->angleGenHandle, obj->speed_int_Hz);
    obj->angleGen_rad = ANGLE_GEN_getAngle(obj->angleGenHandle);
#endif  // (DMC_BUILDLEVEL <= DMC_LEVEL_3)

    // remove offsets
    obj->adcData.V_V.value[0] -=
            obj->adcData.offset_V_sf.value[0] * obj->adcData.VdcBus_V;

    obj->adcData.V_V.value[1] -=
            obj->adcData.offset_V_sf.value[1] * obj->adcData.VdcBus_V;

    obj->adcData.V_V.value[2] -=
            obj->adcData.offset_V_sf.value[2] * obj->adcData.VdcBus_V;

    // run Clarke transform on voltage
    CLARKE_run(obj->clarkeHandle_V,
               &obj->adcData.V_V, &obj->estInputData.Vab_V);

    // run Clarke transform on current
    CLARKE_run(obj->clarkeHandle_I, &obj->adcData.I_A, &obj->estInputData.Iab_A);

    if(obj->flagMotorIdentified == true)
    {
        if(obj->flagRunIdentAndOnLine == true)
        {
            obj->counterTrajSpeed++;

            if(obj->counterTrajSpeed >= objUser->numIsrTicksPerTrajTick)
            {
                // clear counter
                obj->counterTrajSpeed = 0;

                // run a trajectory for speed reference,
                // so the reference changes with a ramp instead of a step
                TRAJ_run(obj->trajHandle_spd);
            }

            obj->enableCurrentCtrl = obj->flagEnableCurrentCtrl;
            obj->enableSpeedCtrl = obj->flagEnableSpeedCtrl;

            // get Id reference for Rs OnLine
            obj->IdRated_A = EST_getIdRated_A(obj->estHandle);
        }
        else
        {
            obj->enableSpeedCtrl = false;
            obj->enableCurrentCtrl = false;
        }

        obj->estInputData.speed_ref_Hz = TRAJ_getIntValue(obj->trajHandle_spd);
        obj->speed_int_Hz = obj->estInputData.speed_ref_Hz;
    }
    else if(EST_isEnabled(obj->estHandle) == true)  // run identification
    {
        // setup the trajectory generator
        EST_setupTrajState(obj->estHandle,
                           obj->Idq_out_A.value[1],
                           obj->speedRef_Hz,
                           0.0f);

        // run the trajectories
        EST_runTraj(obj->estHandle);

        obj->IdRated_A = EST_getIntValue_Id_A(obj->estHandle);

        // store the input data into a buffer
        obj->estInputData.speed_ref_Hz = EST_getIntValue_spd_Hz(obj->estHandle);

        obj->speed_int_Hz = obj->estInputData.speed_ref_Hz;

        obj->enableSpeedCtrl = EST_doSpeedCtrl(obj->estHandle);
        obj->enableCurrentCtrl = EST_doCurrentCtrl(obj->estHandle);

        obj->motorState = MOTOR_CTRL_RUN;
    }

    // store the input data into a buffer
    obj->estInputData.dcBus_V = obj->adcData.VdcBus_V;


    EST_run(obj->estHandle, &obj->estInputData, &obj->estOutputData);

    // compute angle with delay compensation
    obj->angleDelta_rad =
            objUser->angleDelayed_sf_sec * obj->estOutputData.fm_lp_rps;

    obj->angleEST_rad =
            MATH_incrAngle(obj->estOutputData.angle_rad, obj->angleDelta_rad);

    obj->speedEST_Hz = EST_getFm_lp_Hz(obj->estHandle);
    obj->speed_int_Hz = obj->estInputData.speed_ref_Hz;


    obj->oneOverDcBus_invV = obj->estOutputData.oneOverDcBus_invV;


    // Hall Sensor
    HALL_setTimeStamp(obj->hallHandle, HAL_calcCAPCount(obj->halMtrHandle));
    HALL_run(obj->hallHandle, obj->speed_int_Hz);
    obj->angleHall_rad = HALL_getAngle_rad(obj->hallHandle);
    obj->speedHall_Hz = HALL_getSpeed_Hz(obj->hallHandle);


    #ifdef MOTOR1_HALL_CAL
    HALL_calibrateIndexAngle(obj->hallHandle, obj->angleEST_rad);
    #endif  // MOTOR1_HALL_CAL

    obj->speedFilter_Hz = obj->speedFilter_Hz *0.875f + obj->speed_Hz * 0.125f;
    obj->speedAbs_Hz = fabsf(obj->speedFilter_Hz);

    // Running state
    if(obj->estimatorMode == ESTIMATOR_MODE_FAST)
    {
        obj->speed_Hz = obj->speedEST_Hz;

        if(obj->motorState >= MOTOR_CL_RUNNING)
        {
            obj->angleFOC_rad = obj->angleEST_rad;
        }
        else if(obj->motorState == MOTOR_OL_START)
        {
            obj->angleFOC_rad = obj->angleEST_rad;
            obj->motorState = MOTOR_CL_RUNNING;
        }
        else if(obj->motorState == MOTOR_ALIGNMENT)
        {
            obj->angleFOC_rad = 0.0f;
            obj->enableSpeedCtrl = false;

            obj->stateRunTimeCnt++;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = obj->alignCurrent_A;
            obj->Idq_out_A.value[1] = 0.0f;

            TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
            EST_setAngle_rad(obj->estHandle, obj->angleFOC_rad);
            HALL_setForceAngleAndIndex(obj->hallHandle, obj->speedRef_Hz);

            if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                     (obj->flagEnableAlignment == false))
            {
                obj->stateRunTimeCnt = 0;
                obj->motorState = MOTOR_OL_START;

                obj->Idq_out_A.value[0] = obj->fluxCurrent_A;

                PI_setUi(obj->piHandle_spd, 0.0f);
            }
        }
        else if(obj->motorState == MOTOR_SEEK_POS)
        {
            obj->enableSpeedCtrl = false;

            obj->stateRunTimeCnt++;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = 0.0f;
            obj->Idq_out_A.value[1] = 0.0f;

            if(obj->stateRunTimeCnt > obj->flyingStartTimeDelay)
            {
                obj->stateRunTimeCnt = 0;

                if(obj->speedAbs_Hz > obj->speedFlyingStart_Hz)
                {
                    obj->speed_int_Hz = obj->speedFilter_Hz;
                    TRAJ_setIntValue(obj->trajHandle_spd, obj->speedFilter_Hz);
                    PI_setUi(obj->piHandle_spd, 0.0f);

                    obj->motorState = MOTOR_CL_RUNNING;
                }
                else
                {
                    obj->angleFOC_rad = 0.0f;
                    obj->motorState = MOTOR_ALIGNMENT;
                }
            }
        }
    }
    else    // if(obj->estimatorMode == ESTIMATOR_MODE_HALL)
    {
        obj->speed_Hz = obj->speedHall_Hz;

        if(obj->motorState >= MOTOR_CL_RUNNING)
        {
            obj->angleFOC_rad = obj->angleHall_rad;
        }
        else if(obj->motorState == MOTOR_OL_START)
        {
            obj->angleFOC_rad = obj->angleHall_rad;
            obj->enableSpeedCtrl = false;

            obj->Idq_out_A.value[0] = 0.0f;

            if(obj->speed_int_Hz > 0.0f)
            {
                obj->IsRef_A = obj->startCurrent_A;
                obj->Idq_out_A.value[1] = obj->startCurrent_A;
            }
            else
            {
                obj->IsRef_A = -obj->startCurrent_A;
                obj->Idq_out_A.value[1] = -obj->startCurrent_A;
            }

            obj->motorState = MOTOR_CL_RUNNING;
            PI_setUi(obj->piHandle_spd, (obj->frswPos_sf * obj->Idq_out_A.value[1]));
        }
        else if(obj->motorState == MOTOR_ALIGNMENT)
        {
            obj->angleFOC_rad = 0.0f;
            obj->enableSpeedCtrl = false;

            obj->stateRunTimeCnt++;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = obj->alignCurrent_A;
            obj->Idq_out_A.value[1] = 0.0f;

            TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
            EST_setAngle_rad(obj->estHandle, obj->angleFOC_rad);
            HALL_setForceAngleAndIndex(obj->hallHandle, obj->speedRef_Hz);

            if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                     (obj->flagEnableAlignment == false))
            {
                obj->stateRunTimeCnt = 0;
                obj->motorState = MOTOR_OL_START;

                obj->Idq_out_A.value[0] = obj->fluxCurrent_A;

                PI_setUi(obj->piHandle_spd, 0.0f);
            }
        }
        else if(obj->motorState == MOTOR_SEEK_POS)
        {
            obj->enableSpeedCtrl = false;

            obj->stateRunTimeCnt++;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = 0.0f;
            obj->Idq_out_A.value[1] = 0.0f;

            if(obj->stateRunTimeCnt > obj->flyingStartTimeDelay)
            {
                obj->stateRunTimeCnt = 0;

                if(obj->speedAbs_Hz > obj->speedFlyingStart_Hz)
                {
                    obj->speed_int_Hz = obj->speedFilter_Hz;
                    TRAJ_setIntValue(obj->trajHandle_spd, obj->speedFilter_Hz);
                    PI_setUi(obj->piHandle_spd, 0.0f);

                    obj->motorState = MOTOR_CL_RUNNING;
                }
                else
                {
                    obj->angleFOC_rad = 0.0f;
                    obj->motorState = MOTOR_ALIGNMENT;
                }
            }
        }
    }

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    obj->angleFOC_rad = obj->angleGen_rad;
#endif

    // compute the sin/cos phasor
    phasor.value[0] = __cos(obj->angleFOC_rad);
    phasor.value[1] = __sin(obj->angleFOC_rad);

    // set the phasor in the Park transform
    PARK_setPhasor(obj->parkHandle_I, &phasor);

    // run the Park transform
    PARK_run(obj->parkHandle_I, &(obj->estInputData.Iab_A),
             (MATH_vec2 *)&(obj->Idq_in_A));

// End of MOTOR1_FAST && MOTOR1_HALL

//------------------------------------------------------------------------------
#elif defined(MOTOR1_ESMO)
    // sensorless-FOC
    MATH_Vec2 phasor;

    ANGLE_GEN_run(obj->angleGenHandle, obj->speed_int_Hz);
    obj->angleGen_rad = ANGLE_GEN_getAngle(obj->angleGenHandle);

    // run Clarke transform on current
    CLARKE_run(obj->clarkeHandle_I, &obj->adcData.I_A, &obj->Iab_A);

    if(obj->flagRunIdentAndOnLine == true)
    {
        obj->counterTrajSpeed++;

        if(obj->counterTrajSpeed >= objUser->numIsrTicksPerTrajTick)
        {
            // clear counter
            obj->counterTrajSpeed = 0;

            // run a trajectory for speed reference,
            // so the reference changes with a ramp instead of a step
            TRAJ_run(obj->trajHandle_spd);
        }

        obj->enableCurrentCtrl = obj->flagEnableCurrentCtrl;
        obj->enableSpeedCtrl = obj->flagEnableSpeedCtrl;
    }
    else
    {
        obj->enableSpeedCtrl = false;
        obj->enableCurrentCtrl = false;
    }

    obj->speed_int_Hz = TRAJ_getIntValue(obj->trajHandle_spd);
    obj->oneOverDcBus_invV = 1.0f / obj->adcData.VdcBus_V;


    // run the eSMO
    ESMO_setSpeedRef(obj->esmoHandle, obj->speed_int_Hz);
    ESMO_run(obj->esmoHandle, obj->adcData.VdcBus_V,
                    &(obj->pwmData.Vabc_pu), &(obj->Iab_A));

    obj->angleComp_rad = obj->speedPLL_Hz * obj->angleDelayed_sf;
    obj->anglePLL_rad = MATH_incrAngle(ESMO_getAnglePLL(obj->esmoHandle), obj->angleComp_rad);

    SPDFR_run(obj->spdfrHandle, obj->anglePLL_rad);
    obj->speedPLL_Hz = SPDFR_getSpeedHz(obj->spdfrHandle);
    obj->speed_Hz = obj->speedPLL_Hz;


    obj->speedFilter_Hz = obj->speedFilter_Hz *0.875f + obj->speed_Hz * 0.125f;
    obj->speedAbs_Hz = fabsf(obj->speedFilter_Hz);

    if(obj->motorState >= MOTOR_CL_RUNNING)
    {
        obj->angleFOC_rad = obj->anglePLL_rad;

        ESMO_updateKslide(obj->esmoHandle);
    }
    else if(obj->motorState == MOTOR_OL_START)
    {
        obj->angleFOC_rad = obj->angleGen_rad;
        obj->enableSpeedCtrl = false;

        obj->Idq_out_A.value[0] = 0.0f;

        if(obj->speed_int_Hz > 0.0f)
        {
            obj->IsRef_A = obj->startCurrent_A;
            obj->Idq_out_A.value[1] = obj->startCurrent_A;
        }
        else
        {
            obj->IsRef_A = -obj->startCurrent_A;
            obj->Idq_out_A.value[1] = -obj->startCurrent_A;
        }

        if(fabsf(obj->speed_int_Hz) >= obj->speedForce_Hz)
        {
            obj->stateRunTimeCnt++;

            TRAJ_setIntValue(obj->trajHandle_spd, obj->speed_int_Hz);

            if(obj->stateRunTimeCnt > obj->forceRunTimeDelay)
            {
                obj->motorState = MOTOR_CL_RUNNING;
                obj->stateRunTimeCnt = 0;

                ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);

                PI_setUi(obj->piHandle_spd, (obj->frswPos_sf * obj->Idq_out_A.value[1]));
            }
        }
    }
    else if(obj->motorState == MOTOR_ALIGNMENT)
    {
        obj->angleFOC_rad = 0.0f;
        obj->enableSpeedCtrl = false;

        obj->stateRunTimeCnt++;

        obj->IsRef_A = 0.0f;
        obj->Idq_out_A.value[0] = obj->alignCurrent_A;
        obj->Idq_out_A.value[1] = 0.0f;

        TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
        ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);
        ANGLE_GEN_setAngle(obj->angleGenHandle, obj->angleFOC_rad);

        if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                 (obj->flagEnableAlignment == false))
        {
            obj->motorState = MOTOR_OL_START;
            obj->stateRunTimeCnt = 0;

            obj->Idq_out_A.value[0] = obj->fluxCurrent_A;

            PI_setUi(obj->piHandle_spd, 0.0);
        }
    }
    else if(obj->motorState == MOTOR_SEEK_POS)
    {
        obj->enableSpeedCtrl = false;

        obj->stateRunTimeCnt++;

        obj->IsRef_A = 0.0f;
        obj->Idq_out_A.value[0] = 0.0f;
        obj->Idq_out_A.value[1] = 0.0f;

        if(obj->stateRunTimeCnt > obj->flyingStartTimeDelay)
        {
            obj->stateRunTimeCnt = 0;

            if(obj->speedAbs_Hz > obj->speedFlyingStart_Hz)
            {
                obj->speed_int_Hz = obj->speedFilter_Hz;
                TRAJ_setIntValue(obj->trajHandle_spd, obj->speedFilter_Hz);
                PI_setUi(obj->piHandle_spd, 0.0f);

                obj->motorState = MOTOR_CL_RUNNING;
            }
            else
            {
                obj->angleFOC_rad = 0.0f;
                obj->motorState = MOTOR_ALIGNMENT;
            }
        }
    }

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    obj->angleFOC_rad = obj->angleGen_rad;
#endif

    // compute the sin/cos phasor
    phasor.value[0] = __cos(obj->angleFOC_rad);
    phasor.value[1] = __sin(obj->angleFOC_rad);

    // set the phasor in the Park transform
    PARK_setPhasor(obj->parkHandle_I, &phasor);

    // run the Park transform
    PARK_run(obj->parkHandle_I, &(obj->Iab_A), (MATH_vec2 *)&(obj->Idq_in_A));

// End of MOTOR1_ESMO

//------------------------------------------------------------------------------
#elif defined(MOTOR1_ENC)
    MATH_Vec2 phasor;

    ANGLE_GEN_run(obj->angleGenHandle, obj->speed_int_Hz);
    obj->angleGen_rad = ANGLE_GEN_getAngle(obj->angleGenHandle);

    // run Clarke transform on current
    CLARKE_run(obj->clarkeHandle_I, &obj->adcData.I_A, &obj->Iab_A);

    if(obj->flagRunIdentAndOnLine == true)
    {
        obj->counterTrajSpeed++;

        if(obj->counterTrajSpeed >= objUser->numIsrTicksPerTrajTick)
        {
            // clear counter
            obj->counterTrajSpeed = 0;

            // run a trajectory for speed reference,
            // so the reference changes with a ramp instead of a step
            TRAJ_run(obj->trajHandle_spd);
        }

        obj->enableCurrentCtrl = obj->flagEnableCurrentCtrl;
        obj->enableSpeedCtrl = obj->flagEnableSpeedCtrl;
    }
    else
    {
        obj->enableSpeedCtrl = false;
        obj->enableCurrentCtrl = false;
    }

    obj->speed_int_Hz = TRAJ_getIntValue(obj->trajHandle_spd);
    obj->oneOverDcBus_invV = 1.0f / obj->adcData.VdcBus_V;


    ENC_run(obj->encHandle);
    obj->angleENC_rad = ENC_getElecAngle(obj->encHandle);

    SPDCALC_run(obj->spdcalcHandle, obj->angleENC_rad);
    obj->speedENC_Hz = SPDCALC_getSpeedHz(obj->spdcalcHandle);

    obj->speed_Hz = obj->speedENC_Hz;


    obj->speedFilter_Hz = obj->speedFilter_Hz *0.875f + obj->speed_Hz * 0.125f;
    obj->speedAbs_Hz = fabsf(obj->speedFilter_Hz);

    if(obj->motorState >= MOTOR_CL_RUNNING)
    {
        obj->angleFOC_rad = obj->angleENC_rad;
    }
    else if(obj->motorState == MOTOR_OL_START)
    {
        obj->angleFOC_rad = obj->angleGen_rad;
        obj->enableSpeedCtrl = false;

        obj->Idq_out_A.value[0] = 0.0f;

        if(obj->speed_int_Hz > 0.0f)
        {
            obj->IsRef_A = obj->startCurrent_A;
            obj->Idq_out_A.value[1] = obj->startCurrent_A;
        }
        else
        {
            obj->IsRef_A = -obj->startCurrent_A;
            obj->Idq_out_A.value[1] = -obj->startCurrent_A;
        }

        if(ENC_getState(obj->encHandle) == ENC_CALIBRATION_DONE)
        {
            obj->motorState = MOTOR_CL_RUNNING;
        }
    }
    else if(obj->motorState == MOTOR_ALIGNMENT)
    {
        obj->angleFOC_rad = 0.0f;
        obj->enableSpeedCtrl = false;

        obj->stateRunTimeCnt++;

        obj->IsRef_A = 0.0f;
        obj->Idq_out_A.value[0] = obj->alignCurrent_A;
        obj->Idq_out_A.value[1] = 0.0f;

        TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
        ANGLE_GEN_setAngle(obj->angleGenHandle, 0.0f);

        if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                 (obj->flagEnableAlignment == false))
        {
            obj->stateRunTimeCnt = 0;
            obj->motorState = MOTOR_OL_START;

            obj->Idq_out_A.value[0] = obj->fluxCurrent_A;

            ENC_setState(obj->encHandle, ENC_WAIT_FOR_INDEX);
            PI_setUi(obj->piHandle_spd, 0.0);
        }
    }
    else if(obj->motorState == MOTOR_SEEK_POS)
    {
        obj->enableSpeedCtrl = false;

        obj->stateRunTimeCnt++;

        obj->IsRef_A = 0.0f;
        obj->Idq_out_A.value[0] = 0.0f;
        obj->Idq_out_A.value[1] = 0.0f;

        if(obj->stateRunTimeCnt > obj->flyingStartTimeDelay)
        {
            obj->stateRunTimeCnt = 0;

            if(obj->speedAbs_Hz > obj->speedFlyingStart_Hz)
            {
                obj->speed_int_Hz = obj->speedFilter_Hz;
                TRAJ_setIntValue(obj->trajHandle_spd, obj->speedFilter_Hz);
                PI_setUi(obj->piHandle_spd, 0.0f);

                obj->motorState = MOTOR_CL_RUNNING;
            }
            else
            {
                obj->angleFOC_rad = 0.0f;
                obj->motorState = MOTOR_ALIGNMENT;
            }
        }
    }

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    obj->angleFOC_rad = obj->angleGen_rad;
#endif

    // compute the sin/cos phasor
    phasor.value[0] = __cos(obj->angleFOC_rad);
    phasor.value[1] = __sin(obj->angleFOC_rad);

    // set the phasor in the Park transform
    PARK_setPhasor(obj->parkHandle_I, &phasor);

    // run the Park transform
    PARK_run(obj->parkHandle_I, &(obj->Iab_A), (MATH_vec2 *)&(obj->Idq_in_A));

// End of MOTOR1_ENC

//------------------------------------------------------------------------------

#elif defined(MOTOR1_FAST)
    // sensorless-FOC
    MATH_Vec2 phasor;

#if ((DMC_BUILDLEVEL <= DMC_LEVEL_3) || defined(MOTOR1_VOLRECT))
    ANGLE_GEN_run(obj->angleGenHandle, obj->speed_int_Hz);
    obj->angleGen_rad = ANGLE_GEN_getAngle(obj->angleGenHandle);
#endif  // ((DMC_BUILDLEVEL <= DMC_LEVEL_3) || defined(MOTOR1_VOLRECT))

#if defined(MOTOR1_VOLRECT)
    VOLREC_run(obj->volrecHandle, obj->adcData.VdcBus_V,
               &(obj->pwmData.Vabc_pu), &(obj->estInputData.Vab_V));
#else  // !MOTOR1_VOLRECT
    // remove offsets
    obj->adcData.V_V.value[0] -=
            obj->adcData.offset_V_sf.value[0] * obj->adcData.VdcBus_V;

    obj->adcData.V_V.value[1] -=
            obj->adcData.offset_V_sf.value[1] * obj->adcData.VdcBus_V;

    obj->adcData.V_V.value[2] -=
            obj->adcData.offset_V_sf.value[2] * obj->adcData.VdcBus_V;

    // run Clarke transform on voltage
    CLARKE_run(obj->clarkeHandle_V,
               &obj->adcData.V_V, &obj->estInputData.Vab_V);
#endif  // MOTOR1_VOLRECT

    // run Clarke transform on current
    CLARKE_run(obj->clarkeHandle_I, &obj->adcData.I_A, &obj->estInputData.Iab_A);

    if(((EST_isMotorIdentified(obj->estHandle) == false) ||
            (EST_getState(obj->estHandle) == EST_STATE_RS)) &&
            (EST_isEnabled(obj->estHandle) == true))
    {   // run identification or Rs Recalibration
        // setup the trajectory generator
        EST_setupTrajState(obj->estHandle,
                           obj->Idq_out_A.value[1],
                           obj->speedRef_Hz,
                           0.0);

        // run the trajectories
        EST_runTraj(obj->estHandle);

        obj->IdRated_A = EST_getIntValue_Id_A(obj->estHandle);

        // store the input data into a buffer
        obj->estInputData.speed_ref_Hz = EST_getIntValue_spd_Hz(obj->estHandle);
        obj->speed_int_Hz = obj->estInputData.speed_ref_Hz;

        obj->enableSpeedCtrl = EST_doSpeedCtrl(obj->estHandle);
        obj->enableCurrentCtrl = EST_doCurrentCtrl(obj->estHandle);

        obj->motorState = MOTOR_CTRL_RUN;
    }
    else if(obj->flagMotorIdentified == true)   // Normal Running
    {
        if(obj->flagRunIdentAndOnLine == true)
        {
            obj->counterTrajSpeed++;

            if(obj->counterTrajSpeed >= objUser->numIsrTicksPerTrajTick)
            {
                // clear counter
                obj->counterTrajSpeed = 0;

                // run a trajectory for speed reference,
                // so the reference changes with a ramp instead of a step
                TRAJ_run(obj->trajHandle_spd);
            }

            obj->enableCurrentCtrl = obj->flagEnableCurrentCtrl;
            obj->enableSpeedCtrl = obj->flagEnableSpeedCtrl;

            // get Id reference for Rs OnLine
            obj->IdRated_A = EST_getIdRated_A(obj->estHandle);
        }
        else
        {
            obj->enableSpeedCtrl = false;
            obj->enableCurrentCtrl = false;
        }

        obj->estInputData.speed_ref_Hz = TRAJ_getIntValue(obj->trajHandle_spd);
        obj->speed_int_Hz = obj->estInputData.speed_ref_Hz;
    }

    // store the input data into a buffer
    obj->estInputData.dcBus_V = obj->adcData.VdcBus_V;


    EST_run(obj->estHandle, &obj->estInputData, &obj->estOutputData);

    // compute angle with delay compensation
    obj->angleDelta_rad =
            objUser->angleDelayed_sf_sec * obj->estOutputData.fm_lp_rps;

    obj->angleEST_rad =
            MATH_incrAngle(obj->estOutputData.angle_rad, obj->angleDelta_rad);

    obj->speedEST_Hz = EST_getFm_lp_Hz(obj->estHandle);
    obj->speed_Hz = obj->speedEST_Hz;


    obj->speedFilter_Hz = obj->speedFilter_Hz *0.875f + obj->speed_Hz * 0.125f;
    obj->speedAbs_Hz = fabsf(obj->speedFilter_Hz);

    obj->oneOverDcBus_invV = obj->estOutputData.oneOverDcBus_invV;

    // Running state
    if(obj->motorState >= MOTOR_CL_RUNNING)
    {
        obj->angleFOC_rad = obj->angleEST_rad;
    }
    else if(obj->motorState == MOTOR_OL_START)
    {
        obj->angleFOC_rad = obj->angleEST_rad;
        obj->motorState = MOTOR_CL_RUNNING;
    }
    else if(obj->motorState == MOTOR_ALIGNMENT)
    {
        obj->angleFOC_rad = 0.0f;
        obj->enableSpeedCtrl = false;

        obj->stateRunTimeCnt++;

        obj->IsRef_A = 0.0f;
        obj->Idq_out_A.value[0] = obj->alignCurrent_A;
        obj->Idq_out_A.value[1] = 0.0f;

        TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);

        if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                 (obj->flagEnableAlignment == false))
        {
            obj->stateRunTimeCnt = 0;
            obj->motorState = MOTOR_OL_START;
            obj->Idq_out_A.value[0] = obj->fluxCurrent_A;

            EST_setAngle_rad(obj->estHandle, obj->angleFOC_rad);
            PI_setUi(obj->piHandle_spd, 0.0);
        }
    }
    else if(obj->motorState == MOTOR_SEEK_POS)
    {
        obj->enableSpeedCtrl = false;

        obj->stateRunTimeCnt++;

        obj->IsRef_A = 0.0f;
        obj->Idq_out_A.value[0] = 0.0f;
        obj->Idq_out_A.value[1] = 0.0f;

        obj->angleFOC_rad = obj->angleEST_rad;

        if(obj->stateRunTimeCnt > obj->flyingStartTimeDelay)
        {
            obj->stateRunTimeCnt = 0;

            if(obj->speedAbs_Hz > obj->speedFlyingStart_Hz)
            {
                obj->speed_int_Hz = obj->speedFilter_Hz;
                TRAJ_setIntValue(obj->trajHandle_spd, obj->speedFilter_Hz);
                PI_setUi(obj->piHandle_spd, 0.0f);

                obj->motorState = MOTOR_CL_RUNNING;
            }
            else
            {
                obj->motorState = MOTOR_ALIGNMENT;
            }
        }
    }

#ifdef MOTOR1_VOLRECT
    if(obj->motorState == MOTOR_CL_RUNNING)
    {
        obj->angleFOC_rad = obj->angleEST_rad;
    }
    else if(obj->motorState == MOTOR_OL_START)
    {
        obj->angleFOC_rad = obj->angleGen_rad;
        obj->enableSpeedCtrl = false;

        Idq_out_A.value[0] = 0.0f;

        if(obj->speed_int_Hz > 0.0f)
        {
            obj->IsRef_A = obj->startCurrent_A;
            obj->Idq_out_A.value[1] = obj->startCurrent_A;
        }
        else
        {
            obj->IsRef_A = -obj->startCurrent_A;
            obj->Idq_out_A.value[1] = -obj->startCurrent_A;
        }

        if(fabsf(estInputData.speed_ref_Hz) >= obj->speedForce_Hz)
        {
            obj->motorState = MOTOR_CL_RUNNING;

            EST_setAngle_rad(obj->estHandle, angleFOC_rad);

            PI_setUi(obj->piHandle_spd, obj->startCurrent_A * 0.5f);
        }
    }
#endif  // MOTOR1_VOLRECT

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    obj->angleFOC_rad = obj->angleGen_rad;
#endif

    // compute the sin/cos phasor
    phasor.value[0] = __cos(obj->angleFOC_rad);
    phasor.value[1] = __sin(obj->angleFOC_rad);

    // set the phasor in the Park transform
    PARK_setPhasor(obj->parkHandle_I, &phasor);

    // run the Park transform
    PARK_run(obj->parkHandle_I, &(obj->estInputData.Iab_A),
             (MATH_vec2 *)&(obj->Idq_in_A));

// End of MOTOR1_FAST
//------------------------------------------------------------------------------

#elif defined(MOTOR1_HALL)
    MATH_Vec2 phasor;

#if (DMC_BUILDLEVEL <= DMC_LEVEL_3)
    ANGLE_GEN_run(obj->angleGenHandle, obj->speed_int_Hz);
    obj->angleGen_rad = ANGLE_GEN_getAngle(obj->angleGenHandle);
#endif  // (DMC_BUILDLEVEL <= DMC_LEVEL_3)

    // run Clarke transform on current
    CLARKE_run(obj->clarkeHandle_I, &obj->adcData.I_A, &obj->Iab_A);

    if(obj->flagRunIdentAndOnLine == true)
    {
        obj->counterTrajSpeed++;

        if(obj->counterTrajSpeed >= objUser->numIsrTicksPerTrajTick)
        {
            // clear counter
            obj->counterTrajSpeed = 0;

            // run a trajectory for speed reference,
            // so the reference changes with a ramp instead of a step
            TRAJ_run(obj->trajHandle_spd);
        }

        obj->enableCurrentCtrl = obj->flagEnableCurrentCtrl;
        obj->enableSpeedCtrl = obj->flagEnableSpeedCtrl;
    }
    else
    {
        obj->enableSpeedCtrl = false;
        obj->enableCurrentCtrl = false;
    }

    obj->speed_int_Hz = TRAJ_getIntValue(obj->trajHandle_spd);
    obj->oneOverDcBus_invV = 1.0f / obj->adcData.VdcBus_V;


    HALL_setTimeStamp(obj->hallHandle, HAL_calcCAPCount(obj->halMtrHandle));
    HALL_run(obj->hallHandle, obj->speed_int_Hz);
    obj->angleHall_rad = HALL_getAngle_rad(obj->hallHandle);
    obj->speedHall_Hz = HALL_getSpeed_Hz(obj->hallHandle);

    obj->speed_Hz = obj->speedHall_Hz;


    obj->speedFilter_Hz = obj->speedFilter_Hz *0.875f + obj->speed_Hz * 0.125f;
    obj->speedAbs_Hz = fabsf(obj->speedFilter_Hz);

    if(obj->motorState >= MOTOR_CL_RUNNING)
    {
        obj->angleFOC_rad = obj->angleHall_rad;
    }
    else if(obj->motorState == MOTOR_OL_START)
    {
        obj->angleFOC_rad = obj->angleHall_rad;
        obj->enableSpeedCtrl = false;

        obj->Idq_out_A.value[0] = 0.0f;

        if(obj->speed_int_Hz > 0.0f)
        {
            obj->IsRef_A = obj->startCurrent_A;
            obj->Idq_out_A.value[1] = obj->startCurrent_A;
        }
        else
        {
            obj->IsRef_A = -obj->startCurrent_A;
            obj->Idq_out_A.value[1] = -obj->startCurrent_A;
        }

        obj->motorState = MOTOR_CL_RUNNING;
        PI_setUi(obj->piHandle_spd, (obj->frswPos_sf * obj->Idq_out_A.value[1]));
    }
    else if(obj->motorState == MOTOR_ALIGNMENT)
    {
        obj->angleFOC_rad = 0.0f;
        obj->enableSpeedCtrl = false;

        obj->stateRunTimeCnt++;

        obj->IsRef_A = 0.0f;
        obj->Idq_out_A.value[0] = obj->alignCurrent_A;
        obj->Idq_out_A.value[1] = 0.0f;

        TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
        HALL_setForceAngleAndIndex(obj->hallHandle, obj->speedRef_Hz);

        if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                 (obj->flagEnableAlignment == false))
        {
            obj->stateRunTimeCnt = 0;
            obj->motorState = MOTOR_OL_START;

            obj->Idq_out_A.value[0] = obj->fluxCurrent_A;

            PI_setUi(obj->piHandle_spd, 0.0);
        }
    }
    else if(obj->motorState == MOTOR_SEEK_POS)
    {
        obj->enableSpeedCtrl = false;

        obj->stateRunTimeCnt++;

        obj->IsRef_A = 0.0f;
        obj->Idq_out_A.value[0] = 0.0f;
        obj->Idq_out_A.value[1] = 0.0f;

        if(obj->stateRunTimeCnt > obj->flyingStartTimeDelay)
        {
            obj->stateRunTimeCnt = 0;

            if(obj->speedAbs_Hz > obj->speedFlyingStart_Hz)
            {
                obj->speed_int_Hz = obj->speedFilter_Hz;
                TRAJ_setIntValue(obj->trajHandle_spd, obj->speedFilter_Hz);
                PI_setUi(obj->piHandle_spd, 0.0f);

                obj->motorState = MOTOR_CL_RUNNING;

                obj->IsRef_A = 0.0f;
                obj->Idq_out_A.value[0] = 0.0f;
            }
            else
            {
                obj->angleFOC_rad = 0.0f;
                obj->motorState = MOTOR_ALIGNMENT;
            }
        }
    }

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    obj->angleFOC_rad = obj->angleGen_rad;
#endif

    // compute the sin/cos phasor
    phasor.value[0] = __cos(obj->angleFOC_rad);
    phasor.value[1] = __sin(obj->angleFOC_rad);

    // set the phasor in the Park transform
    PARK_setPhasor(obj->parkHandle_I, &phasor);

    // run the Park transform
    PARK_run(obj->parkHandle_I, &(obj->Iab_A), (MATH_vec2 *)&(obj->Idq_in_A));

// End of MOTOR1_HALL

//------------------------------------------------------------------------------
#else   // No Any Estimator
#error Not select a right estimator for this project
#endif  // (ESTIMATOR)

//---------- Common Speed and Current Loop for all observers -------------------
#if(DMC_BUILDLEVEL >= DMC_LEVEL_4)

#if defined(SFRA_ENABLE)

    if(sfraCollectStart == true)
    {
        collectSFRA(motorHandle_M1);    // Collect noise feedback from loop
    }

    //  SFRA injection
    injectSFRA();                   // create SFRA Noise per 'sfraTestLoop'

    sfraCollectStart = true;       // enable SFRA data collection
#endif  // SFRA_ENABLE

    // run the speed controller
    obj->counterSpeed++;

    if(obj->counterSpeed >= objUser->numCtrlTicksPerSpeedTick)
    {
        obj->counterSpeed = 0;

        if(obj->enableSpeedCtrl == true)
        {
#if defined(SFRA_ENABLE)
            PI_run(obj->piHandle_spd,
                   (obj->speed_int_Hz + sfraNoiseSpd),
                   obj->speed_Hz,
                   (float32_t *)&obj->IsRef_A);
#else     // !SFRA_ENABLE
            PI_run(obj->piHandle_spd,
                   obj->speed_int_Hz,
                   obj->speed_Hz,
                   (float32_t *)&obj->IsRef_A);
#endif  // !SFRA_ENABLE
        }
        else if((obj->motorState >= MOTOR_CL_RUNNING) &&
                (obj->flagMotorIdentified == true))
        {
            if(obj->speed_int_Hz > 0.0f)
            {
                obj->IsRef_A = obj->IsSet_A;
            }
            else
            {
                obj->IsRef_A = -obj->IsSet_A;
            }

            // for switching back speed closed-loop control
            PI_setUi(obj->piHandle_spd, obj->IsRef_A);
        }
    }
#if defined(MOTOR1_FWC) && defined(MOTOR1_MTPA)
    else if(obj->counterSpeed == 1)
    {
        MATH_Vec2 fwcPhasor;

        // get the current angle
        obj->angleCurrent_rad =
                (obj->angleFWC_rad > obj->angleMTPA_rad) ?
                        obj->angleFWC_rad : obj->angleMTPA_rad;

        fwcPhasor.value[0] = __cos(obj->angleCurrent_rad);
        fwcPhasor.value[1] = __sin(obj->angleCurrent_rad);

        if((obj->flagEnableFWC == true) || (obj->flagEnableMTPA == true))
        {
            obj->Idq_out_A.value[0] = obj->IsRef_A * fwcPhasor.value[0];
        }

        obj->Idq_out_A.value[1] = obj->IsRef_A * fwcPhasor.value[1];
    }
    else if(obj->counterSpeed == 2)
    {
        //
        // Compute the output and reference vector voltage
        obj->Vs_V =
                __sqrt((obj->Vdq_out_V.value[0] * obj->Vdq_out_V.value[0]) +
                       (obj->Vdq_out_V.value[1] * obj->Vdq_out_V.value[1]));

        obj->VsRef_V = obj->VsRef_pu * obj->adcData.VdcBus_V;

    }
    else if(obj->counterSpeed == 3)   // FWC
    {
        if(obj->flagEnableFWC == true)
        {
            float32_t angleFWC;

            PI_run(obj->piHandle_fwc,
                   obj->VsRef_V, obj->Vs_V, (float32_t*)&angleFWC);
            obj->angleFWC_rad = MATH_PI_OVER_TWO - angleFWC;
        }
        else
        {
            PI_setUi(obj->piHandle_fwc, 0.0f);
            obj->angleFWC_rad = MATH_PI_OVER_TWO;
        }
    }
    else if(obj->counterSpeed == 4)   // MTPA
    {
        if(obj->flagEnableMTPA == true)
        {
            obj->angleMTPA_rad =
                    MTPA_computeCurrentAngle(mtpaHandle, obj->IsRef_A);
        }
        else
        {
            obj->angleMTPA_rad = MATH_PI_OVER_TWO;
        }
    }
#elif defined(MOTOR1_FWC)
    else if(obj->counterSpeed == 1)
    {
        MATH_Vec2 fwcPhasor;

        // get the current angle
        obj->angleCurrent_rad = obj->angleFWC_rad;

        fwcPhasor.value[0] = __cos(obj->angleCurrent_rad);
        fwcPhasor.value[1] = __sin(obj->angleCurrent_rad);

        if(obj->flagEnableFWC == true)
        {
            obj->Idq_out_A.value[0] = obj->IsRef_A * fwcPhasor.value[0];
        }

        obj->Idq_out_A.value[1] = obj->IsRef_A * fwcPhasor.value[1];
    }
    else if(obj->counterSpeed == 2)
    {
        //
        // Compute the output and reference vector voltage
        obj->Vs_V =
                __sqrt((obj->Vdq_out_V.value[0] * obj->Vdq_out_V.value[0]) +
                       (obj->Vdq_out_V.value[1] * obj->Vdq_out_V.value[1]));

        obj->VsRef_V = obj->VsRef_pu * obj->adcData.VdcBus_V;

    }
    else if(obj->counterSpeed == 3)   // FWC
    {
        if(obj->flagEnableFWC == true)
        {
            float32_t angleFWC;

            PI_run(obj->piHandle_fwc,
                   obj->VsRef_V, obj->Vs_V, (float32_t*)&angleFWC);
            obj->angleFWC_rad = MATH_PI_OVER_TWO - angleFWC;
        }
        else
        {
            PI_setUi(obj->piHandle_fwc, 0.0f);
            obj->angleFWC_rad = MATH_PI_OVER_TWO;
        }
    }
#elif defined(MOTOR1_MTPA)
    else if(counterSpeed == 1)
    {
        MATH_Vec2 fwcPhasor;

        // get the current angle
        angleCurrentM1_rad = angleMTPA_rad;

        fwcPhasor.value[0] = __cos(angleCurrentM1_rad);
        fwcPhasor.value[1] = __sin(angleCurrentM1_rad);

        if(flagEnableMTPAM1 == true)
        {
            Idq_out_A.value[0] = obj->IsRef_A * fwcPhasor.value[0];
        }

        Idq_out_A.value[1] = obj->IsRef_A * fwcPhasor.value[1];
    }
    else if(counterSpeed == 4)   // MTPA
    {
        if(flagEnableMTPAM1 == true)
        {
            angleMTPA_rad = MTPA_computeCurrentAngle(mtpaHandle, obj->IsRef_A);
        }
        else
        {
            angleMTPA_rad = MATH_PI_OVER_TWO;
        }
    }
#else   // !MOTOR1_MTPA && !MOTOR1_FWC
    obj->Idq_out_A.value[1] = obj->IsRef_A;
#endif  // !MOTOR1_MTPA && !MOTOR1_FWC/

#if !defined(STEP_RP_EN)
    obj->IdqRef_A.value[0] = obj->Idq_out_A.value[0] + obj->IdRated_A;
#endif  // STEP_RP_EN

#if defined(MOTOR1_FAST)
    // update Id reference for Rs OnLine
    EST_updateId_ref_A(obj->estHandle, &obj->IdqRef_A.value[0]);
#endif  // MOTOR1_FAST

#if !defined(STEP_RP_EN)
#if defined(MOTOR1_VIBCOMP)
    // get the Iq reference value plus vibration compensation
    obj->IdqRef_A.value[1] = Idq_out_A.value[1] +
            VIB_COMP_inline_run(vibCompHandle, angleFOCM1_rad, Idq_in_A.value[1]);
#else
    obj->IdqRef_A.value[1] = obj->Idq_out_A.value[1];
#endif  // MOTOR1_VIBCOMP
#else   // STEP_RP_EN
    if(GRAPH_getBufferMode(&stepRPVars) != GRAPH_STEP_RP_TORQUE)
    {
        obj->IdqRef_A.value[1] = obj->Idq_out_A.value[1];
    }
    else
    {
        PI_setUi(obj->piHandle_spd, obj->IdqRef_A.value[1]);
    }
#endif  // STEP_RP_EN

#if defined(MOTOR1_SSIPD) && !defined(MOTOR1_ISBLDC)
    if(SSIPD_getRunState(obj->ssipdHandle) == true)
    {
#if defined(MOTOR1_FAST)
        SSIPD_inine_run(obj->ssipdHandle, &(obj->estInputData.Iab_A));
#else  // !MOTOR1_FAST
        SSIPD_inine_run(obj->ssipdHandle, &(obj->Iab_A));
#endif  // !MOTOR1_FAST

        obj->Vdq_out_V.value[0] = 0.0f;
        obj->Vdq_out_V.value[0] = SSIPD_getVolInject_V(obj->ssipdHandle);
        obj->angleFOC_rad = SSIPD_getAngleCmd_rad(obj->ssipdHandle);

        TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
    }
#endif  // MOTOR1_SSIPD & !MOTOR1_ISBLDC

#elif(DMC_BUILDLEVEL == DMC_LEVEL_3)
    obj->IdqRef_A.value[0] = obj->Idq_set_A.value[0];
    obj->IdqRef_A.value[1] = obj->Idq_set_A.value[1];
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_3)

    if(obj->enableCurrentCtrl == true)
    {
        // Maximum voltage output
        objUser->maxVsMag_V =
                objUser->maxVsMag_pu * obj->adcData.VdcBus_V;

        PI_setMinMax(obj->piHandle_Id,
                     -objUser->maxVsMag_V, objUser->maxVsMag_V);

#if defined(MOTOR1_DECOUP)
        obj->speed_rps = EST_getFm_lp_rps(obj->estHandle);

        if(objUser->flag_bypassMotorId == true)
        {
            obj->Vdq_offset_V.value[1] = obj->speed_rps *
                    (objSets->flux_Wb + objSets->Ls_d_H * obj->Idq_in_A.value[0]);

            obj->Vdq_offset_V.value[1] = obj->speed_rps *
                    (objSets->Ls_d_H * obj->Idq_in_A.value[0]);
        }
        else
        {
            obj->Vdq_offset_V.value[0] = 0.0f;
            obj->Vdq_offset_V.value[1] = 0.0f;
        }

        // run the Id controller
        PI_run_series(obj->piHandle_Id,
                      obj->IdqRef_A.value[0], obj->Idq_in_A.value[0],
                      obj->Vdq_offset_V.value[0],
                      (float32_t*)&obj->Vdq_out_V.value[0]);

        // calculate Iq controller limits
        float32_t outMax_V = __sqrt((objUser->maxVsMag_V * objUser->maxVsMag_V) -
                          (obj->Vdq_out_V.value[0] * obj->Vdq_out_V.value[0]));

        PI_setMinMax(obj->piHandle_Iq, -outMax_V, outMax_V);

        PI_run_series(obj->piHandle_Iq,
                      obj->IdqRef_A.value[1], obj->Idq_in_A.value[1],
                      obj->Vdq_offset_V.value[1],
                      (float32_t*)&obj->Vdq_out_V.value[1]);
#else  // !MOTOR1_DECOUP

        // run the Id controller
#if defined(SFRA_ENABLE)
        PI_run(obj->piHandle_Id, (obj->IdqRef_A.value[0] + sfraNoiseId),
               obj->Idq_in_A.value[0], (float32_t*)&obj->Vdq_out_V.value[0]);
#else     // !SFRA_ENABLE
            PI_run(obj->piHandle_Id, obj->IdqRef_A.value[0],
                   obj->Idq_in_A.value[0], (float32_t*)&obj->Vdq_out_V.value[0]);
#endif  // !SFRA_ENABLE


        // calculate Iq controller limits
        float32_t outMax_V = __sqrt((objUser->maxVsMag_V * objUser->maxVsMag_V) -
                          (obj->Vdq_out_V.value[0] * obj->Vdq_out_V.value[0]));

        PI_setMinMax(obj->piHandle_Iq, -outMax_V, outMax_V);

        // run the Iq controller
#if defined(SFRA_ENABLE)
        PI_run(obj->piHandle_Iq, (obj->IdqRef_A.value[1] + sfraNoiseIq),
               obj->Idq_in_A.value[1], (float32_t*)&obj->Vdq_out_V.value[1]);
#else     // !SFRA_ENABLE
        PI_run(obj->piHandle_Iq, obj->IdqRef_A.value[1],
               obj->Idq_in_A.value[1], (float32_t*)&obj->Vdq_out_V.value[1]);
#endif  // !SFRA_ENABLE
#endif  // !MOTOR1_DECOUP
    }

#if(DMC_BUILDLEVEL == DMC_LEVEL_2)
    VS_FREQ_run(obj->VsFreqHandle, obj->speed_int_Hz);
    obj->Vdq_out_V.value[0] = VS_FREQ_getVd_out(obj->VsFreqHandle);
    obj->Vdq_out_V.value[1] = VS_FREQ_getVq_out(obj->VsFreqHandle);
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_2)

    // set the phasor in the inverse Park transform
    IPARK_setPhasor(obj->iparkHandle_V, &phasor);

    // run the inverse Park module
    IPARK_run(obj->iparkHandle_V,
              &obj->Vdq_out_V, &obj->Vab_out_V);

    // setup the space vector generator (SVGEN) module
    SVGEN_setup(obj->svgenHandle,
                obj->oneOverDcBus_invV);

    // run the space vector generator (SVGEN) module
    SVGEN_run(obj->svgenHandle,
              &obj->Vab_out_V, &(obj->pwmData.Vabc_pu));

#if(DMC_BUILDLEVEL == DMC_LEVEL_1)
    // output 50%
    obj->pwmData.Vabc_pu.value[0] = 0.0f;
    obj->pwmData.Vabc_pu.value[1] = 0.0f;
    obj->pwmData.Vabc_pu.value[2] = 0.0f;
#endif

    if(HAL_getPwmEnableStatus(obj->halMtrHandle) == false)
    {
        // clear PWM data
        obj->pwmData.Vabc_pu.value[0] = 0.0f;
        obj->pwmData.Vabc_pu.value[1] = 0.0f;
        obj->pwmData.Vabc_pu.value[2] = 0.0f;
    }

#if defined(MOTOR1_DCLINKSS)
    // write the PWM compare values
    HAL_writePWMData(obj->halMtrHandle, &obj->pwmData);

    // revise PWM compare(CMPA/B) values for shifting switching pattern
    // and, update SOC trigger point
    HAL_runSingleShuntCompensation(obj->halMtrHandle, obj->dclinkHandle,
                         &obj->Vab_out_V, &obj->pwmData, obj->adcData.VdcBus_V);
#else   // !(MOTOR1_DCLINKSS)
#if defined(MOTOR1_OVM)
    else
    {
        // run the PWM compensation and current ignore algorithm
        SVGENCURRENT_compPWMData(obj->svgencurrentHandle,
                                 &obj->pwmData.Vabc_pu, &obj->pwmDataPrev);
    }

    // write the PWM compare values
    HAL_writePWMData(obj->halMtrHandle, &obj->pwmData);

    obj->ignoreShuntNextCycle = SVGENCURRENT_getIgnoreShunt(obj->svgencurrentHandle);
    obj->midVolShunt = SVGENCURRENT_getVmid(obj->svgencurrentHandle);

    // Set trigger point in the middle of the low side pulse
    HAL_setTrigger(obj->halMtrHandle,
                   &obj->pwmData, obj->ignoreShuntNextCycle, obj->midVolShunt);
#else   // !MOTOR1_OVM
    // write the PWM compare values
    HAL_writePWMData(obj->halMtrHandle, &obj->pwmData);
#endif  // !MOTOR1_OVM
#endif // !(MOTOR1_DCLINKSS)

    // Collect current and voltage data to calculate the RMS value
    collectRMSData(motorHandle_M1);

#if defined(STEP_RP_EN)
    // Collect predefined data into arrays
    GRAPH_updateBuffer(&stepRPVars);
#endif  // STEP_RP_EN



#if defined(EPWMDAC_MODE)
    // connect inputs of the PWMDAC module.
    HAL_writePWMDACData(halHandle, &pwmDACData);
#endif  // EPWMDAC_MODE

#endif  // !MOTOR1_ISBLDC

#if defined(DATALOGF2_EN)
    if(DATALOGIF_enable(datalogHandle) == true)
    {
        DATALOGIF_updateWithDMA(datalogHandle);

        // Force trig DMA channel to save the data
        HAL_trigDMAforDLOG(halHandle, 0);
        HAL_trigDMAforDLOG(halHandle, 1);
    }
#endif  // DATALOGF2_EN

#if defined(DAC128S_ENABLE)
    // Write the variables data value to DAC128S085
    DAC128S_writeData(dac128sHandle);
#endif  // DAC128S_ENABLE

    return;
} // end of motor1CtrlISR() function

//
//-- end of this file ----------------------------------------------------------
//
