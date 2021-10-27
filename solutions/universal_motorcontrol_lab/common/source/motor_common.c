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
//! \file   /solutions/fast_uni_lab/common/source/motor_comm.c
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

HAL_Handle    halHandle;      //!< the handle for the hardware abstraction layer
HAL_Obj       hal;            //!< the hardware abstraction layer object
#pragma DATA_SECTION(halHandle, "user_data");
#pragma DATA_SECTION(hal, "user_data");

// Sets up control parameters for stopping motor
void stopMotorControl(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;

    obj->speed_int_Hz = 0.0f;

    obj->flagRunIdentAndOnLine = false;

#ifdef BRAKE_ENABLE
    if(obj->motorState == MOTOR_BRAKE_STOP)
    {
        if(obj->brakingMode == HARDSWITCH_BRAKE_MODE)
        {
            // Exit braking PWM mode
            HAL_exitBrakeResetPWM(obj->halMtrHandle);
        }

        obj->motorState = MOTOR_STOP_IDLE;
        obj->mctrlState = MCTRL_BRAKE_STOP;
        obj->flagEnableBraking = false;

        obj->IsRef_A = 0.0f;
        PI_setUi(obj->piHandle_spd, 0.0f);
        PI_setRefValue(obj->piHandle_spd, 0.0f);
    }
#endif  // BRAKE_ENABLE

#if defined(MOTOR1_ENC)
    if(obj->mctrlState >= MCTRL_CONT_RUN)
    {
        obj->motorState = MOTOR_STOP_IDLE;
        obj->mctrlState = MCTRL_NORM_STOP;
    }
    else if(obj->mctrlState == MCTRL_FAULT_STOP)
    {
        ENC_resetState(obj->encHandle);

        obj->motorState = MOTOR_STOP_IDLE;
        obj->mctrlState = MCTRL_INIT_SET;
    }
#else
    obj->motorState = MOTOR_STOP_IDLE;
    obj->mctrlState = MCTRL_NORM_STOP;
#endif  // MOTOR1_ENC

#if !defined(MOTOR1_ISBLDC)
    SVGEN_setMode(obj->svgenHandle, SVM_COM_C);
#endif  //! MOTOR1_ISBLDC

    obj->restartTimesCnt = 0;

    return;
}

// Sets up control parameters for restarting motor
void restartMotorControl(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;

#ifdef BRAKE_ENABLE
    if(obj->motorState == MOTOR_BRAKE_STOP)
    {
        if(obj->brakingMode == HARDSWITCH_BRAKE_MODE)
        {
            // Exit braking PWM mode
            HAL_exitBrakeResetPWM(obj->halMtrHandle);
        }

        obj->motorState = MOTOR_STOP_IDLE;
        obj->mctrlState = MCTRL_BRAKE_STOP;
        obj->flagEnableBraking = false;

        obj->IsRef_A = 0.0f;
        PI_setUi(obj->piHandle_spd, 0.0f);
        PI_setRefValue(obj->piHandle_spd, 0.0f);
    }
#endif  // BRAKE_ENABLE

    if(obj->mctrlState >= MCTRL_NORM_STOP)
    {
        obj->mctrlState = MCTRL_CONT_RUN;
    }
    else
    {
        obj->mctrlState = MCTRL_FIRST_RUN;
    }

#if defined(MOTOR1_ENC)
    if(obj->estimatorMode == ESTIMATOR_MODE_ENC)
    {
        if(obj->mctrlState >= MCTRL_NORM_STOP)
        {
            obj->motorState = MOTOR_CL_RUNNING;
        }
        else
        {
            obj->motorState = MOTOR_ALIGNMENT;
            ENC_setState(obj->encHandle, ENC_ALIGNMENT);
        }
    }
    else if(obj->flagEnableFlyingStart == true)
    {
        obj->motorState = MOTOR_SEEK_POS;
    }
    else
    {
        obj->motorState = MOTOR_ALIGNMENT;
        ENC_setState(obj->encHandle, ENC_ALIGNMENT);
    }
    //MOTOR1_ENC
#else   // !MOTOR1_ENC
    if(obj->flagEnableFlyingStart == true)
    {
        obj->motorState = MOTOR_SEEK_POS;
    }
    else
    {
        obj->motorState = MOTOR_ALIGNMENT;
    }

#endif  // !MOTOR1_ENC

#if defined(MOTOR1_ESMO)
    ESMO_resetPLL(obj->esmoHandle);
#endif  // MOTOR1_ESMO

#if defined(MOTOR1_HALL)
    HALL_resetParams(obj->hallHandle);
    HAL_resetCAPTimeStamp(obj->halMtrHandle);
#endif  // MOTOR1_HALL

#if defined(MOTOR1_ISBLDC)
    MOD6CNT_setCounter(obj->mod6cntHandle, 0);

    RIMPULSE_resetParameters(obj->rimpulseHandle);

    obj->bldcHandle->pwmDuty = 0.0f;
    PI_setUi(obj->piHandle_Iq, obj->bldcHandle->pwmDutyStart);

    ISBLDC_resetState(obj->isbldcHandle);

#endif  // MOTOR1_ISBLDC

    obj->speed_int_Hz = 0.0f;

#if !defined(MOTOR1_ISBLDC)
    SVGEN_setMode(obj->svgenHandle, SVM_COM_C);
#endif  //! MOTOR1_ISBLDC

    obj->flagRunIdentAndOnLine = true;
    obj->stateRunTimeCnt = 0;
    obj->startSumTimesCnt++;

#if defined(SFRA_ENABLE)
    sfraCollectStart = false;       // disable SFRA data collection
#endif  // SFRA_ENABLE
    return;
}

// Resets motor control parameters for restarting motor
void resetMotorControl(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;

    if(obj->flagEnableFlyingStart == false)
    {
#if defined(MOTOR1_FAST)
        // disable the estimator
        EST_disable(obj->estHandle);

        // disable the trajectory generator
        EST_disableTraj(obj->estHandle);
#endif // MOTOR1_FAST

        TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
    }
    else
    {
        TRAJ_setIntValue(obj->trajHandle_spd, obj->speed_Hz);

        obj->stateRunTimeCnt = 0;
        obj->flagStateFlyingStart = false;
    }

    TRAJ_setTargetValue(obj->trajHandle_spd, 0.0f);

    // disable the PWM
    HAL_disablePWM(obj->halMtrHandle);

    // clear integral outputs of the controllers
    PI_setRefValue(obj->piHandle_Id, 0.0f);
    PI_setRefValue(obj->piHandle_Iq, 0.0f);
    PI_setRefValue(obj->piHandle_spd, 0.0f);

    PI_setUi(obj->piHandle_Id, 0.0f);
    PI_setUi(obj->piHandle_Iq, 0.0f);
    PI_setUi(obj->piHandle_spd, 0.0f);

    // clear current references
    obj->Idq_out_A.value[0] = 0.0f;
    obj->Idq_out_A.value[1] = 0.0f;

    obj->IdRated_A = 0.0f;
    obj->IsRef_A = 0.0f;

    obj->angleCurrent_rad = 0.0f;

#if defined(MOTOR1_FWC)
    PI_setUi(obj->piHandle_fwc, 0.0f);
#endif  // MOTOR1_FWC

    obj->stateRunTimeCnt = 0;

    obj->overLoadTimeCnt = 0;
    obj->motorStallTimeCnt = 0;
    obj->lostPhaseTimeCnt = 0;
    obj->unbalanceTimeCnt = 0;
    obj->startupFailTimeCnt = 0;
    obj->overSpeedTimeCnt = 0;

#if !defined(MOTOR1_ISBLDC)
    SVGEN_setMode(obj->svgenHandle, SVM_COM_C);
#endif  //! MOTOR1_ISBLDC

    return;
}

// timer base is 5ms
void runMotorMonitor(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(handle->motorSetsHandle);

#if defined(MOTOR1_FAST)
    if(obj->flagEnableRsOnLine == true)
    {
        if(obj->flagRsOnLineContinue == true)
        {
            obj->flagStartRsOnLine = true;
        }
        else if(obj->RsOnlineTimeCnt == 0)
        {
            if(EST_getFlag_enableRsOnLine(obj->estHandle) == true)
            {
                obj->RsOnlineTimeCnt = objSets->RsOnlineWaitTimeSet;
                obj->flagStartRsOnLine = false;
            }
            else
            {
                obj->RsOnlineTimeCnt = objSets->RsOnlineWorkTimeSet;
                obj->flagStartRsOnLine = true;
            }
        }
        else
        {
            obj->RsOnlineTimeCnt--;
        }
    }
#endif // MOTOR1_FAST

    if(obj->stopWaitTimeCnt > 0)
    {
        obj->stopWaitTimeCnt--;
    }

    // Check if DC bus voltage is over threshold
    if(obj->adcData.VdcBus_V > objSets->overVoltageFault_V)
    {
        if(obj->overVoltageTimeCnt > objSets->voltageFaultTimeSet)
        {
            obj->faultMtrNow.bit.overVoltage = 1;
        }
        else
        {
            obj->overVoltageTimeCnt++;
        }
    }
    else if(obj->adcData.VdcBus_V < objSets->overVoltageNorm_V)
    {
        if(obj->overVoltageTimeCnt == 0)
        {
            obj->faultMtrNow.bit.overVoltage = 0;
        }
        else
        {
            obj->overVoltageTimeCnt--;
        }
    }

    // Check if DC bus voltage is under threshold
    if(obj->adcData.VdcBus_V < objSets->underVoltageFault_V)
    {
        if(obj->underVoltageTimeCnt > objSets->voltageFaultTimeSet)
        {
            obj->faultMtrNow.bit.underVoltage = 1;
        }
        else
        {
            obj->underVoltageTimeCnt++;
        }
    }
    else if(obj->adcData.VdcBus_V > objSets->underVoltageNorm_V)
    {
        if(obj->underVoltageTimeCnt == 0)
        {
            obj->faultMtrNow.bit.underVoltage = 0;
        }
        else
        {
            obj->underVoltageTimeCnt--;
        }
    }

    // check these fault when motor is running
    if(obj->mctrlState == MCTRL_CONT_RUN)
    {
        // Over Load Check
        if(obj->powerActive_W > objSets->overLoadSet_W)
        {
            if(obj->overLoadTimeCnt > objSets->overLoadTimeSet)
            {
                obj->faultMtrNow.bit.overLoad = 1;
                obj->overLoadTimeCnt = 0;
            }
            else
            {
                obj->overLoadTimeCnt++;
            }
        }
        else if(obj->overLoadTimeCnt > 0)
        {
            obj->overLoadTimeCnt--;
        }

        // Motor Stall
        if( (obj->Is_A > objSets->stallCurrentSet_A)
                && (obj->speedAbs_Hz < objSets->speedFailMinSet_Hz))
        {
            if(obj->motorStallTimeCnt > objSets->motorStallTimeSet)
            {
                obj->faultMtrNow.bit.motorStall = 1;
                obj->motorStallTimeCnt = 0;
            }
            else
            {
                obj->motorStallTimeCnt++;
            }
        }
        else if(obj->motorStallTimeCnt > 0)
        {
            obj->motorStallTimeCnt--;
        }

        // (obj->torque_Nm < objSets->toqueFailMinSet_Nm)
        // Motor Lost Phase Fault Check
        if( (obj->speedAbs_Hz > objSets->speedFailMinSet_Hz) &&
            ( (obj->Irms_A[0] < objSets->lostPhaseSet_A) ||
              (obj->Irms_A[1] < objSets->lostPhaseSet_A) ||
              (obj->Irms_A[2] < objSets->lostPhaseSet_A)) )
        {
            if(obj->lostPhaseTimeCnt > objSets->lostPhaseTimeSet)
            {
                obj->faultMtrNow.bit.motorLostPhase = 1;
                obj->lostPhaseTimeCnt = 0;
            }
            else
            {
                obj->lostPhaseTimeCnt++;
            }
        }
        else if(obj->lostPhaseTimeCnt > 0)
        {
            obj->lostPhaseTimeCnt--;
        }

        // Only when the torque is great than a setting value
        if(obj->Is_A > objSets->IsFailedChekSet_A)
        {
            // Motor Phase Current Unbalance
            if(obj->unbalanceRatio > objSets->unbalanceRatioSet)
            {
                if(obj->unbalanceTimeCnt > objSets->unbalanceTimeSet)
                {
                    obj->faultMtrNow.bit.currentUnbalance = 1;
                    obj->unbalanceTimeCnt = 0;
                }
                else
                {
                    obj->unbalanceTimeCnt++;
                }
            }
            else if(obj->unbalanceTimeCnt > 0)
            {
                obj->unbalanceTimeCnt--;
            }

            // Motor Startup Failed
            if( (obj->Is_A < objSets->stallCurrentSet_A)
               && (obj->speedAbs_Hz < objSets->speedFailMinSet_Hz))
            {
                if(obj->startupFailTimeCnt > objSets->startupFailTimeSet)
                {
                    obj->faultMtrNow.bit.startupFailed = 1;
                    obj->startupFailTimeCnt = 0;
                }
                else
                {
                    obj->startupFailTimeCnt++;
                }
            }
            else if(obj->startupFailTimeCnt > 0)
            {
                obj->startupFailTimeCnt--;
            }

            // Motor Over speed
            if(obj->speedAbs_Hz > objSets->speedFailMaxSet_Hz)
            {
                if(obj->overSpeedTimeCnt > objSets->overSpeedTimeSet)
                {
                    obj->faultMtrNow.bit.overSpeed = 1;
                    obj->overSpeedTimeCnt = 0;
                }
                else
                {
                    obj->overSpeedTimeCnt++;
                }
            }
            else if(obj->overSpeedTimeCnt > 0)
            {
                obj->overSpeedTimeCnt--;
            }
        } // obj->Is_A > objSets->IsFailedChekSet_A
    } // obj->operateState == OPERATE_State_Run

    return;
}

void collectRMSData(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;

    obj->IrmsCalSum[0] += obj->adcData.I_A.value[0] * obj->adcData.I_A.value[0];
    obj->IrmsCalSum[1] += obj->adcData.I_A.value[1] * obj->adcData.I_A.value[1];
    obj->IrmsCalSum[2] += obj->adcData.I_A.value[2] * obj->adcData.I_A.value[2];

    obj->VrmsCalSum[0] += obj->adcData.V_V.value[0] * obj->adcData.V_V.value[0];
    obj->VrmsCalSum[1] += obj->adcData.V_V.value[1] * obj->adcData.V_V.value[1];
    obj->VrmsCalSum[2] += obj->adcData.V_V.value[2] * obj->adcData.V_V.value[2];

    obj->VIrmsIsrCnt++;

    if(obj->VIrmsIsrCnt > obj->VIrmsIsrSet)
    {
        obj->IrmsPrdSum[0] = obj->IrmsCalSum[0];
        obj->IrmsPrdSum[1] = obj->IrmsCalSum[1];
        obj->IrmsPrdSum[2] = obj->IrmsCalSum[2];

        obj->IrmsCalSum[0] = 0.0f;
        obj->IrmsCalSum[1] = 0.0f;
        obj->IrmsCalSum[2] = 0.0f;

        obj->VrmsPrdSum[0] = obj->VrmsCalSum[0];
        obj->VrmsPrdSum[1] = obj->VrmsCalSum[1];
        obj->VrmsPrdSum[2] = obj->VrmsCalSum[2];

        obj->VrmsCalSum[0] = 0.0f;
        obj->VrmsCalSum[1] = 0.0f;
        obj->VrmsCalSum[2] = 0.0f;

        obj->VIrmsIsrCnt = 0;
        obj->flagVIrmsCal = true;
    }
}

void calculateRMSData(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;

    float32_t IrmsMax_A, IrmsMin_A, VIrmsIsrSet;

    if(obj->flagVIrmsCal == true)
    {
        obj->flagVIrmsCal = false;

        obj->Irms_A[0] =
                sqrtf(obj->IrmsPrdSum[0] * obj->IrmsCalSF);

        obj->Irms_A[1] =
                sqrtf(obj->IrmsPrdSum[1] * obj->IrmsCalSF);

        obj->Irms_A[2] =
                sqrtf(obj->IrmsPrdSum[2] * obj->IrmsCalSF);

        obj->Vrms_V[0] =
                sqrtf(obj->VrmsPrdSum[0] * obj->VrmsCalSF);

        obj->Vrms_V[1] =
                sqrtf(obj->VrmsPrdSum[1] * obj->VrmsCalSF);

        obj->Vrms_V[2] =
                sqrtf(obj->VrmsPrdSum[2] * obj->VrmsCalSF);

        if(obj->Irms_A[0] > obj->Irms_A[1])
        {
            IrmsMax_A = obj->Irms_A[0];
            IrmsMin_A = obj->Irms_A[1];
        }
        else
        {
            IrmsMax_A = obj->Irms_A[0];
            IrmsMin_A = obj->Irms_A[1];
        }

        IrmsMax_A = (obj->Irms_A[2] > IrmsMax_A) ? obj->Irms_A[2] : IrmsMax_A;
        IrmsMin_A = (obj->Irms_A[2] < IrmsMin_A) ? obj->Irms_A[2] : IrmsMin_A;

        VIrmsIsrSet = obj->VIrmsIsrScale / obj->speedAbs_Hz;
        VIrmsIsrSet = (VIrmsIsrSet > obj->VIrmsIsrScale) ?
                obj->VIrmsIsrScale : VIrmsIsrSet;

        obj->VIrmsIsrSet = (uint16_t)(VIrmsIsrSet);

        obj->IrmsCalSF = 1.0f / ((float32_t)(obj->VIrmsIsrSet));
        obj->VrmsCalSF = obj->IrmsCalSF;

        obj->unbalanceRatio =
                (IrmsMax_A - IrmsMin_A) / (IrmsMax_A + IrmsMin_A);

        obj->powerActive_W = obj->Irms_A[0] * obj->Vrms_V[0] +
                obj->Irms_A[1] * obj->Vrms_V[1] + obj->Irms_A[2] * obj->Vrms_V[2];
    }

    obj->powerReal_W = obj->torque_Nm *
            obj->speedAbs_Hz * obj->power_sf;
}

// setupCurrentControllers()
void setupCurrentControllers(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    USER_Params *objUser = (USER_Params *)(handle->userParamsHandle);

    float32_t RoverL_Kp_sf = objUser->RoverL_Kp_sf;
    float32_t dcBus_nominal_V = objUser->dcBus_nominal_V;
    float32_t maxCurrent_A = objUser->maxCurrent_A;
    float32_t RoverL_min_rps = objUser->RoverL_min_rps;
    float32_t currentCtrlPeriod_sec =
                (float32_t)objUser->numCtrlTicksPerCurrentTick /
                    objUser->ctrlFreq_Hz;

    float32_t outMax_V = objUser->Vd_sf * objUser->maxVsMag_V;
    float32_t Kp = RoverL_Kp_sf * dcBus_nominal_V / maxCurrent_A;
    float32_t Ki = RoverL_min_rps * currentCtrlPeriod_sec;

    // set the Id controller
    PI_setGains(obj->piHandle_Id, Kp, Ki);
    PI_setUi(obj->piHandle_Id, 0.0f);
    PI_setRefValue(obj->piHandle_Id, 0.0f);
    PI_setFbackValue(obj->piHandle_Id, 0.0f);
    PI_setFfwdValue(obj->piHandle_Id, 0.0f);
    PI_setMinMax(obj->piHandle_Id, -outMax_V, outMax_V);

    // set the Iq controller
    PI_setGains(obj->piHandle_Iq, Kp, Ki);
    PI_setUi(obj->piHandle_Iq, 0.0f);
    PI_setRefValue(obj->piHandle_Iq, 0.0f);
    PI_setFbackValue(obj->piHandle_Iq, 0.0f);
    PI_setFfwdValue(obj->piHandle_Iq, 0.0f);
    PI_setMinMax(obj->piHandle_Iq, -outMax_V, outMax_V);

    return;
} // end of setupCurrentControllers() function

void setupControllers(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    USER_Params *objUser = (USER_Params *)(handle->userParamsHandle);

#if defined(MOTOR1_ISBLDC)
    float32_t Kp_Iq = 0.2f / objUser->maxCurrent_A;
    float32_t Ki_Iq = 20.0f / objUser->maxCurrent_A * objUser->ctrlPeriod_sec;

    // set the Iq controller
    PI_setGains(obj->piHandle_Iq, Kp_Iq, Ki_Iq);
    PI_setUi(obj->piHandle_Iq, 0.0f);
    PI_setRefValue(obj->piHandle_Iq, 0.0f);
    PI_setFbackValue(obj->piHandle_Iq, 0.0f);
    PI_setFfwdValue(obj->piHandle_Iq, 0.0f);
    PI_setMinMax(obj->piHandle_Iq, -1.0f, 1.0f);

#if (DMC_BUILDLEVEL >= DMC_LEVEL_4)
    float32_t Kp_spd = 2.0f * objUser->maxCurrent_A / objUser->maxFrequency_Hz;
    float32_t Ki_spd = 2.0f * objUser->maxCurrent_A * objUser->ctrlPeriod_sec;

    PI_setGains(obj->piHandle_spd, Kp_spd, Ki_spd);

    PI_setUi(obj->piHandle_spd, 0.0f);
    PI_setRefValue(obj->piHandle_spd, 0.0f);
    PI_setFbackValue(obj->piHandle_spd, 0.0f);
    PI_setFfwdValue(obj->piHandle_spd, 0.0f);

    PI_setMinMax(obj->piHandle_spd, -objUser->maxCurrent_A, objUser->maxCurrent_A);
#else
    float32_t Kp_spd = 1.5 f/ objUser->maxFrequency_Hz;
    float32_t Ki_spd = 2500.0f / objUser->maxFrequency_Hz * objUser->ctrlPeriod_sec;

    PI_setGains(obj->piHandle_spd, Kp_spd, Ki_spd);

    PI_setUi(obj->piHandle_spd, 0.0f);
    PI_setRefValue(obj->piHandle_spd, 0.0f);
    PI_setFbackValue(obj->piHandle_spd, 0.0f);
    PI_setFfwdValue(obj->piHandle_spd, 0.0f);

    PI_setMinMax(obj->piHandle_spd, -1.0f, 1.0f);
#endif  // (DMC_BUILDLEVEL < DMC_LEVEL_4)

#else   // !MOTOR1_ISBLDC
    float32_t Ls_d_H = objUser->motor_Ls_d_H;
    float32_t Ls_q_H = objUser->motor_Ls_q_H;
#if defined(_SOFT_LIB)
    float32_t Rs_Ohm = objUser->motor_Rs_Ohm;
    float32_t RdoverLd_rps = Rs_Ohm / Ls_d_H;
    float32_t RqoverLq_rps = Rs_Ohm / Ls_q_H;
#else   // !(_SOFT_LIB)
    float32_t Rs_d_Ohm = objUser->motor_Rs_d_Ohm;
    float32_t Rs_q_Ohm = objUser->motor_Rs_q_Ohm;
    float32_t RdoverLd_rps = Rs_d_Ohm / Ls_d_H;
    float32_t RqoverLq_rps = Rs_q_Ohm / Ls_q_H;
#endif  // !(_SOFT_LIB)

    float32_t BWc_rps = objUser->BWc_rps;
    float32_t currentCtrlPeriod_sec =
                (float32_t)objUser->numCtrlTicksPerCurrentTick /
                objUser->ctrlFreq_Hz;

#if !defined(MOTOR1_PUMODE)
    float32_t outMax_V = objUser->Vd_sf *
            objUser->maxVsMag_V;

    float32_t Kp_Id = Ls_d_H * BWc_rps;
    float32_t Ki_Id = 0.25f * RdoverLd_rps * currentCtrlPeriod_sec;

    float32_t Kp_Iq = Ls_q_H * BWc_rps;
    float32_t Ki_Iq = 0.25f * RqoverLq_rps * currentCtrlPeriod_sec;

    // set the Id controller
    PI_setGains(obj->piHandle_Id, Kp_Id, Ki_Id);
    PI_setUi(obj->piHandle_Id, 0.0f);
    PI_setRefValue(obj->piHandle_Id, 0.0f);
    PI_setFbackValue(obj->piHandle_Id, 0.0f);
    PI_setFfwdValue(obj->piHandle_Id, 0.0f);
    PI_setMinMax(obj->piHandle_Id, -outMax_V, outMax_V);

    // set the Iq controller
    PI_setGains(obj->piHandle_Iq, Kp_Iq, Ki_Iq);
    PI_setUi(obj->piHandle_Iq, 0.0f);
    PI_setRefValue(obj->piHandle_Iq, 0.0f);
    PI_setFbackValue(obj->piHandle_Iq, 0.0f);
    PI_setFfwdValue(obj->piHandle_Iq, 0.0f);
    PI_setMinMax(obj->piHandle_Iq, 0.0f, 0.0f);
#else   // MOTOR1_PUMODE
    float32_t outMax_pu = objUser->Vd_sf;

    float32_t Kp_Id = Ls_d_H * BWc_rps / objUser->current_sf / 4096.0f;
    float32_t Ki_Id = 0.25f * RdoverLd_rps * currentCtrlPeriod_sec;

    float32_t Kp_Iq = Ls_q_H * BWc_rps / objUser->current_sf / 4096.0f;
    float32_t Ki_Iq = 0.25f * RqoverLq_rps * currentCtrlPeriod_sec;

    // set the Id controller
    PI_setGains(obj->piHandle_Id, Kp_Id, Ki_Id);
    PI_setUi(obj->piHandle_Id, 0.0f);
    PI_setRefValue(obj->piHandle_Id, 0.0f);
    PI_setFbackValue(obj->piHandle_Id, 0.0f);
    PI_setFfwdValue(obj->piHandle_Id, 0.0f);
    PI_setMinMax(obj->piHandle_Id, -outMax_pu, outMax_pu);

    // set the Iq controller
    PI_setGains(obj->piHandle_Iq, Kp_Iq, Ki_Iq);
    PI_setUi(obj->piHandle_Iq, 0.0f);
    PI_setRefValue(obj->piHandle_Iq, 0.0f);
    PI_setFbackValue(obj->piHandle_Iq, 0.0f);
    PI_setFfwdValue(obj->piHandle_Iq, 0.0f);
    PI_setMinMax(obj->piHandle_Iq, 0.0f, 0.0f);
#endif  // MOTOR1_PUMODE

    // set the speed controller
    if(objUser->Kctrl_Wb_p_kgm2 <= 0.01f)
    {
        float32_t Kp_spd1 = 2.5f * objUser->maxCurrent_A / objUser->maxFrequency_Hz;
        float32_t Ki_spd1 = 5.0f * objUser->maxCurrent_A * objUser->ctrlPeriod_sec;

        PI_setGains(obj->piHandle_spd, Kp_spd1, Ki_spd1);
    }
    else
    {
        float32_t speedCtrlPeriod_sec =
            (float32_t)objUser->numCtrlTicksPerSpeedTick /
            objUser->ctrlFreq_Hz;

        float32_t BWdelta = objUser->BWdelta;

        float32_t Kctrl_Wb_p_kgm2 = objUser->Kctrl_Wb_p_kgm2;

        float32_t Kp_spd = BWc_rps / (BWdelta * Kctrl_Wb_p_kgm2);
        float32_t Ki_spd = BWc_rps * speedCtrlPeriod_sec / (BWdelta * BWdelta);

        PI_setGains(obj->piHandle_spd, Kp_spd, Ki_spd);
    }

    PI_setUi(obj->piHandle_spd, 0.0f);
    PI_setRefValue(obj->piHandle_spd, 0.0f);
    PI_setFbackValue(obj->piHandle_spd, 0.0f);
    PI_setFfwdValue(obj->piHandle_spd, 0.0f);
    PI_setMinMax(obj->piHandle_spd,
                 -objUser->maxCurrent_A,
                 objUser->maxCurrent_A);
#endif  // !MOTOR1_ISBLDC

    // copy the Id, Iq and speed controller parameters to motorVars
    getControllers(handle);

#if !defined(MOTOR1_FAST) && !defined(MOTOR2_FAST)
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(handle->motorSetsHandle);

    objSets->Rs_Ohm = objUser->Rs_Ohm;
    objSets->Ls_d_H = objUser->Ls_d_H;
    objSets->Ls_q_H = objUser->Ls_q_H;
    objSets->flux_VpHz = objUser->motor_ratedFlux_Wb * MATH_TWO_PI;
#endif

    return;
} // end of setupControllers() function


//
#if defined(MOTOR1_FWC) || defined(MOTOR2_FWC)
void updateFWCParams(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(handle->motorSetsHandle);

    // Update FW control parameters
    PI_setGains(obj->piHandle_fwc, objSets->Kp_fwc, objSets->Ki_fwc);
    PI_setOutMin(obj->piHandle_fwc, objSets->angleFWCMax_rad);
}
#endif  // MOTOR1_FWC || MOTOR2_FWC

//
#if defined(MOTOR1_MTPA) || defined(MOTOR2_MTPA)
void updateMTPAParams(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(handle->motorSetsHandle);

    if(obj->flagUpdateMTPAParams == true)
    {
        //
        // update motor parameters according to current
        //
        obj->LsOnline_d_H = MTPA_updateLs_d_withLUT(obj->mtpaHandle, obj->Is_A);

        obj->LsOnline_q_H = MTPA_updateLs_q_withLUT(obj->mtpaHandle, obj->Is_A);

        obj->fluxOnline_Wb = objSets->flux_Wb;

        //
        // update the motor constant for MTPA based on
        // the update Ls_d and Ls_q which are the function of Is
        //
        MTPA_computeParameters(obj->mtpaHandle,
                               obj->LsOnline_d_H,
                               obj->LsOnline_q_H,
                               obj->fluxOnline_Wb);
    }

    return;
}
#endif  // MOTOR1_MTPA || MOTOR2_MTPA

#if defined(MOTOR1_FAST) || defined(MOTOR2_FAST)
void runRsOnLine(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(handle->motorSetsHandle);

    // execute Rs OnLine code
    if(obj->flagRunIdentAndOnLine == true)
    {
        if((EST_getState(obj->estHandle) == EST_STATE_ONLINE) &&
                (obj->flagStartRsOnLine == true))
        {
            EST_setFlag_enableRsOnLine(obj->estHandle, true);

            EST_setRsOnLineId_mag_A(obj->estHandle, objSets->RsOnLineCurrent_A);

            float32_t RsError_Ohm =
                    objSets->RsOnLine_Ohm - objSets->Rs_Ohm;

            if(fabsf(RsError_Ohm) < (objSets->Rs_Ohm * 0.05f))
            {
                EST_setFlag_updateRs(obj->estHandle, true);
            }
        }
        else
        {
            EST_setRsOnLineId_mag_A(obj->estHandle, 0.0f);
            EST_setRsOnLineId_A(obj->estHandle, 0.0f);
            EST_setRsOnLine_Ohm(obj->estHandle, EST_getRs_Ohm(obj->estHandle));

            EST_setFlag_enableRsOnLine(obj->estHandle, false);
            EST_setFlag_updateRs(obj->estHandle, false);
        }
    }

    return;
} // end of runRsOnLine() function
#endif // MOTOR1_FAST || MOTOR2_FAST


#if defined(MOTOR1_FAST) || defined(MOTOR2_FAST)
// update motor control variables
void updateGlobalVariables(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(handle->motorSetsHandle);

    // get the states
#ifdef __TMS320C28XX_CLA__
    obj->estState = cla_EST_getState(obj->estHandle);
    obj->trajState = cla_EST_getTrajState(obj->estHandle);
#else
    obj->estState = EST_getState(obj->estHandle);
    obj->trajState = EST_getTrajState(obj->estHandle);
#endif

    // get the rotor resistance
#ifdef __TMS320C28XX_CLA__
    objSets->Rr_Ohm = cla_EST_getRr_Ohm(obj->estHandle);
#else
    objSets->Rr_Ohm = EST_getRr_Ohm(obj->estHandle);
#endif

    // get the stator resistance
#ifdef __TMS320C28XX_CLA__
    objSets->Rs_Ohm = cla_EST_getRs_Ohm(obj->estHandle);
#else
    objSets->Rs_Ohm = EST_getRs_Ohm(obj->estHandle);
#endif

    // get the stator inductance in the direct coordinate direction
#ifdef __TMS320C28XX_CLA__
    objSets->Ls_d_H = cla_EST_getLs_d_H(obj->estHandle);
#else
    objSets->Ls_d_H = EST_getLs_d_H(obj->estHandle);
#endif

    // get the stator inductance in the quadrature coordinate direction
#ifdef __TMS320C28XX_CLA__
    objSets->Ls_q_H = cla_EST_getLs_q_H(obj->estHandle);
#else
    objSets->Ls_q_H = EST_getLs_q_H(obj->estHandle);
#endif

    // get the flux, V/Hz
#ifdef __TMS320C28XX_CLA__
    objSets->flux_Wb   = cla_EST_getFlux_Wb(obj->estHandle);
    objSets->flux_VpHz = cla_EST_getFlux_Wb(obj->estHandle) * MATH_TWO_PI;
#else
    objSets->flux_Wb   = EST_getFlux_Wb(obj->estHandle);
    objSets->flux_VpHz = EST_getFlux_Wb(obj->estHandle) * MATH_TWO_PI;
#endif

    // get the rated magnetizing current value
#ifdef __TMS320C28XX_CLA__
    objSets->magneticCurrent_A = cla_EST_getIdRated_A(obj->estHandle);
#else
    objSets->magneticCurrent_A = EST_getIdRated_A(obj->estHandle);
#endif

    // get R/L
#ifdef __TMS320C28XX_CLA__
    objSets->RoverL_rps = cla_EST_getRoverL_rps(obj->estHandle);
#else
    objSets->RoverL_rps = EST_getRoverL_rps(obj->estHandle);
#endif

    // get the torque estimate
#ifdef __TMS320C28XX_CLA__
    obj->torque_Nm = cla_EST_computeTorque_Nm(obj->estHandle);
#else
    obj->torque_Nm = EST_computeTorque_Nm(obj->estHandle);
#endif

    // get the stator resistance estimate from RsOnLine
#ifdef __TMS320C28XX_CLA__
    objSets->RsOnLine_Ohm = cla_EST_getRsOnLine_Ohm(obj->estHandle);
#else
    objSets->RsOnLine_Ohm = EST_getRsOnLine_Ohm(obj->estHandle);
#endif

    obj->Is_A =
                sqrtf(obj->Idq_in_A.value[0] * obj->Idq_in_A.value[0] +
                      obj->Idq_in_A.value[1] * obj->Idq_in_A.value[1]);

    obj->Vs_V =
                sqrtf(obj->Vdq_out_V.value[0] * obj->Vdq_out_V.value[0] +
                      obj->Vdq_out_V.value[1] * obj->Vdq_out_V.value[1]);

    return;
} // end of updateGlobalVariables() function
#endif // MOTOR1_FAST || MOTOR2_FAST


//! \brief     Sets the number of current sensors
//! \param[in] handle             The Clarke (CLARKE) handle
//! \param[in] numCurrentSensors  The number of current sensors
void setupClarke_I(CLARKE_Handle handle, const uint16_t numCurrentSensors)
{
    float32_t alpha_sf, beta_sf;

    // initialize the Clarke transform module for current
    if(3 == numCurrentSensors)
    {
        alpha_sf = MATH_ONE_OVER_THREE;
        beta_sf = MATH_ONE_OVER_SQRT_THREE;
    }
    else if(2 == numCurrentSensors)
    {
        alpha_sf = 1.0f;
        beta_sf = MATH_ONE_OVER_SQRT_THREE;
    }
    else
    {
        alpha_sf = 0.0f;
        beta_sf = 0.0f;
    }

    // set the parameters
    CLARKE_setScaleFactors(handle, alpha_sf, beta_sf);
    CLARKE_setNumSensors(handle, numCurrentSensors);

    return;
} // end of setupClarke_I() function

//! \brief     Sets the number of voltage sensors
//! \param[in] handle             The Clarke (CLARKE) handle
//! \param[in] numVoltageSensors  The number of voltage sensors
void setupClarke_V(CLARKE_Handle handle,const uint16_t numVoltageSensors)
{
    float32_t alpha_sf,beta_sf;

    // initialize the Clarke transform module for voltage
    if(numVoltageSensors == 3)
    {
        alpha_sf = MATH_ONE_OVER_THREE;
        beta_sf = MATH_ONE_OVER_SQRT_THREE;
    }
    else
    {
        alpha_sf = 0.0f;
        beta_sf = 0.0f;
    }

    // set the parameters
    CLARKE_setScaleFactors(handle, alpha_sf, beta_sf);
    CLARKE_setNumSensors(handle, numVoltageSensors);

    return;
} // end of setupClarke_V() function


#if defined(CMD_POT_EN)
void setExtCmdPotParams(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;

    // set the target speed via POT
    obj->cmdPot.speedConv_sf = USER_M1_POT_SPEED_SF;
    obj->cmdPot.adcMin = USER_M1_POT_ADC_MIN;
    obj->cmdPot.speedMin_Hz = USER_M1_POT_SPEED_MIN_Hz;
    obj->cmdPot.speedMax_Hz = USER_M1_POT_SPEED_MAX_Hz;

    obj->cmdPot.waitTimeCnt = 0;
    obj->cmdPot.waitTimeSet = USER_M1_WAIT_TIME_SET;
    obj->cmdPot.flagCmdRun = false;
    obj->cmdPot.flagEnableCmd = false;

    return;
} // end of setCmdPotParams() function

void updateExtCmdPotFreq(MOTOR_Handle handle)
{
    MOTOR_Vars_t *objMtr = (MOTOR_Vars_t *)handle;
    float32_t speedPot_Hz = 0.0f;

    // set the target speed via POT
    if(objMtr->adcData.potAdc <= objMtr->cmdPot.adcMin)
    {
        objMtr->cmdPot.waitTimeCnt++;

        if(objMtr->cmdPot.waitTimeCnt > objMtr->cmdPot.waitTimeSet)
        {
            objMtr->cmdPot.flagCmdRun = false;
            objMtr->cmdPot.speedSet_Hz = 0.0f;
        }
    }
    else
    {
        if(objMtr->cmdPot.waitTimeCnt == 0)
        {
            objMtr->cmdPot.flagCmdRun = true;
        }
        else
        {
            objMtr->cmdPot.waitTimeCnt--;
        }

        speedPot_Hz = objMtr->cmdPot.speedConv_sf *
                ((float32_t)(((objMtr->adcData.potAdc>>4)<<4) - objMtr->cmdPot.adcMin));

        objMtr->cmdPot.speedSet_Hz =
                (float32_t)((uint32_t)(objMtr->cmdPot.speedSet_Hz * 0.75f + speedPot_Hz * 0.25f));

        if(objMtr->cmdPot.speedSet_Hz < objMtr->cmdPot.speedMin_Hz)
        {
            objMtr->cmdPot.speedSet_Hz = objMtr->cmdPot.speedMin_Hz;
        }
        else if(objMtr->cmdPot.speedSet_Hz > objMtr->cmdPot.speedMax_Hz)
        {
            objMtr->cmdPot.speedSet_Hz = objMtr->cmdPot.speedMax_Hz;
        }
    }

    if((objMtr->cmdPot.flagEnableCmd == true) && (objMtr->faultMtrUse.all == 0))
    {
        objMtr->flagEnableRunAndIdentify = objMtr->cmdPot.flagCmdRun;
        objMtr->speedRef_Hz = objMtr->cmdPot.speedSet_Hz;
    }

    return;
} // end of setCmdPotParams() function
#endif  // CMD_POT_EN

#if defined(HALL_ENABLE) && defined(CMD_CAP_EN)
#error HALL and CMD_CAP can't be enabled at the same time
#elif defined(CMD_CAP_EN)
void setExtCmdCapParams(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    USER_Params *objUser = (USER_Params *)(handle->userParamsHandle);

    obj->cmdCAP.freqScaler = 2.0f * 1000000.0f * objUser->systemFreq_MHz;   // 4 pulses
    obj->cmdCAP.timeStamp = 0;

    obj->cmdCAP.speedMin_Hz = USER_M1_SPEED_CAP_MIN_Hz;
    obj->cmdCAP.speedMax_Hz = USER_M1_SPEED_CAP_MAX_Hz;
    obj->cmdCAP.speedRef_Hz = 0.0f;
    obj->cmdCAP.speedSet_Hz = 0.0f;

    obj->cmdCAP.waitTimeCnt = 0;
    obj->cmdCAP.waitTimeSet = USER_M1_CAP_WAIT_TIME_SET;
    obj->cmdCAP.flagEnableCmd = false;

    return;
} // end of setExtCmdCapParams() function

// ~1ms time base
void updateExtCmdCapFreq(MOTOR_Handle handle, const uint32_t timeStamp)
{
    MOTOR_Vars_t *objMtr = (MOTOR_Vars_t *)handle;
    float32_t speedCap_Hz = 0.0f;

    objMtr->cmdCAP.timeStamp = (objMtr->cmdCAP.timeStamp + timeStamp)>>1;

    if(objMtr->cmdCAP.timeStamp > 100)
    {
        speedCap_Hz = objMtr->cmdCAP.freqScaler / ((float32_t)objMtr->cmdCAP.timeStamp);
    }


    if(GPIO_readPin(MTR1_CAP_FREQ_GPIO) == 0)
    {
        objMtr->cmdCAP.waitTimeCnt++;

        if(objMtr->cmdCAP.waitTimeCnt > objMtr->cmdCAP.waitTimeSet)
        {
            objMtr->cmdCAP.speedSet_Hz = 0.0f;
            objMtr->cmdCAP.waitTimeCnt = 0;
        }
    }
    else if(GPIO_readPin(MTR1_CAP_FREQ_GPIO) == 1)
    {
        if(objMtr->cmdCAP.waitTimeCnt > 5)
        {
            objMtr->cmdCAP.waitTimeCnt -=5;
        }

        if((speedCap_Hz > 1.0f) && (objMtr->cmdCAP.waitTimeCnt <= 5))
        {
            objMtr->cmdCAP.speedSet_Hz =
                    (float32_t)((uint32_t)(objMtr->cmdCAP.speedSet_Hz * 0.75f + speedCap_Hz * 0.25f));
        }
    }

    if((objMtr->cmdCAP.speedSet_Hz > objMtr->cmdCAP.speedMin_Hz) &&
            (objMtr->cmdCAP.speedSet_Hz < objMtr->cmdCAP.speedMax_Hz))
    {
        objMtr->cmdCAP.speedRef_Hz = objMtr->cmdCAP.speedSet_Hz;
        objMtr->cmdCAP.flagCmdRun = true;
    }
    else
    {
        objMtr->cmdCAP.speedRef_Hz = 0.0f;
        objMtr->cmdCAP.flagCmdRun = false;
    }

#if !defined(CMD_SWITCH_EN)
    if((objMtr->cmdCAP.flagEnableCmd == true) && (objMtr->faultMtrUse.all == 0))
    {
        objMtr->flagEnableRunAndIdentify = objMtr->cmdCAP.flagCmdRun;
        objMtr->speedRef_Hz = objMtr->cmdCAP.speedRef_Hz;
    }
#else
    if(objMtr->cmdCAP.flagEnableCmd == true)
    {
        objMtr->speedRef_Hz = objMtr->cmdCAP.speedRef_Hz;
    }
#endif  //
    return;
} // end of updateExtCmdCapFreq() function
#endif  // CMD_CAP_EN

#if defined(CMD_SWITCH_EN)
void setExtCmdSwitchParams(MOTOR_Handle handle)
{
    MOTOR_Vars_t *objMtr = (MOTOR_Vars_t *)handle;

    objMtr->cmdSwtich.delayTimeSet = USER_M1_SWITCH_WAIT_TIME_SET;
    objMtr->cmdSwtich.highTimeCnt = 0;
    objMtr->cmdSwtich.lowTimeCnt = 0;

    objMtr->cmdSwtich.flagCmdRun = false;
    objMtr->cmdSwtich.flagEnablCmd = false;
    return;
} // end of updateExtCmdCapFreq() function

void updateCmdSwitch(MOTOR_Handle handle)
{
    MOTOR_Vars_t *objMtr = (MOTOR_Vars_t *)handle;

    if(GPIO_readPin(MTR1_CMD_SWITCH_GPIO) == 0)
    {
        objMtr->cmdSwtich.lowTimeCnt++;

        if(objMtr->cmdSwtich.lowTimeCnt > objMtr->cmdSwtich.delayTimeSet)
        {
            objMtr->cmdSwtich.flagCmdRun = true;
        }

        if(objMtr->cmdSwtich.highTimeCnt > 0)
        {
            objMtr->cmdSwtich.highTimeCnt--;
        }
    }
    else
    {
        objMtr->cmdSwtich.highTimeCnt++;

        if(objMtr->cmdSwtich.highTimeCnt > objMtr->cmdSwtich.delayTimeSet)
        {
            objMtr->cmdSwtich.flagCmdRun = false;
        }

        if(objMtr->cmdSwtich.lowTimeCnt > 0)
        {
            objMtr->cmdSwtich.lowTimeCnt--;
        }
    }

    if((objMtr->cmdSwtich.flagEnablCmd == true) && (objMtr->faultMtrUse.all == 0))
    {
        objMtr->flagEnableRunAndIdentify = objMtr->cmdSwtich.flagCmdRun;
    }

    return;
} // end of updateCmdSwitch() function

// Use a GPIO to indicate the operating state
void outputCmdState(MOTOR_Handle handle)
{
    MOTOR_Vars_t *objMtr = (MOTOR_Vars_t *)handle;

    if(objMtr->motorState < MOTOR_CL_RUNNING)
    {
        GPIO_writePin(MTR1_CMD_STATE_GPIO, 0);
        GPIO_writePin(MTR1_CMD_STATE_GPIO, 0);
    }
    else
    {
        GPIO_writePin(MTR1_CMD_STATE_GPIO, 1);
        GPIO_writePin(MTR1_CMD_STATE_GPIO, 1);
    }

    return;
} // end of outputCmdState() function
#endif  // CMD_SWITCH_EN

//
//-- end of this file ----------------------------------------------------------
//
