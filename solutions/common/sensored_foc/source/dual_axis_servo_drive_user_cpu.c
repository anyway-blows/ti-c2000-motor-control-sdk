//#############################################################################
//
// FILE:    dual_axis_servo_drive_user.c
//
// TITLE:   Initialize the parameter variables for motor
//
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

// Variables for current feedback offset calculation
float32_t K1 = 0.998;         // Offset filter coefficient K1: 0.05/(T+0.05);
float32_t K2 = 0.001999;      // Offset filter coefficient K2: T/(T+0.05);

uint16_t offsetCalCounter = 0;

//
//initMotorParameters() function enter
//
void initMotorParameters(MOTOR_Vars_t *pMotor, HAL_MTR_Handle mtrHandle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)mtrHandle;

    if(pMotor == &motorVars[0])
    {
        pMotor->motorNum =  MTR_1;

        pMotor->baseFreq =  M1_BASE_FREQ;
        pMotor->pwmHalfPeriod = M1_INV_PWM_HALF_TBPRD;

        pMotor->polePairs = M1_POLES / 2.0;
        pMotor->posMechScaler = 0.25f /((float32_t)M1_ENCODER_LINES);

        pMotor->Ts = 0.001f / ((float32_t)M1_ISR_FREQUENCY);

        pMotor->voltageLimit = (float32_t)(M1_MAXIMUM_VOLTAGE);                 // V
        pMotor->currentLimit = (float32_t)(M1_MAXIMUM_CURRENT);                 // A
        pMotor->speedRefStart = (float32_t)(M1_STARTUP_FREQ / M1_BASE_FREQ);    // PU
        pMotor->speedRefMax = (float32_t)(M1_MAXIMUM_FREQ / M1_BASE_FREQ);      // PU

        // set mock REFERENCES for Speed and current loops
        pMotor->IdRef_start = M1_ID_START;
        pMotor->IdRef_run   = M1_ID_RUN;
        pMotor->IdRef       = M1_ID_RUN;

#if(BUILDLEVEL == FCL_LEVEL5)
        pMotor->IqRef_start = M1_IQ_LEVEL5;
#else
        pMotor->IqRef_start = M1_IQ_NO_LEVEL5;
#endif // BUILDLEVEL

        pMotor->IqRef_start = 0.0;
        pMotor->speedRef    = M1_SPEED_REF;
        pMotor->lsw1Speed   = M1_SPEED_LSW;

        // set up current and voltage scaling coefficient
        pMotor->currentScale = M1_CURRENT_SF;
        pMotor->voltageScale = M1_VOLTAGE_SF;
        pMotor->adcScale = M1_ADC_PU_SCALE_FACTOR;
        pMotor->currentInvSF = M1_CURRENT_INV_SF;
        pMotor->voltageInvSF = M1_VOLTAGE_INV_SF;

        pMotor->Vdcbus = M1_VDCBUS_MIN;
        pMotor->VdcbusMax = M1_VDCBUS_MAX;
        pMotor->VdcbusMin = M1_VDCBUS_MIN;

#if(SAMPLING_METHOD == SINGLE_SAMPLING)
        pMotor->maxModIndex = (M1_TPWM_CARRIER -
                (2 * M1_FCL_COMPUTATION_TIME)) / M1_TPWM_CARRIER;
        pMotor->FCL_params.carrierMid = pMotor->maxModIndex *
                M1_INV_PWM_HALF_TBPRD * 0x10000L;
#elif(SAMPLING_METHOD == DOUBLE_SAMPLING)
        pMotor->maxModIndex = (M1_TPWM_CARRIER -
                (4 * M1_FCL_COMPUTATION_TIME)) / M1_TPWM_CARRIER;
        pMotor->FCL_params.carrierMid = M1_INV_PWM_HALF_TBPRD * 0x10000L;
#endif // SAMPLING_METHOD

#ifdef BOOSTXL_3PHGANINV
        // LAUNCHXL-F280049C + BOOSTXL-3PhGanInv
        // or LAUNCHXL-F280025C + BOOSTXL-3PhGanInv
        pMotor->FCL_params.adcPPBScale = -M1_ADC_PPB_PU_SCALE_FACTOR;
        pMotor->FCL_params.adcAlphaScale = -M2_ADC_PPB_PU_SCALE_FACTOR;
        pMotor->FCL_params.adcBetaScale = -M2_ADC_PPB_PU_SCALE_FACTOR * 2.0f;
#else
        // ControlCard-F280049C/F280025C + IDDK
        pMotor->FCL_params.adcPPBScale =  M1_ADC_PPB_PU_SCALE_FACTOR;
        pMotor->FCL_params.adcAlphaScale = M1_ADC_PPB_PU_SCALE_FACTOR;
        pMotor->FCL_params.adcBetaScale = M1_ADC_PPB_PU_SCALE_FACTOR * 2.0f;
#endif  // BOOSTXL_3PHGANINV

        pMotor->FCL_params.cmidsqrt3 =
                pMotor->FCL_params.carrierMid * __sqrt(3.0f);

        pMotor->FCL_params.tSamp = (1.0F / M1_SAMPLING_FREQ);
        pMotor->FCL_params.Rd    = M1_RS;
        pMotor->FCL_params.Rq    = M1_RS;
        pMotor->FCL_params.Ld    = M1_LD;
        pMotor->FCL_params.Lq    = M1_LQ;
        pMotor->FCL_params.BemfK = M1_KB;
        pMotor->FCL_params.Ibase = M1_BASE_CURRENT;
        pMotor->FCL_params.Wbase = 2.0 * PI * M1_BASE_FREQ;
        pMotor->FCL_params.wccD  = M1_CUR_LOOP_BANDWIDTH;
        pMotor->FCL_params.wccQ  = M1_CUR_LOOP_BANDWIDTH;

        pMotor->drvEnableGateGPIO = M1_EN_GATE_GPIO;
        pMotor->drvFaultTripGPIO = M1_nFAULT_GPIO;
        pMotor->drvClearFaultGPIO = M1_CLR_FAULT_GPIO;

        pMotor->currentThreshHi = 2048 +
                       scaleCurrentValue(M1_MAXIMUM_CURRENT, M1_CURRENT_INV_SF);

        pMotor->currentThreshLo = 2048 -
                       scaleCurrentValue(M1_MAXIMUM_CURRENT, M1_CURRENT_INV_SF);

        //
        // This function initializes the ADC PPB result bases, as well as the
        // ADC module used to sample phase W. Ensure that the final argument
        // passed corresponds to the ADC base used to sample phase W on the
        // HW board
        //
        FCL_initADC_2In(pMotor, M1_IW_ADC_BASE,
                       M1_IV_ADCRESULT_BASE, M1_IV_ADC_PPB_NUM,
                       M1_IW_ADCRESULT_BASE, M1_IW_ADC_PPB_NUM);

        pMotor->adcIntNumber = M1_INT_ADC_NUM;

        pMotor->volDC_PPBRESULT = M1_VDC_ADCRESULT_BASE +
                ADC_PPBxRESULT_OFFSET_BASE + M1_VDC_ADC_PPB_NUM * 2;
    }
    else if(pMotor == &motorVars[1])
    {
        pMotor->motorNum =  MTR_2;

        pMotor->baseFreq =  M2_BASE_FREQ;
        pMotor->pwmHalfPeriod = M2_INV_PWM_HALF_TBPRD;

        pMotor->polePairs = M2_POLES / 2.0;
        pMotor->posMechScaler = 0.25f /((float32_t)M2_ENCODER_LINES);

        pMotor->Ts = 0.001f / ((float32_t)M2_ISR_FREQUENCY);

        pMotor->voltageLimit = (float32_t)(M2_MAXIMUM_VOLTAGE);                 // V
        pMotor->currentLimit = (float32_t)(M2_MAXIMUM_CURRENT);                 // A
        pMotor->speedRefStart = (float32_t)(M2_STARTUP_FREQ / M2_BASE_FREQ);    // PU
        pMotor->speedRefMax = (float32_t)(M2_MAXIMUM_FREQ / M2_BASE_FREQ);      // PU

        // set mock REFERENCES for Speed and current loops
        pMotor->IdRef_start = M2_ID_START;
        pMotor->IdRef_run   = M2_ID_RUN;
        pMotor->IdRef       = M2_ID_RUN;

#if(BUILDLEVEL == FCL_LEVEL5)
        pMotor->IqRef_start = M2_IQ_LEVEL5;
#else
        pMotor->IqRef_start = M2_IQ_NO_LEVEL5;
#endif // BUILDLEVEL

        pMotor->IqRef_start = 0.0;
        pMotor->speedRef    = M2_SPEED_REF;
        pMotor->lsw1Speed   = M2_SPEED_LSW;

        // set up current and voltage scaling coefficient
        pMotor->currentScale = M2_CURRENT_SF;
        pMotor->voltageScale = M2_VOLTAGE_SF;
        pMotor->adcScale = M2_ADC_PU_SCALE_FACTOR;
        pMotor->currentInvSF = M2_CURRENT_INV_SF;
        pMotor->voltageInvSF = M2_VOLTAGE_INV_SF;

        pMotor->Vdcbus = M2_VDCBUS_MIN;
        pMotor->VdcbusMax = M2_VDCBUS_MAX;
        pMotor->VdcbusMin = M2_VDCBUS_MIN;

#if(SAMPLING_METHOD == SINGLE_SAMPLING)
        pMotor->maxModIndex = (M2_TPWM_CARRIER -
                (2 * M2_FCL_COMPUTATION_TIME)) / M2_TPWM_CARRIER;
        pMotor->FCL_params.carrierMid = pMotor->maxModIndex *
                M2_INV_PWM_HALF_TBPRD * 0x10000L;
#elif(SAMPLING_METHOD == DOUBLE_SAMPLING)
        pMotor->maxModIndex = (M2_TPWM_CARRIER -
                (4 * M2_FCL_COMPUTATION_TIME)) / M2_TPWM_CARRIER;
        pMotor->FCL_params.carrierMid = M2_INV_PWM_HALF_TBPRD * 0x10000L;
#endif // SAMPLING_METHOD

#ifdef BOOSTXL_3PHGANINV
        // LAUNCHXL-F280049C + BOOSTXL-3PhGanInv
        // or LAUNCHXL-F280025C + BOOSTXL-3PhGanInv
        pMotor->FCL_params.adcPPBScale = -M2_ADC_PPB_PU_SCALE_FACTOR;
        pMotor->FCL_params.adcAlphaScale = -M2_ADC_PPB_PU_SCALE_FACTOR;
        pMotor->FCL_params.adcBetaScale = -M2_ADC_PPB_PU_SCALE_FACTOR * 2.0f;
#else
        // ControlCard-F280049C/F280025C + IDDK
        pMotor->FCL_params.adcPPBScale =  M2_ADC_PPB_PU_SCALE_FACTOR;
        pMotor->FCL_params.adcAlphaScale = M2_ADC_PPB_PU_SCALE_FACTOR;
        pMotor->FCL_params.adcBetaScale = M2_ADC_PPB_PU_SCALE_FACTOR * 2.0f;
#endif  // BOOSTXL_3PHGANINV

        pMotor->FCL_params.cmidsqrt3 =
                pMotor->FCL_params.carrierMid * __sqrt(3.0f);

        pMotor->FCL_params.tSamp = (1.0F / M2_SAMPLING_FREQ);
        pMotor->FCL_params.Rd    = M2_RS;
        pMotor->FCL_params.Rq    = M2_RS;
        pMotor->FCL_params.Ld    = M2_LD;
        pMotor->FCL_params.Lq    = M2_LQ;
        pMotor->FCL_params.BemfK = M2_KB;
        pMotor->FCL_params.Ibase = M2_BASE_CURRENT;
        pMotor->FCL_params.Wbase = 2.0 * PI * M2_BASE_FREQ;
        pMotor->FCL_params.wccD  = M2_CUR_LOOP_BANDWIDTH;
        pMotor->FCL_params.wccQ  = M2_CUR_LOOP_BANDWIDTH;

        pMotor->drvEnableGateGPIO = M2_EN_GATE_GPIO;
        pMotor->drvFaultTripGPIO = M2_nFAULT_GPIO;
        pMotor->drvClearFaultGPIO = M2_CLR_FAULT_GPIO;

        pMotor->currentThreshHi = 2048 +
                       scaleCurrentValue(M2_MAXIMUM_CURRENT, M2_CURRENT_INV_SF);

        pMotor->currentThreshLo = 2048 -
                       scaleCurrentValue(M2_MAXIMUM_CURRENT, M2_CURRENT_INV_SF);

        //
        // This function initializes the ADC PPB result bases, as well as the
        // ADC module used to sample phase W. Ensure that the final argument
        // passed corresponds to the ADC base used to sample phase W on the
        // HW board
        //
        FCL_initADC_2In(pMotor, M2_IW_ADC_BASE,
                       M2_IV_ADCRESULT_BASE, M2_IV_ADC_PPB_NUM,
                       M2_IW_ADCRESULT_BASE, M2_IW_ADC_PPB_NUM);

        pMotor->adcIntNumber = M2_INT_ADC_NUM;

        pMotor->volDC_PPBRESULT = M2_VDC_ADCRESULT_BASE +
                ADC_PPBxRESULT_OFFSET_BASE + M2_VDC_ADC_PPB_NUM * 2;
    }

    //
    // ensure that the correct PWM base addresses are being passed to the
    // FCL library here. pwmHandle[0:2] should represent inverter phases
    // U/V/W in the hardware
    //
    FCL_initPWM(pMotor,
                obj->pwmHandle[0], obj->pwmHandle[1], obj->pwmHandle[2]);

    // ensure the correct QEP base is being passed
    FCL_initQEP(pMotor, obj->qepHandle);

    return;
}

//
// initControlVars() function enter
//
void initControlVars(MOTOR_Vars_t *pMotor)
{
    pMotor->tripFlagDMC = 0;
    pMotor->tripFlagPrev = 0;
    pMotor->tripCountDMC = 0;

    // Set up the initialization value for some variables
    pMotor->IdRef_start = 0.2f;
    pMotor->IqRef = 0.10f;
    pMotor->speedRef = 0.10f;
    pMotor->lsw1Speed = 0.02f;

    pMotor->alignCnt = 2000;
    pMotor->posPtrMax = 2;
    pMotor->posPtr = 0;
    pMotor->posCntrMax = 5000;
    pMotor->posSlewRate =  0.001;
    pMotor->fclClrCntr = 1;

    // Initialize the RAMPCTRL module
    // Maximum delay rate of ramp control
    pMotor->rc.RampDelayMax = 1;
    pMotor->rc.RampLowLimit = -1.0f;
    pMotor->rc.RampHighLimit = 1.0f;

    // Initialize the RAMPGEN module
    pMotor->rg.StepAngleMax = pMotor->baseFreq * pMotor->Ts;
    pMotor->rg.Gain = 1.0f;
    pMotor->rg.Offset = 1.0f;

    pMotor->pi_pos.Kp = 0.25f;            //10.0;
    pMotor->pi_pos.Ki = 0.001f;           //T*speedLoopPrescaler/0.3;
    pMotor->pi_pos.Umax = 1.0f;
    pMotor->pi_pos.Umin = -1.0f;

    // Initialize the PID module for speed
    pMotor->pid_spd.param.Kp   = 0.250f;
    pMotor->pid_spd.param.Ki   = 0.0010f;
    pMotor->pid_spd.param.Kd   = 0.0f;
    pMotor->pid_spd.param.Kr   = 1.0f;
    pMotor->pid_spd.param.Umax = 0.95f;
    pMotor->pid_spd.param.Umin = -0.95f;

    // Init PI module for ID loop
    pMotor->cmplx_Id.Kp = 1.0f;         // LS * CUR_LOOP_BW;
    pMotor->cmplx_Id.Ki = pMotor->Ts / 0.04f;      // (RS * T) * CUR_LOOP_BW;
    pMotor->cmplx_Id.Kerr = (pMotor->cmplx_Id.Ki * 0.5f) + pMotor->cmplx_Id.Kp;
    pMotor->cmplx_Id.KerrOld = (pMotor->cmplx_Id.Ki * 0.5f) - pMotor->cmplx_Id.Kp;
    pMotor->cmplx_Id.Umax = 0.5f * pMotor->maxModIndex;
    pMotor->cmplx_Id.Umin = -0.5f * pMotor->maxModIndex;

    // Init PI module for IQ loop
    pMotor->cmplx_Iq.Kp = pMotor->cmplx_Id.Kp;
    pMotor->cmplx_Iq.Ki = pMotor->cmplx_Id.Ki;
    pMotor->cmplx_Iq.Kerr =
            (pMotor->cmplx_Iq.Ki * 0.5f) + pMotor->cmplx_Iq.Kp;

    pMotor->cmplx_Iq.KerrOld =
            (pMotor->cmplx_Iq.Ki * 0.5f) - pMotor->cmplx_Iq.Kp;

    pMotor->cmplx_Iq.Umax = 0.8f * pMotor->maxModIndex;
    pMotor->cmplx_Iq.Umin = -0.8f * pMotor->maxModIndex;

    // Initialize the Speed module for speed calculation from QEP/RESOLVER
    pMotor->speed.K1 = 1.0f / (pMotor->baseFreq * pMotor->Ts);

    // Low-pass cut-off frequency
    pMotor->speed.K2 = 1.0f / (1.0f + (2.0f * PI * pMotor->Ts * 5.0f));
    pMotor->speed.K3 = 1.0f - pMotor->speed.K2;
    pMotor->speed.BaseRpm = (60.0F * pMotor->baseFreq) / pMotor->polePairs;


    // Initialize speed observer
    pMotor->speedObs.Kp = 15.0f;
    pMotor->speedObs.Ki = 15.0f * pMotor->Ts;
    pMotor->speedObs.Umax = 1.0f;
    pMotor->speedObs.Umin = -1.0f;
    pMotor->speedObs.ui = 0.0f;
    pMotor->speedObs.thetaMax = pMotor->baseFreq * pMotor->Ts;

    pMotor->speedLoopPrescaler = 10;

    // Variables for SFRA module
    #if(BUILDLEVEL == FCL_LEVEL6)
    sfraNoiseD = 0.0f;
    sfraNoiseQ = 0.0f;
    sfraNoiseW = 0.0f;
    sfraTestLoop = SFRA_TEST_D_AXIS;
    sfraCollectStart = false;
    #endif  // (BUILDLEVEL == FCL_LEVEL6)

    return;
}

//
// initControlVars() function enter
// This function is called to reset the FCL variables and is useful when user
// wants to stop the motor and restart the motor
void resetControlVars(MOTOR_Vars_t *pMotor)
{
    pMotor->runMotor = MOTOR_STOP;

    pMotor->lsw = ENC_ALIGNMENT;
    pMotor->speedWePrev = 0.0f;

    pMotor->cmplx_Id.carryOver = 0.0f;
    pMotor->cmplx_Id.out = 0.0f;
    pMotor->cmplx_Id.err = 0.0f;
    pMotor->cmplx_Id.xErr = 0.0f;

    pMotor->cmplx_Iq.carryOver = 0.0f;
    pMotor->cmplx_Iq.out = 0.0f;
    pMotor->cmplx_Iq.err = 0.0f;
    pMotor->cmplx_Iq.xErr = 0.0f;

    return;
}

//
// initControlVars() function enter
// This function is called to reset the FCL variables and is useful when user
// wants to stop the motor and restart the motor
//
void runOffsetsCalculation(MOTOR_Vars_t *pMotor)
{
    // Feedbacks OFFSET Calibration
    pMotor->offset_currentAs = 0.0f;
    pMotor->offset_currentBs = 0.0f;
    pMotor->offset_currentCs = 0.0f;

    if(pMotor->motorNum == MTR_1)
    {
        for(offsetCalCounter = 0; offsetCalCounter < 10000; offsetCalCounter++)
        {
            EPWM_clearEventTriggerInterruptFlag(pMotor->pwmBaseU);

            while(EPWM_getEventTriggerInterruptStatus(pMotor->pwmBaseU) == false);

            if(offsetCalCounter > 1000)
            {
                // Offsets in phase current sensing
                pMotor->offset_currentAs  = (K1 * pMotor->offset_currentAs) +
                        (((float32_t)(M1_IFB_U)) * K2 * pMotor->adcScale);

                pMotor->offset_currentBs  = (K1 * pMotor->offset_currentBs) +
                        (((float32_t)(M1_IFB_V)) * K2 * pMotor->adcScale);

                pMotor->offset_currentCs  = (K1 * pMotor->offset_currentCs) +
                        (((float32_t)(M1_IFB_W)) * K2 * pMotor->adcScale);
            }
        }

        //
        // Read and update DC BUS voltage for FCL to use
        //
        pMotor->FCL_params.Vdcbus = getVdc(pMotor);

        // setting Iu offset
        ADC_setPPBReferenceOffset(M1_IU_ADC_BASE, M1_IU_ADC_PPB_NUM,
                             (uint16_t)(pMotor->offset_currentAs * ADC_RESOLUTION));
        // setting Iv offset
        ADC_setPPBReferenceOffset(M1_IV_ADC_BASE, M1_IV_ADC_PPB_NUM,
                             (uint16_t)(pMotor->offset_currentBs * ADC_RESOLUTION));

        // setting Iw offset
        ADC_setPPBReferenceOffset(M1_IW_ADC_BASE, M1_IW_ADC_PPB_NUM,
                             (uint16_t)(pMotor->offset_currentCs * ADC_RESOLUTION));

        // setting Vdc offset
        ADC_setPPBReferenceOffset(M1_VDC_ADC_BASE, M1_VDC_ADC_PPB_NUM, 0);
    }
    else if(pMotor->motorNum == MTR_2)
    {
        for(offsetCalCounter = 0; offsetCalCounter < 10000; offsetCalCounter++)
        {
            EPWM_clearEventTriggerInterruptFlag(pMotor->pwmBaseU);

            while(EPWM_getEventTriggerInterruptStatus(pMotor->pwmBaseU) == false);

            if(offsetCalCounter > 1000)
            {
                // Offsets in phase current sensing
                pMotor->offset_currentAs  = (K1 * pMotor->offset_currentAs) +
                        (((float32_t)(M1_IFB_U)) * K2 * pMotor->adcScale);

                pMotor->offset_currentBs  = (K1 * pMotor->offset_currentBs) +
                        (((float32_t)(M1_IFB_V)) * K2 * pMotor->adcScale);

                pMotor->offset_currentCs  = (K1 * pMotor->offset_currentCs) +
                        (((float32_t)(M1_IFB_W)) * K2 * pMotor->adcScale);
            }
        }

        //
        // Read and update DC BUS voltage for FCL to use
        //
        pMotor->FCL_params.Vdcbus = getVdc(pMotor);

        // setting Iu offset
        ADC_setPPBReferenceOffset(M2_IU_ADC_BASE, M2_IU_ADC_PPB_NUM,
                             (uint16_t)(pMotor->offset_currentAs * ADC_RESOLUTION));
        // setting Iv offset
        ADC_setPPBReferenceOffset(M2_IV_ADC_BASE, M2_IV_ADC_PPB_NUM,
                             (uint16_t)(pMotor->offset_currentBs * ADC_RESOLUTION));

        // setting Iw offset
        ADC_setPPBReferenceOffset(M2_IW_ADC_BASE, M2_IW_ADC_PPB_NUM,
                             (uint16_t)(pMotor->offset_currentCs * ADC_RESOLUTION));

        // setting Vdc offset
        ADC_setPPBReferenceOffset(M2_VDC_ADC_BASE, M2_VDC_ADC_PPB_NUM, 0);
    }

    pMotor->offsetDoneFlag = 1;

    return;
}

//
// End of Code
//
