//#############################################################################
//
// FILE:    multi_axis_lead_drive_user.c
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
#include "multi_axis_lead_drive_main.h"


// Variables for current feedback offset calculation
float32_t K1 = 0.998;         // Offset filter coefficient K1: 0.05/(T+0.05);
float32_t K2 = 0.001999;      // Offset filter coefficient K2: T/(T+0.05);
#pragma DATA_SECTION(K1, "ramInitVars");
#pragma DATA_SECTION(K2, "ramInitVars");

uint16_t offsetCalCounter = 0;
#pragma DATA_SECTION(offsetCalCounter, "ramInitVars");

//
//initMotorParameters() function enter
//
void initMotorParameters(MOTOR_Vars_t *pMotor, HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    pMotor->faultStatusFlag = 0;
    pMotor->faultStatusPrev = 0;
    pMotor->faultTimesCount = 0;

    pMotor->posSlewRate = 0.001;
    pMotor->posCntrMax = 5000;

    pMotor->baseFreq =  M_BASE_FREQ;
    pMotor->polePairs = M_POLES / 2.0;
    pMotor->posMechScaler = 0.25F/((float32_t)M_ENC_SLOTS);

    pMotor->Ts = 0.001F / ((float32_t)M_ISR_FREQUENCY);

    pMotor->voltageLimit = (float32_t)(M_MAXIMUM_VOLTAGE);              // V
    pMotor->currentLimit = (float32_t)(M_MAXIMUM_CURRENT);              // A
    pMotor->speedRefStart = (float32_t)(M_STARTUP_FREQ/M_BASE_FREQ);    // PU
    pMotor->speedRefMax = (float32_t)(M_MAXIMUM_FREQ/M_BASE_FREQ);      // PU

    // set mock REFERENCES for Speed and current loops
    pMotor->IdRef_start = M_ID_START;
    pMotor->IdRef_run   = M_ID_RUN;
    pMotor->IdRef       = M_ID_RUN;
#if(BUILDLEVEL == FCL_LEVEL5)
    pMotor->IqRef_start = M_IQ_LEVEL5;
#else
    pMotor->IqRef_start = M_IQ_NO_LEVEL5;
#endif // BUILDLEVEL

    pMotor->IqRef = 0.0;

#if(BUILDLEVEL == FCL_LEVEL7)
    pMotor->IdRefDelta = 0.0001;
    pMotor->IqRefDelta = 0.0001;
#else
    pMotor->IdRefDelta = 0.005;
    pMotor->IqRefDelta = 0.010;
#endif

    pMotor->speedRef    = M_SPEED_REF;
    pMotor->lsw1Speed   = 0.02;     // 5Hz

#if(BUILDLEVEL == FCL_LEVEL7)
    pMotor->posMechTheta = 0.15;    // Just for FSI debug
    pMotor->posElecTheta = 0.60;    // Just for FSI debug
    pMotor->speedWe = 0.10;         // Just for FST debug
#endif // (BUILDLEVEL == FCL_LEVEL7)

    // set up current and voltage scaling coefficient
    pMotor->currentScale = M_CURRENT_SENSE_SCALE;
    pMotor->voltageScale = M_VOLTAGE_SENSE_SCALE;
    pMotor->adcScale = M_ADC_PU_SCALE_FACTOR;

    pMotor->Vdcbus = M_VDCBUS_MIN;
    pMotor->VdcbusMax = M_VDCBUS_MAX;
    pMotor->VdcbusMin = M_VDCBUS_MIN;

#if(SAMPLING_METHOD == SINGLE_SAMPLING)
    pMotor->maxModIndex = (M_TPWM_CARRIER -
            (2 * M_FCL_COMPUTATION_TIME)) / M_TPWM_CARRIER;
    pMotor->FCL_params.carrierMid = pMotor->maxModIndex *
            M_INV_PWM_HALF_TBPRD * 0x10000L;
#elif(SAMPLING_METHOD == DOUBLE_SAMPLING)
    pMotor->maxModIndex = (M_TPWM_CARRIER -
            (4 * M_FCL_COMPUTATION_TIME)) / M_TPWM_CARRIER;
    pMotor->FCL_params.carrierMid = M_INV_PWM_HALF_TBPRD * 0x10000L;
#endif // SAMPLING_METHOD

    // ControlCard-F28388D + IDDK
    pMotor->FCL_params.adcPPBScale = M_ADC_PPB_PU_SCALE_FACTOR;

    pMotor->FCL_params.cmidsqrt3 =
            pMotor->FCL_params.carrierMid * __sqrt(3.0);

    pMotor->FCL_params.tSamp = (1.0F / M_SAMPLING_FREQ);
    pMotor->FCL_params.Rd    = M_RS;
    pMotor->FCL_params.Rq    = M_RS;
    pMotor->FCL_params.Ld    = M_LD;
    pMotor->FCL_params.Lq    = M_LQ;
    pMotor->FCL_params.BemfK = M_KB;
    pMotor->FCL_params.Ibase = M_BASE_CURRENT;
    pMotor->FCL_params.Wbase = 2.0 * PI * M_BASE_FREQ;
    pMotor->FCL_params.wccD  = M_CUR_LOOP_BANDWIDTH;
    pMotor->FCL_params.wccQ  = M_CUR_LOOP_BANDWIDTH;

    // Initialize the RAMPCTRL module
    // Maximum delay rate of ramp control
    pMotor->rc.RampDelayMax = 5;
    pMotor->rc.RampLowLimit = -1.0;
    pMotor->rc.RampHighLimit = 1.0;

    // Initialize the RAMPGEN module
    pMotor->rg.StepAngleMax = pMotor->baseFreq * pMotor->Ts;
    pMotor->rg.Gain = 1.0;
    pMotor->rg.Offset = 1.0;

    // Don't need below commented code if memory is cleared by HAL_clearDataRAM
//    pMotor->rg.Angle = 0;
//    pMotor->rg.Out = 0;

#if(SPD_CNTLR == SPD_PID_CNTLR)
    //
    // PI Controllers Configuration
    // Initialize the PI module for position
    #if(BUILDLEVEL < FCL_LEVEL7)
    pMotor->pi_pos.Kp = 0.5;             //10.0;
    pMotor->pi_pos.Ki = 0.001;           //T*speedLoopPrescaler/0.3;
    pMotor->pi_pos.Umax = 1.0;
    pMotor->pi_pos.Umin = -1.0;
    #endif  // (BUILDLEVEL < FCL_LEVEL7)

    // Initialize the PID module for speed
    pMotor->pid_spd.param.Kp   = 0.35;
    pMotor->pid_spd.param.Ki   = 0.0010;
    pMotor->pid_spd.param.Kd   = 0.0;
    pMotor->pid_spd.param.Kr   = 1.0;
    pMotor->pid_spd.param.Umax = 0.95;
    pMotor->pid_spd.param.Umin = -0.95;
#endif // (SPD_CNTLR == SPD_PID_CNTLR)

#if(SPD_CNTLR == SPD_DCL_CNTLR)
    //
    // initialize DCL controller for position
    //
    #if(BUILDLEVEL < FCL_LEVEL7)
    pMotor->dcl_pos.Kp = 0.2;
    pMotor->dcl_pos.Ki = 0.002;
    pMotor->dcl_pos.Umax = 1.0;
    pMotor->dcl_pos.Umin = -1.0;
    #endif  // (BUILDLEVEL < FCL_LEVEL7)
    //
    // initialize DCL controller for speed
    //
    pMotor->dcl_spd.Kp = 0.35;
    pMotor->dcl_spd.Ki = 0.0010;
    pMotor->dcl_spd.Umax = 0.95;
    pMotor->dcl_spd.Umin = -0.95;

#endif // (SPD_CNTLR == SPD_DCL_CNTLR)

    // Init PI module for ID loop
    pMotor->cmplx_Id.Kp = 1.0;         // LS * CUR_LOOP_BW;
    pMotor->cmplx_Id.Ki = pMotor->Ts / 0.04;      // (RS * T) * CUR_LOOP_BW;
    pMotor->cmplx_Id.Kerr = (pMotor->cmplx_Id.Ki * 0.5) + pMotor->cmplx_Id.Kp;
    pMotor->cmplx_Id.KerrOld = (pMotor->cmplx_Id.Ki * 0.5) - pMotor->cmplx_Id.Kp;
    pMotor->cmplx_Id.Umax = 0.5 * pMotor->maxModIndex;
    pMotor->cmplx_Id.Umin = -0.5 * pMotor->maxModIndex;

    // Don't need below commented code if memory is cleared by HAL_clearDataRAM
    pMotor->cmplx_Id.ref = 0;
    pMotor->cmplx_Id.err = 0;
    pMotor->cmplx_Id.out = 0;

    // Init PI module for IQ loop
    pMotor->cmplx_Iq.Kp = pMotor->cmplx_Id.Kp;
    pMotor->cmplx_Iq.Ki = pMotor->cmplx_Id.Ki;
    pMotor->cmplx_Iq.Kerr =
            (pMotor->cmplx_Iq.Ki * 0.5) + pMotor->cmplx_Iq.Kp;

    pMotor->cmplx_Iq.KerrOld =
            (pMotor->cmplx_Iq.Ki * 0.5) - pMotor->cmplx_Iq.Kp;

    pMotor->cmplx_Iq.Umax = 0.8 * pMotor->maxModIndex;
    pMotor->cmplx_Iq.Umin = -0.8 * pMotor->maxModIndex;

    // Don't need below commented code if memory is cleared by HAL_clearDataRAM
    pMotor->cmplx_Iq.ref = 0;
    pMotor->cmplx_Iq.err = 0;
    pMotor->cmplx_Iq.out = 0;

    // Initialize the Speed module for speed calculation from QEP/RESOLVER
    pMotor->speed.K1 = 1 / (pMotor->baseFreq * pMotor->Ts);

    // Low-pass cut-off frequency
    pMotor->speed.K2 = 1 / (1 + (2 * PI * pMotor->Ts * 5));
    pMotor->speed.K3 = 1 - pMotor->speed.K2;
    pMotor->speed.BaseRpm = (60.0F * pMotor->baseFreq) / pMotor->polePairs;


    pMotor->drvEnableGateGPIO = M_EN_GATE_GPIO;
    pMotor->drvFaultTripGPIO = M_nFAULT_GPIO;
    pMotor->drvClearFaultGPIO = M_CLR_FAULT_GPIO;

#if(BUILDLEVEL >= FCL_LEVEL7)
    pMotor->speedLoopPrescaler = 5;
#else
    pMotor->speedLoopPrescaler = 10;
#endif

    pMotor->alignCnt = 2000;
    pMotor->posPtrMax = 2;

    pMotor->currentThreshHi = 2048 + scaleCurrentValue(M_MAXIMUM_CURRENT);
    pMotor->currentThreshLo = 2048 - scaleCurrentValue(M_MAXIMUM_CURRENT);
    pMotor->voltageThreshHi = scaleVoltageValue(M_MAXIMUM_VOLTAGE);

    pMotor->pwmUpdateMode = PWW_CMP_CTR_BOTH;

    pMotor->ctrlStateCom = CTRL_STOP;
    pMotor->ctrlStateUse = CTRL_STOP;
    pMotor->ctrlStateFdb = CTRL_STOP;
    pMotor->lsw = ENC_ALIGNMENT;

    //
    // This function initializes the ADC PPB result bases, as well as the
    // ADC module used to sample phase W. Ensure that the final argument
    // passed corresponds to the ADC base used to sample phase W on the
    // HW board
    //
    FCL_initADC_2In(pMotor, M_IW_ADC_BASE,
                   M_IV_ADCRESULT_BASE, M_IV_ADC_PPB_NUM,
                   M_IW_ADCRESULT_BASE, M_IW_ADC_PPB_NUM);

    pMotor->volDC_PPBRESULT = M_VDC_ADCRESULT_BASE +
            ADC_PPBxRESULT_OFFSET_BASE + M_VDC_ADC_PPB_NUM * 2;

    //
    // ensure that the correct PWM base addresses are being passed to the
    // FCL library here. pwmHandle[0:2] should represent inverter phases
    // U/V/W in the hardware
    //
    FCL_initPWM(pMotor,
                obj->pwmHandle[0], obj->pwmHandle[1], obj->pwmHandle[2]);

    // ensure the correct QEP base is being passed
    FCL_initQEP(pMotor, obj->qepHandle);

    // comparator references
    // Set DAC-H to allowed maximum voltage
    CMPSS_setDACValueHigh(obj->cmpssHandle[3],
                          scaleVoltageValue(M_MAXIMUM_VOLTAGE));

    // Set DAC-L to allowed minimum voltage
    CMPSS_setDACValueLow(obj->cmpssHandle[3],
                          scaleVoltageValue(M_MINIMUM_VOLTAGE));

    return;
}

//
// initControlVars() function enter
// This function is called to reset the FCL variables and is useful when user
// wants to stop the motor and restart the motor
void resetControlVars(MOTOR_Vars_t *pMotor)
{
    pMotor->ctrlStateFdb = CTRL_STOP;
    pMotor->lsw = ENC_ALIGNMENT;
    pMotor->speedWePrev = 0.0;

    pMotor->cmplx_Id.carryOver = 0.0;
    pMotor->cmplx_Id.out = 0.0;
    pMotor->cmplx_Id.err = 0.0;
    pMotor->cmplx_Id.xErr = 0.0;

    pMotor->cmplx_Iq.carryOver = 0.0;
    pMotor->cmplx_Iq.out = 0.0;
    pMotor->cmplx_Iq.err = 0.0;
    pMotor->cmplx_Iq.xErr = 0.0;

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
    pMotor->offset_currentAs = 0.0;
    pMotor->offset_currentBs = 0.0;
    pMotor->offset_currentCs = 0.0;

    for(offsetCalCounter = 0; offsetCalCounter < 10000; offsetCalCounter++)
    {
        EPWM_clearEventTriggerInterruptFlag(pMotor->pwmBaseU);

        while(EPWM_getEventTriggerInterruptStatus(pMotor->pwmBaseU) == false);

        if(offsetCalCounter > 1000)
        {
            // Offsets in phase current sensing
            pMotor->offset_currentAs  = (K1 * pMotor->offset_currentAs) +
                    (((float32_t)(M_IFB_U)) * K2 * pMotor->adcScale);

            pMotor->offset_currentBs  = (K1 * pMotor->offset_currentBs) +
                    (((float32_t)(M_IFB_V)) * K2 * pMotor->adcScale);

            pMotor->offset_currentCs  = (K1 * pMotor->offset_currentCs) +
                    (((float32_t)(M_IFB_W)) * K2 * pMotor->adcScale);
        }
    }

    //
    // Read and update DC BUS voltage for FCL to use
    //
    pMotor->FCL_params.Vdcbus = getVdc(pMotor);

    // setting Iu offset
    ADC_setPPBReferenceOffset(M_IU_ADC_BASE, M_IU_ADC_PPB_NUM,
                         (uint16_t)(pMotor->offset_currentAs * ADC_RESOLUTION));
    // setting Iv offset
    ADC_setPPBReferenceOffset(M_IV_ADC_BASE, M_IV_ADC_PPB_NUM,
                         (uint16_t)(pMotor->offset_currentBs * ADC_RESOLUTION));

    // setting Iw offset
    ADC_setPPBReferenceOffset(M_IW_ADC_BASE, M_IW_ADC_PPB_NUM,
                         (uint16_t)(pMotor->offset_currentCs * ADC_RESOLUTION));

    // setting Vdc offset
    ADC_setPPBReferenceOffset(M_VDC_ADC_BASE, M_VDC_ADC_PPB_NUM, 0);



    pMotor->offsetDoneFlag = 1;

    return;
}

//
// End of Code
//

