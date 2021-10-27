//#############################################################################
//
// FILE:    fcl_foc_cpu.c
//
// TITLE:   FCL motor handler functions from CPU
//
// Group:   C2000
//
// Target Family: F2837x/F2838x/F28004x/F28002x
//
//#############################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:25 CDT 2021 $
// $Copyright:
// Copyright (C) 2017-2021 Texas Instruments Incorporated
//
//     http://www.ti.com/ ALL RIGHTS RESERVED
// $
//#############################################################################


#include "fcl_foc_cpu.h"


//*****************************************************************************
// Revision status  - this version returns 0x00000101
uint32_t FCL_getSwVersion(void)
{
    //for this version return 0x00000101, CPU Version
    return (uint32_t)(0x00000201);
}

//
// initialize ADC for 2 current sensors
//
void FCL_initADC_2In(MOTOR_Vars_t *pMotor, const uint32_t adcBaseW,
                     const uint32_t adcResultBaseV, ADC_PPBNumber adcV_PPBNum,
                     const uint32_t adcResultBaseW, ADC_PPBNumber adcW_PPBNum )
{
    uint16_t adcPPBCurV, adcPPBCurW;

    pMotor->adcBaseW = adcBaseW;

    adcPPBCurV = (uint16_t)(ADC_PPBxRESULT_OFFSET_BASE +
                            (uint16_t)adcV_PPBNum * 2);
    adcPPBCurW = (uint16_t)(ADC_PPBxRESULT_OFFSET_BASE +
                            (uint16_t)adcW_PPBNum * 2);

    pMotor->curA_PPBRESULT = adcResultBaseV + adcPPBCurV;
    pMotor->curB_PPBRESULT = adcResultBaseW + adcPPBCurW;

    pMotor->pADCIntFlag = (union ADCINTFLG_REG *)(adcBaseW + ADC_O_INTFLG);

    return;
}

//
// initialize ADC for 3 current sensors
//
void FCL_initADC_3In(MOTOR_Vars_t *pMotor, const uint32_t adcBaseW,
                     const uint32_t adcResultBaseU, ADC_PPBNumber adcU_PPBNum,
                     const uint32_t adcResultBaseV, ADC_PPBNumber adcV_PPBNum,
                     const uint32_t adcResultBaseW, ADC_PPBNumber adcW_PPBNum )
{
    uint16_t adcPPBCurU, adcPPBCurV, adcPPBCurW;

    adcPPBCurU = (uint16_t)(ADC_PPBxRESULT_OFFSET_BASE +
                            (uint16_t)adcU_PPBNum * 2);
    adcPPBCurV = (uint16_t)(ADC_PPBxRESULT_OFFSET_BASE +
                            (uint16_t)adcV_PPBNum * 2);
    adcPPBCurW = (uint16_t)(ADC_PPBxRESULT_OFFSET_BASE +
                            (uint16_t)adcW_PPBNum * 2);

    pMotor->curA_PPBRESULT = adcResultBaseU + adcPPBCurU;
    pMotor->curB_PPBRESULT = adcResultBaseV + adcPPBCurV;
    pMotor->curC_PPBRESULT = adcResultBaseW + adcPPBCurW;

    pMotor->pADCIntFlag = (union ADCINTFLG_REG *)(adcBaseW + ADC_O_INTFLG);

    return;
}

//
//  Function to initialize PWMs for the FCL operation, this will be called by
//  the user application during the initialization or setup process
//
void FCL_initPWM(MOTOR_Vars_t *ptrMotor,
                 const uint32_t pwmBaseU,
                 const uint32_t pwmBaseV,
                 const uint32_t pwmBaseW)
{
    ptrMotor->pwmBaseU = pwmBaseU;
    ptrMotor->pwmBaseV = pwmBaseV;
    ptrMotor->pwmBaseW = pwmBaseW;

    switch(ptrMotor->pwmUpdateMode)
    {
        case PWW_CMP_CTR_ZERO:
            EPWM_setCounterCompareShadowLoadMode(ptrMotor->pwmBaseU,
                                            EPWM_COUNTER_COMPARE_A,
                                            EPWM_COMP_LOAD_ON_CNTR_ZERO);
            EPWM_setCounterCompareShadowLoadMode(ptrMotor->pwmBaseV,
                                            EPWM_COUNTER_COMPARE_A,
                                            EPWM_COMP_LOAD_ON_CNTR_ZERO);
            EPWM_setCounterCompareShadowLoadMode(ptrMotor->pwmBaseW,
                                            EPWM_COUNTER_COMPARE_A,
                                            EPWM_COMP_LOAD_ON_CNTR_ZERO);
            break;
        case PWW_CMP_CTR_PRD:
            EPWM_setCounterCompareShadowLoadMode(ptrMotor->pwmBaseU,
                                            EPWM_COUNTER_COMPARE_A,
                                            EPWM_COMP_LOAD_ON_CNTR_PERIOD);
            EPWM_setCounterCompareShadowLoadMode(ptrMotor->pwmBaseV,
                                            EPWM_COUNTER_COMPARE_A,
                                            EPWM_COMP_LOAD_ON_CNTR_PERIOD);
            EPWM_setCounterCompareShadowLoadMode(ptrMotor->pwmBaseW,
                                            EPWM_COUNTER_COMPARE_A,
                                            EPWM_COMP_LOAD_ON_CNTR_PERIOD);
            break;
        case PWW_CMP_CTR_BOTH:
            EPWM_setCounterCompareShadowLoadMode(ptrMotor->pwmBaseU,
                                            EPWM_COUNTER_COMPARE_A,
                                            EPWM_COMP_LOAD_ON_CNTR_ZERO_PERIOD);
            EPWM_setCounterCompareShadowLoadMode(ptrMotor->pwmBaseV,
                                            EPWM_COUNTER_COMPARE_A,
                                            EPWM_COMP_LOAD_ON_CNTR_ZERO_PERIOD);
            EPWM_setCounterCompareShadowLoadMode(ptrMotor->pwmBaseW,
                                            EPWM_COUNTER_COMPARE_A,
                                            EPWM_COMP_LOAD_ON_CNTR_ZERO_PERIOD);
            break;
        case PWW_CMP_IMMEDIATE:
            EPWM_disableCounterCompareShadowLoadMode(ptrMotor->pwmBaseU,
                                            EPWM_COUNTER_COMPARE_A);
            EPWM_disableCounterCompareShadowLoadMode(ptrMotor->pwmBaseV,
                                            EPWM_COUNTER_COMPARE_A);
            EPWM_disableCounterCompareShadowLoadMode(ptrMotor->pwmBaseW,
                                            EPWM_COUNTER_COMPARE_A);
            break;
    }

    ptrMotor->pwmCompA = (uint32_t *)(ptrMotor->pwmBaseU + EPWM_O_CMPA);
    ptrMotor->pwmCompB = (uint32_t *)(ptrMotor->pwmBaseV + EPWM_O_CMPA);
    ptrMotor->pwmCompC = (uint32_t *)(ptrMotor->pwmBaseW + EPWM_O_CMPA);

    return;
}

//
// initialize QEP
//
void FCL_initQEP(MOTOR_Vars_t *pMotor, const uint32_t qepBaseA)
{
    pMotor->pQEPRegs = (struct EQEP_REGS *)qepBaseA;

    return;
}

//
// End of the file
//
