//#############################################################################
//
// FILE:    fcl_cpu_code.c
//
// TITLE:   FCL motor handler functions from CPU
//
// Group:   C2000
//
// Target Family: F2837x/F2838x/F28004x
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

//
// includes
//

#include "fcl_cpu_cla.h"
#include "cpu_cla_shared.h"
#include "fcl_cla.h"


#ifndef __cplusplus
#pragma CODE_SECTION(FCL_runPICtrl,".TI.ramfunc");
#pragma CODE_SECTION(FCL_runSDFMPICtrl,".TI.ramfunc");
#pragma CODE_SECTION(FCL_runComplexCtrl,".TI.ramfunc");
#pragma CODE_SECTION(FCL_runSDFMComplexCtrl,".TI.ramfunc");
#pragma CODE_SECTION(FCL_runPICtrlWrap,".TI.ramfunc");
#pragma CODE_SECTION(FCL_runComplexCtrlWrap,".TI.ramfunc");

#pragma CODE_SECTION(FCL_runAbsEncPICtrl,".TI.ramfunc");
#pragma CODE_SECTION(FCL_runSDFMAbsEncPICtrl,".TI.ramfunc");
#pragma CODE_SECTION(FCL_runAbsEncComplexCtrl,".TI.ramfunc");
#pragma CODE_SECTION(FCL_runSDFMAbsEncComplexCtrl,".TI.ramfunc");
#pragma CODE_SECTION(FCL_runAbsEncPICtrlWrap,".TI.ramfunc");
#pragma CODE_SECTION(FCL_runAbsEncComplexCtrlWrap,".TI.ramfunc");

#pragma CODE_SECTION(FCL_runQEPWrap,".TI.ramfunc");
#pragma CODE_SECTION(FCL_resetController,".TI.ramfunc");
#endif

#if 0
typedef struct MotorVARS {
    float32_t   As;
    float32_t   Bs;
    float32_t   Alpha;
    float32_t   Beta;
    float32_t   Angle;
    float32_t   Cosine;
    float32_t   Sine;
    float32_t   Ds;
    float32_t   Qs;
} motorVars_t;

SVGEN svgen2;
uint16_t ppbCurA, ppbCurB;
uint32_t adcBaseCurA;
uint32_t adcBaseCurB;
#endif

CLARKE          clarke1;        // LEM/Shunt Current sensing
CLARKE          clarke2;        // SDFM Current sensing
float32_t       currentSenV;    //
float32_t       currentSenW;    //

PARK            park1;

SVGEN           svgen1;

ENC_Status_e    lsw;
#pragma DATA_SECTION(lsw, "ClaData");

float32_t       pangle;
#pragma DATA_SECTION(pangle, "ClaData");

QEP             qep1;
#pragma DATA_SECTION(qep1, "ClaData");

FCL_PIController_t pi_id;
FCL_PIController_t pi_iq;
#pragma DATA_SECTION(pi_iq, "ClaData");

RAMP_GEN_CLA    rg1;
#pragma DATA_SECTION(rg1, "ClaData");

FCL_Parameters_t FCL_params;

cmplxPars_t  D_cpu;

cmplxPars_t  Q_cla;
CLA_QEP_PTR  ClaQep;
#pragma DATA_SECTION(Q_cla, "ClaData");
#pragma DATA_SECTION(ClaQep, "ClaData");

float32_t speedWePrev;
float32_t speedWe;
#pragma DATA_SECTION(speedWe, "ClaData");

uint32_t curA_PPBRESULT, curB_PPBRESULT;
volatile union ADCINTFLG_REG *AdcIntFlag;

uint32_t * pwmCompA;
uint32_t * pwmCompB;
uint32_t * pwmCompC;

//
// function prototypes
//

#ifndef ONEbySQRT3
#define  ONEbySQRT3   (0.57735026918963)    /* 1/sqrt(3) */
#endif

//
// FCL internal function called in the complex control API
// This function implements the complex control algorithm
//
static inline void complexCtrl(void)
{
    SETGPIO18_HIGH;

    pi_id.out += D_cpu.kDirect * (Q_cla.idErr * D_cpu.cosWTs -
                                  Q_cla.iqErr * D_cpu.sinWTs) +
                 D_cpu.carryOver ;

    //
    // __fsat(pi_id.out, pi_id.Umax, pi_id.Umin);
    //
    CLAMP_MACRO(pi_id);

    SETGPIO18_LOW;

    return;
}

//
// FCL MACRO implementing CLARKE transform on currents using pointers to
// registers used in both PI CONTROL and COMPLEX CONTROL
//
#define FCL_CLARKE_ADC_IN()                                                    \
    clarke1Alpha = (float32_t)((int16_t)HWREGH(curA_PPBRESULT) *               \
                            FCL_params.adcScale);                              \
    clarke1Beta  = ((clarke1Alpha +                                            \
                     (2.0 * ((float32_t)((int16_t)HWREGH(curB_PPBRESULT)) *    \
                      FCL_params.adcScale))) * ONEbySQRT3);

//
// FCL MACRO implementing CLARKE transform on currents using pointers to
// registers used in both PI CONTROL and COMPLEX CONTROL
//
#define FCL_CLARKE_SDFM_IN()                                                   \
    clarke1Alpha = currentSenV;                                                \
    clarke1Beta  = ((clarke1Alpha + 2.0 * currentSenW) * ONEbySQRT3);

//
// FCL MACRO implementing PARK on position feed back and CLARKE on currents
// used in both PI CONTROL and COMPLEX CONTROL for QEP encoder
//

#define FCL_POSITION_CURRENT_CLARKE_MACRO()                                    \
/*                                                                             \
 *-----------------------------------------------------------------------------\
 *  Wait for QEP sense to complete (Position encoder suite module)             \
 * ----------------------------------------------------------------------------\
 */                                                                            \
    SETGPIO18_HIGH;                                                            \
    /* below line took 2 additional cycles vs bitfield style*/                 \
    /*while((HWREGH(PIECTRL_BASE + PIE_O_IFR11) & PIE_IFR11_INTX1) == false);*/\
    while(PieCtrlRegs.PIEIFR11.bit.INTx1 == 0);                                \
    SETGPIO18_LOW;                                                             \
/*                                                                             \
 *  ---------------------------------------------------------------------------\
 *  Connect inputs of the PARK module and call the park trans. macro           \
 *  ---------------------------------------------------------------------------\
 */                                                                            \
    park1Sine   = __sinpuf32(pangle);                                          \
    park1Cosine = __cospuf32(pangle);                                          \
/*                                                                             \
 * ----------------------------------------------------------------------------\
 * Measure phase currents, and normalize to (-1,+1).                           \
 * Connect inputs of the CLARKE module and call the clarke xform macro         \
 * ----------------------------------------------------------------------------\
 */                                                                            \
    SETGPIO18_HIGH;                                                            \
    /* below line took 2 additional cycles vs bitfield style*/                 \
    /*while((HWREGH(adcBasePhaseW) & ADC_INTFLG_ADCINT1) == false);*/          \
    while(!(AdcIntFlag->bit.ADCINT1));                                         \
    asm(" NOP");                                                               \
    SETGPIO18_LOW;                                                             \
    FCL_CLARKE_ADC_IN();

//
// FCL MACRO implementing PARK on position feed back and CLARKE on currents
// used in both PI CONTROL and COMPLEX CONTROL with SDFM for QEP encoder
//

#define FCL_POSITION_SDFM_CURRENT_CLARKE_MACRO()                               \
/*                                                                             \
 *-----------------------------------------------------------------------------\
 *  Wait for QEP sense to complete (Position encoder suite module)             \
 * ----------------------------------------------------------------------------\
 */                                                                            \
    SETGPIO18_HIGH;                                                            \
    /* CLA: below line took 2 additional cycles vs bitfield style*/            \
    /*while((HWREGH(PIECTRL_BASE + PIE_O_IFR11) & PIE_IFR11_INTX1) == false);*/\
    while(PieCtrlRegs.PIEIFR11.bit.INTx1 == 0);                                \
/*                                                                             \
 *  ---------------------------------------------------------------------------\
 *  Connect inputs of the PARK module and call the park trans. macro           \
 *  ---------------------------------------------------------------------------\
 */                                                                            \
    park1Sine   = __sinpuf32(pangle);                                          \
    park1Cosine = __cospuf32(pangle);                                          \
/*                                                                             \
 * ----------------------------------------------------------------------------\
 * Measure phase currents, and normalize to (-1,+1).                           \
 * Connect inputs of the CLARKE module and call the clarke xform macro         \
 * ----------------------------------------------------------------------------\
 */                                                                            \
    FCL_CLARKE_SDFM_IN();                                                      \
                                                                               \
    SETGPIO18_LOW;
//
// FCL MACRO implementing PARK on position feed back and CLARKE on currents
// used in both PI CONTROL and COMPLEX CONTROL for absolute encoder
//
#define FCL_ABS_ENC_POSITION_CURRENT_CLARKE_MACRO()                            \
/*                                                                             \
 *  ---------------------------------------------------------------------------\
 *  Connect inputs of the PARK module and call the park trans. macro           \
 *  ---------------------------------------------------------------------------\
 */                                                                            \
    park1Sine   = __sinpuf32(pangle);                                          \
    park1Cosine = __cospuf32(pangle);                                          \
/*                                                                             \
 * ----------------------------------------------------------------------------\
 * Measure phase currents, and normalize to (-1,+1).                           \
 * Connect inputs of the CLARKE module and call the clarke xform macro         \
 * ----------------------------------------------------------------------------\
 */                                                                            \
    SETGPIO18_HIGH;                                                            \
    /* below line took 2 additional cycles vs bitfield style*/                 \
    /* while((HWREGH(adcBasePhaseW) & ADC_INTFLG_ADCINT1) == false);*/         \
    while(!(AdcIntFlag->bit.ADCINT1));                                         \
    asm(" NOP");                                                               \
    SETGPIO18_LOW;                                                             \
    FCL_CLARKE_ADC_IN();

//
// FCL MACRO implementing PARK on position feed back and CLARKE on currents
// used in both PI CONTROL and COMPLEX CONTROL with SDFM for absolute encoder
//
#define FCL_ABS_ENC_POSITION_SDFM_CURRENT_CLARKE_MACRO()                       \
/*                                                                             \
*  ----------------------------------------------------------------------------\
*  Connect inputs of the PARK module and call the park trans. macro            \
*  ----------------------------------------------------------------------------\
*/                                                                             \
    SETGPIO18_HIGH;                                                            \
    park1Sine   = __sinpuf32(pangle);                                          \
    park1Cosine = __cospuf32(pangle);                                          \
/*                                                                             \
* -----------------------------------------------------------------------------\
* Measure phase currents, and normalize to (-1,+1).                            \
* Connect inputs of the CLARKE module and call the clarke xform macro          \
* -----------------------------------------------------------------------------\
*/                                                                             \
    FCL_CLARKE_SDFM_IN();                                                      \
                                                                               \
    SETGPIO18_LOW;
//
// FCL MACRO implementing PWM updates using pointers access to registers
// used in both PI CONTROL and COMPLEX CONTROL
//
#define FCL_PWM_UPDATE_STYLE_1()                                               \
    EPWM_setCounterCompareValue(pwmHandle[0], EPWM_COUNTER_COMPARE_A,          \
                                (uint32_t)(svgen2.Tc + svgen2.tmp1));          \
    EPWM_setCounterCompareValue(pwmHandle[1], EPWM_COUNTER_COMPARE_A,          \
                                (uint32_t)(svgen2.Ualpha + svgen2.tmp1));      \
    EPWM_setCounterCompareValue(pwmHandle[2], EPWM_COUNTER_COMPARE_A,          \
                                (uint32_t)(svgen2.Tb + svgen2.tmp1));

//
// FCL MACRO implementing PWM updates using direct address access to registers
// (or pointers) used in both PI CONTROL and COMPLEX CONTROL
// This macro saves about 40ns of cycle time
//
#define FCL_PWM_UPDATE_STYLE_2()                                               \
    *(volatile uint32_t *)(0x406A) = (uint32_t)(svgen2.Tc + svgen2.tmp1);      \
    *(volatile uint32_t *)(0x416A) = (uint32_t)(svgen2.Ualpha + svgen2.tmp1);  \
    *(volatile uint32_t *)(0x426A) = (uint32_t)(svgen2.Tb + svgen2.tmp1);

#define FCL_PWM_UPDATE_STYLE_3()                                               \
    *(pwmCompA) = (uint32_t)(svgen2.Tc + svgen2.tmp1);                         \
    *(pwmCompB) = (uint32_t)(svgen2.Ualpha + svgen2.tmp1);                     \
    *(pwmCompC) = (uint32_t)(svgen2.Tb + svgen2.tmp1);

//
// FCL MACRO implementing SVGEN and PWM updates
// used in both PI CONTROL and COMPLEX CONTROL
//
#define FCL_SVGEN_PWM_PDATE_MACRO()                                            \
/*                                                                             \
 * ----------------------------------------------------------------------------\
 * Call the space vector gen. macro                                            \
 * ----------------------------------------------------------------------------\
 */                                                                            \
    svgen2.Tb = (svgen2.Ubeta - svgen2.Ualpha) / 2;                            \
    svgen2.Tc = svgen2.Tb - svgen2.Ubeta;                                      \
                                                                               \
    svgen2.tmp2 = __fmax(__fmax(svgen2.Ualpha, svgen2.Tc), svgen2.Tb);         \
    svgen2.tmp2 += __fmin(__fmin(svgen2.Ualpha, svgen2.Tc), svgen2.Tb);        \
                                                                               \
    svgen2.tmp1  =  ((-svgen2.tmp2) / 2) + FCL_params.carrierMid;              \
                                                                               \
/*                                                                             \
 * ----------------------------------------------------------------------------\
 * Computed Duty and Write to CMPA register                                    \
 * ----------------------------------------------------------------------------\
 */                                                                            \
     FCL_PWM_UPDATE_STYLE_3();

//
// To do instrumentation from library use below
//
extern volatile uint16_t FCL_cycleCount;
#define READ_FCL_COUNT                                                         \
    FCL_cycleCount = HWREGH(EPWM1_BASE + EPWM_O_TBCTR);

//
// Function to initialize PWMs for the FCL operation, this will be called by the
// user application during the initialization or setup process
//
void FCL_initPWM(uint32_t baseA, uint32_t baseB, uint32_t baseC)
{
    EPWM_disableCounterCompareShadowLoadMode(baseA, EPWM_COUNTER_COMPARE_A);
    EPWM_disableCounterCompareShadowLoadMode(baseB, EPWM_COUNTER_COMPARE_A);
    EPWM_disableCounterCompareShadowLoadMode(baseC, EPWM_COUNTER_COMPARE_A);
    pwmCompA = (uint32_t *)(baseA + EPWM_O_CMPA);
    pwmCompB = (uint32_t *)(baseB + EPWM_O_CMPA);
    pwmCompC = (uint32_t *)(baseC + EPWM_O_CMPA);

    return;
}

void FCL_initADC(uint32_t resultBaseA, ADC_PPBNumber baseA_PPB,
                 uint32_t resultBaseB, ADC_PPBNumber baseB_PPB,
                 uint32_t basePhaseW)
{
    uint16_t ppbCurA, ppbCurB;

    ppbCurA = (uint16_t)(ADC_PPBxRESULT_OFFSET_BASE | (uint16_t)baseA_PPB * 2);
    ppbCurB = (uint16_t)(ADC_PPBxRESULT_OFFSET_BASE | (uint16_t)baseB_PPB * 2);
    curA_PPBRESULT = resultBaseA + ppbCurA;
    curB_PPBRESULT = resultBaseB + ppbCurB;
    AdcIntFlag = (union ADCINTFLG_REG *)(basePhaseW + ADC_O_INTFLG);

    return;
}

void FCL_initQEP(uint32_t baseA)
{
    ClaQep.ptr = (struct EQEP_REGS *)baseA;

    return;
}

//
// This function is called to reset the FCL variables and is useful when user
// wants to stop the motor and restart the motor
//
void FCL_resetController(void)
{
    pi_id.carryOver = pi_id.out = 0;
    pi_iq.carryOver = pi_iq.out = 0;
    D_cpu.carryOver = D_cpu.idErr = D_cpu.iqErr = 0;
    Q_cla.carryOver = Q_cla.idErr = Q_cla.iqErr = 0;

    speedWePrev = 0;
}

//
// Function that performs the PI Control as part of the Fast Current Loop
// for QEP encoder
//
void FCL_runPICtrl(void)
{
    register float32_t  clarke1Alpha, clarke1Beta;
    register float32_t  park1Sine, park1Cosine;
    register float32_t  piidc, piids;
    SVGEN               svgen2;

    //
    //  MACRO to :-
    //      1. read QEP position
    //      2. get current feed back
    //      3. do the Clarke
    //
    FCL_POSITION_CURRENT_CLARKE_MACRO();

//
    //  PARK Transformation
    //  Connect inputs of the PI module and
    // call the PI IQ controller macro in CLA
    //
    pi_iq.err = pi_iq.ref -
                ((clarke1Beta * park1Cosine) - (clarke1Alpha * park1Sine));
    pi_id.err = pi_id.ref -
                ((clarke1Alpha * park1Cosine) + (clarke1Beta * park1Sine));

    //
    //    Connect inputs of the PI module and call the PI ID controller macro
    //
    Cla1ForceTask2();

    FCL_PI_MACRO(pi_id)                        // Id loop - PI controller - CPU

    piidc = pi_id.out * park1Cosine;
    piids = pi_id.out * park1Sine;

    //
    //  Wait for PI IQ calc in CLA to complete
    //
    SETGPIO18_HIGH; // LOW
    while((HWREGH(PIECTRL_BASE + PIE_O_IFR11) & PIE_IFR11_INTX2) == false);
    SETGPIO18_LOW; // LOW

    //
    //  Perform the inverse park and connect inputs of the SVGEN_DQ module
    //
    svgen2.Ualpha = ( piidc - (pi_iq.out * park1Sine)) * FCL_params.carrierMid;
    svgen2.Ubeta  = ((pi_iq.out * park1Cosine) + piids) * FCL_params.cmidsqrt3;

    //
    //  MACRO to :-
    //      1. do SVGEN
    //      2. do PWMupdates
    //
    FCL_SVGEN_PWM_PDATE_MACRO();

    READ_FCL_COUNT;

    return;
}


//
// Function that performs the PI Control as part of the Fast Current Loop
// with SDFM sensing current for QEP encoder
//
void FCL_runSDFMPICtrl(void)
{
    register float32_t  clarke1Alpha, clarke1Beta;
    register float32_t  park1Sine, park1Cosine;
    register float32_t  piidc, piids;
    SVGEN               svgen2;

    //
    //  MACRO to :-
    //      1. read QEP position
    //      2. get current feed back
    //      3. do the Clarke
    //
    FCL_POSITION_SDFM_CURRENT_CLARKE_MACRO();

    //
    //  PARK Transformation
    //  Connect inputs of the PI module and
    // call the PI IQ controller macro in CLA
    //
    pi_iq.err = pi_iq.ref -
                ((clarke1Beta * park1Cosine) - (clarke1Alpha * park1Sine));
    pi_id.err = pi_id.ref -
                ((clarke1Alpha * park1Cosine) + (clarke1Beta * park1Sine));

    //
    //    Connect inputs of the PI module and call the PI ID controller macro
    //
    Cla1ForceTask2();

    FCL_PI_MACRO(pi_id)                        // Id loop - PI controller - CPU

    piidc = pi_id.out * park1Cosine;
    piids = pi_id.out * park1Sine;

    //
    //  Wait for PI IQ calc in CLA to complete
    //
    SETGPIO18_HIGH; // LOW
    while((HWREGH(PIECTRL_BASE + PIE_O_IFR11) & PIE_IFR11_INTX2) == false);
    SETGPIO18_LOW; // LOW

    //
    //  Perform the inverse park and connect inputs of the SVGEN_DQ module
    //
    svgen2.Ualpha = ( piidc - (pi_iq.out * park1Sine)) * FCL_params.carrierMid;
    svgen2.Ubeta  = ((pi_iq.out * park1Cosine) + piids) * FCL_params.cmidsqrt3;

    //
    //  MACRO to :-
    //      1. do SVGEN
    //      2. do PWMupdates
    //
    FCL_SVGEN_PWM_PDATE_MACRO();

    READ_FCL_COUNT;

    return;
}

//
// Function that performs the PI Control as part of the Fast Current Loop
// for absolute encoder
//
void FCL_runAbsEncPICtrl(void)
{
    register float32_t  clarke1Alpha, clarke1Beta;
    register float32_t  park1Sine, park1Cosine;
    register float32_t  piidc, piids;
    SVGEN               svgen2;

    //
    //  MACRO to :-
    //      1. read QEP position
    //      2. get current feed back
    //      3. do the Clarke
    //
    FCL_ABS_ENC_POSITION_CURRENT_CLARKE_MACRO();

    //
    //  PARK Transformation
    //  Connect inputs of the PI module and
    //  call the PI IQ controller macro in CLA
    //
    pi_iq.err = pi_iq.ref -
                ((clarke1Beta * park1Cosine) - (clarke1Alpha * park1Sine));
    pi_id.err = pi_id.ref -
                ((clarke1Alpha * park1Cosine) + (clarke1Beta * park1Sine));

    //
    //    Connect inputs of the PI module and call the PI ID controller macro
    //
    Cla1ForceTask2();
    FCL_PI_MACRO(pi_id)                        // Id loop - PI controller - CPU

    piidc = pi_id.out * park1Cosine;
    piids = pi_id.out * park1Sine;

    //
    //  Wait for PI IQ calc in CLA to complete
    //
    SETGPIO18_HIGH; // LOW
    while((HWREGH(PIECTRL_BASE + PIE_O_IFR11) & PIE_IFR11_INTX2) == false);
    SETGPIO18_LOW; // LOW

    //
    //  Perform the inverse park and connect inputs of the SVGEN_DQ module
    //
    svgen2.Ualpha = ( piidc - (pi_iq.out * park1Sine)) * FCL_params.carrierMid;
    svgen2.Ubeta  = ((pi_iq.out * park1Cosine) + piids) * FCL_params.cmidsqrt3;

    //
    //  MACRO to :-
    //      1. do SVGEN
    //      2. do PWMupdates
    //
    FCL_SVGEN_PWM_PDATE_MACRO();

    READ_FCL_COUNT;

    return;
}


//
// Function that performs the PI Control as part of the Fast Current Loop
// with SDFM snesing current for absolute encoder
//
void FCL_runSDFMAbsEncPICtrl(void)
{
    register float32_t  clarke1Alpha, clarke1Beta;
    register float32_t  park1Sine, park1Cosine;
    register float32_t  piidc, piids;
    SVGEN               svgen2;

    //
    //  MACRO to :-
    //      1. read QEP position
    //      2. get current feed back
    //      3. do the Clarke
    //
    FCL_ABS_ENC_POSITION_SDFM_CURRENT_CLARKE_MACRO();

    //
    //  PARK Transformation
    //  Connect inputs of the PI module and
    //  call the PI IQ controller macro in CLA
    //
    pi_iq.err = pi_iq.ref -
                ((clarke1Beta * park1Cosine) - (clarke1Alpha * park1Sine));
    pi_id.err = pi_id.ref -
                ((clarke1Alpha * park1Cosine) + (clarke1Beta * park1Sine));

    //
    //    Connect inputs of the PI module and call the PI ID controller macro
    //
    Cla1ForceTask2();
    FCL_PI_MACRO(pi_id)                        // Id loop - PI controller - CPU

    piidc = pi_id.out * park1Cosine;
    piids = pi_id.out * park1Sine;

    //
    //  Wait for PI IQ calc in CLA to complete
    //
    SETGPIO18_HIGH; // LOW
    while((HWREGH(PIECTRL_BASE + PIE_O_IFR11) & PIE_IFR11_INTX2) == false);
    SETGPIO18_LOW; // LOW

    //
    //  Perform the inverse park and connect inputs of the SVGEN_DQ module
    //
    svgen2.Ualpha = ( piidc - (pi_iq.out * park1Sine)) * FCL_params.carrierMid;
    svgen2.Ubeta  = ((pi_iq.out * park1Cosine) + piids) * FCL_params.cmidsqrt3;

    //
    //  MACRO to :-
    //      1. do SVGEN
    //      2. do PWMupdates
    //
    FCL_SVGEN_PWM_PDATE_MACRO();

    READ_FCL_COUNT;

    return;
}

//
// Wrap up function to be called by the user application at the completion of
// Fast Current Loop in PI Control Mode for QEP encoder
//
void FCL_runPICtrlWrap(void)
{
    float32_t Vbase = FCL_params.Vdcbus * 1.15 / 2;
    float32_t invZbase = FCL_params.Ibase / Vbase;

    //
    // To save CPU cycles and speed up calcn, carry over math is done within
    //   this wrap function and .CarryOver carries it to the next iteration
    // Bemf calc is rolled in to the Q calcs as the speed and flux does not
    //   change much between iterations - equation tweaked to fit here
    //

    //
    // CLA_forceTasks(CLA1_BASE, CLA_TASKFLAG_4);
    //
    Cla1ForceTask4();

    //
    // Update PI ID parameters
    //
    pi_id.Kp = FCL_params.Ld * invZbase * FCL_params.wccD;
    pi_id.Ki = FCL_params.Rd * invZbase * FCL_params.wccD * FCL_params.tSamp;

    pi_id.Kerr      = (pi_id.Ki * 0.5F) + pi_id.Kp,
    pi_id.KerrOld   = (pi_id.Ki * 0.5F) - pi_id.Kp;
    pi_id.carryOver = (pi_id.err * pi_id.KerrOld);

    //
    // Update PI IQ parameters
    //
    pi_iq.Kp = FCL_params.Lq * invZbase * FCL_params.wccQ;
    pi_iq.Ki = FCL_params.Rq * invZbase * FCL_params.wccQ * FCL_params.tSamp;

    pi_iq.Kerr      = (pi_iq.Ki * 0.5) + pi_iq.Kp,
    pi_iq.KerrOld   = (pi_iq.Ki * 0.5) - pi_iq.Kp;
    pi_iq.carryOver = (pi_iq.err * pi_iq.KerrOld) +
                      FCL_params.BemfK * (speed1.Speed - speedWePrev);
    speedWePrev     = speed1.Speed;

    //
    // to pass on the id and iq current feedback back to user,
    // update them in the wrap function
    //
    pi_id.fbk =  pi_id.ref - pi_id.err;
    pi_iq.fbk =  pi_iq.ref - pi_iq.err;

    //
    //give enough time before clearing INTx4 for CLA TASK 4 completion
    //
    while((HWREGH(PIECTRL_BASE + PIE_O_IFR11) & PIE_IFR11_INTX4) == false);

    //
    // clear CLA task flags 1, 2, 4
    //
    HWREGH(PIECTRL_BASE + PIE_O_IFR11) &= ~(PIE_IFR11_INTX1 |
                                            PIE_IFR11_INTX2 |
                                            PIE_IFR11_INTX4 );
}

//
// Wrap up function to be called by the user application at the completion of
// Fast Current Loop in PI Control Mode for absolute encoder
//
void FCL_runAbsEncPICtrlWrap(void)
{
    float32_t Vbase = FCL_params.Vdcbus * (1.15 / 2);
    float32_t invZbase = FCL_params.Ibase / Vbase;

    //
    // To save CPU cycles and speed up calcn, carry over math is done within
    //   this wrap function and .CarryOver carries it to the next iteration
    // Bemf calc is rolled in to the Q calcs as the speed and flux does not
    //   change much between iterations - equation tweaked to fit here
    //

    //
    // Update PI ID parameters
    //
    pi_id.Kp = FCL_params.Ld * invZbase * FCL_params.wccD;
    pi_id.Ki = FCL_params.Rd * invZbase * FCL_params.wccD * FCL_params.tSamp;

    pi_id.Kerr      = (pi_id.Ki * 0.5F) + pi_id.Kp,
    pi_id.KerrOld   = (pi_id.Ki * 0.5F) - pi_id.Kp;
    pi_id.carryOver = (pi_id.err * pi_id.KerrOld);

    //
    // Update PI IQ parameters
    //
    pi_iq.Kp = FCL_params.Lq * invZbase * FCL_params.wccQ;
    pi_iq.Ki = FCL_params.Rq * invZbase * FCL_params.wccQ * FCL_params.tSamp;

    pi_iq.Kerr      = (pi_iq.Ki * 0.5) + pi_iq.Kp,
    pi_iq.KerrOld   = (pi_iq.Ki * 0.5) - pi_iq.Kp;
    pi_iq.carryOver = (pi_iq.err * pi_iq.KerrOld) +
                      FCL_params.BemfK * (speedWe - speedWePrev);
    speedWePrev     = speedWe;

    //
    // to pass on the id and iq current feedback back to user,
    // update them in the wrap function
    //
    pi_id.fbk =  pi_id.ref - pi_id.err;
    pi_iq.fbk =  pi_iq.ref - pi_iq.err;

    //
    // clear CLA task flags 2
    //
    HWREGH(PIECTRL_BASE + PIE_O_IFR11) &= ~PIE_IFR11_INTX2;
}

//
// This function returns a 32-bit constant and for this version the value
// returned is 0x00000006
// Ver   Date       Platform              Description   Location
//  1   07/2016  IDDK / TMDSCNCDF28379D   Beta MSS      \FCL\v01_00_00_00
//  2   03/2017  IDDK / TMDSCNCDF28379D   cSUITE GA     \FCL\v02_00_00_00
//  3   11/2017  F28379DXL/ GaN/ DRV8305  Beta MSS      \FCL\v03_00_00_02
//  4   03/2018  F28379DXL/ GaN/ DRV8305  cSUITE GA     \FCL_SFRA\v01_00_00_00
//  5   03/2019  IDDK / TMDSCNCDF28379D   MCSDK_1
//  6   06/2019  IDDK / TMDSCNCDF28379D   MCSDK_2
//  7   09/2019  IDDK / TMDSCNCDF28379D   MCSDK_2.1
//
uint32_t FCL_getSwVersion(void)
{
    //
    // for this version return 0x00000007
    //
    return((uint32_t)(0x00000007));
}

//
// Function to be called by the user application after the completion of
// Fast current Loop to wrap the QEP feedback completion.
// This is used only in FCL_LEVE2
//
void FCL_runQEPWrap(void)
{
    //
    // poll INT11.1 IFR flag
    //
    while((HWREGH(PIECTRL_BASE + PIE_O_IFR11) & PIE_IFR11_INTX1) == false);

    //
    // force CLA task 4
    //
    CLA_forceTasks(CLA1_BASE, CLA_TASKFLAG_4);

    //
    // poll INT11.4 IFR flag
    //
    while((HWREGH(PIECTRL_BASE + PIE_O_IFR11) & PIE_IFR11_INTX4) == false);

    //
    // clear INT11.1 and 11.4 IFR flags
    //
    HWREGH(PIECTRL_BASE + PIE_O_IFR11) &= ~(PIE_IFR11_INTX1 |
                                            PIE_IFR11_INTX4);

    //
    // acknowledge PIE group 11 interrupt
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP11);
}


//
// Function that performs the Complex control as part of the Fast Current Loop
// for QEP encoder
//
void FCL_runComplexCtrl(void)
{
    register float32_t  clarke1Alpha, clarke1Beta;
    register float32_t  park1Sine, park1Cosine;
    register float32_t  piidc, piids;
    SVGEN               svgen2;

    //
    //  MACRO to :-
    //      1. read QEP position
    //      2. get current feed back
    //      3. do the Clarke
    //
    FCL_POSITION_CURRENT_CLARKE_MACRO();

    //
    //    PARK Transformation
    //    Connect inputs of the CMPLX module and call the CMPLX controller
    //
    Q_cla.iqErr = pi_iq.ref -
                  ((clarke1Beta * park1Cosine) - (clarke1Alpha * park1Sine));
    Q_cla.idErr = pi_id.ref -
                  ((clarke1Alpha * park1Cosine) + (clarke1Beta * park1Sine));

    SETGPIO18_HIGH;
    Cla1ForceTask3();               // Iq loop - complex control - CLA
    SETGPIO18_LOW;
    complexCtrl();                  // Id loop - complex control - CPU

    piidc = pi_id.out * park1Cosine;
    piids = pi_id.out * park1Sine;

    //
    //  Wait for CMPLX IQ calc in CLA to complete
    //
    SETGPIO18_HIGH;
    while((HWREGH(PIECTRL_BASE + PIE_O_IFR11) & PIE_IFR11_INTX3) == false);
    SETGPIO18_LOW;

    //
    //  Perform the inverse park and connect inputs of the SVGEN_DQ module
    //
    svgen2.Ualpha = ( piidc - (pi_iq.out * park1Sine)) * FCL_params.carrierMid;
    svgen2.Ubeta  = ((pi_iq.out * park1Cosine) + piids) * FCL_params.cmidsqrt3;

    //
    //  MACRO to :-
    //      1. do SVGEN
    //      2. do PWMupdates
    //
    FCL_SVGEN_PWM_PDATE_MACRO();
    READ_FCL_COUNT;
    return;
}


//
// Function that performs the Complex control as part of the Fast Current Loop
// with SDFM sensing current for QEP encoder
//
void FCL_runSDFMComplexCtrl(void)
{
    register float32_t  clarke1Alpha, clarke1Beta;
    register float32_t  park1Sine, park1Cosine;
    register float32_t  piidc, piids;
    SVGEN               svgen2;

    //
    //  MACRO to :-
    //      1. read QEP position
    //      2. get current feed back
    //      3. do the Clarke
    //
    FCL_POSITION_SDFM_CURRENT_CLARKE_MACRO();

    //
    //    PARK Transformation
    //    Connect inputs of the CMPLX module and call the CMPLX controller
    //
    Q_cla.iqErr = pi_iq.ref -
                  ((clarke1Beta * park1Cosine) - (clarke1Alpha * park1Sine));
    Q_cla.idErr = pi_id.ref -
                  ((clarke1Alpha * park1Cosine) + (clarke1Beta * park1Sine));

    SETGPIO18_HIGH;
    Cla1ForceTask3();               // Iq loop - complex control - CLA
    SETGPIO18_LOW;
    complexCtrl();                  // Id loop - complex control - CPU

    piidc = pi_id.out * park1Cosine;
    piids = pi_id.out * park1Sine;

    //
    //  Wait for CMPLX IQ calc in CLA to complete
    //
    SETGPIO18_HIGH;
    while((HWREGH(PIECTRL_BASE + PIE_O_IFR11) & PIE_IFR11_INTX3) == false);
    SETGPIO18_LOW;

    //
    //  Perform the inverse park and connect inputs of the SVGEN_DQ module
    //
    svgen2.Ualpha = ( piidc - (pi_iq.out * park1Sine)) * FCL_params.carrierMid;
    svgen2.Ubeta  = ((pi_iq.out * park1Cosine) + piids) * FCL_params.cmidsqrt3;

    //
    //  MACRO to :-
    //      1. do SVGEN
    //      2. do PWMupdates
    //
    FCL_SVGEN_PWM_PDATE_MACRO();
    READ_FCL_COUNT;
    return;
}

//
// Function that performs the Complex control as part of the Fast Current Loop
// for absolute encoder
//
void FCL_runAbsEncComplexCtrl(void)
{
    register float32_t  clarke1Alpha, clarke1Beta;
    register float32_t  park1Sine, park1Cosine;
    register float32_t  piidc, piids;
    SVGEN               svgen2;

    //
    //  MACRO to :-
    //      1. read QEP position
    //      2. get current feed back
    //      3. do the Clarke
    //
    FCL_ABS_ENC_POSITION_CURRENT_CLARKE_MACRO();

    //
    //    PARK Transformation
    //    Connect inputs of the CMPLX module and call the CMPLX controller
    //
    Q_cla.iqErr = pi_iq.ref -
                  ((clarke1Beta * park1Cosine) - (clarke1Alpha * park1Sine));
    Q_cla.idErr = pi_id.ref -
                  ((clarke1Alpha * park1Cosine) + (clarke1Beta * park1Sine));

    SETGPIO18_HIGH;
    Cla1ForceTask3();               // Iq loop - complex control - CLA
    SETGPIO18_LOW;
    complexCtrl();                  // Id loop - complex control - CPU

    piidc = pi_id.out * park1Cosine;
    piids = pi_id.out * park1Sine;

    //
    //  Wait for CMPLX IQ calc in CLA to complete
    //
    SETGPIO18_HIGH;
    while((HWREGH(PIECTRL_BASE + PIE_O_IFR11) & PIE_IFR11_INTX3) == false);
    SETGPIO18_LOW;

    //
    //  Perform the inverse park and connect inputs of the SVGEN_DQ module
    //
    svgen2.Ualpha = ( piidc - (pi_iq.out * park1Sine)) * FCL_params.carrierMid;
    svgen2.Ubeta  = ((pi_iq.out * park1Cosine) + piids) * FCL_params.cmidsqrt3;

    //
    //  MACRO to :-
    //      1. do SVGEN
    //      2. do PWMupdates
    //
    FCL_SVGEN_PWM_PDATE_MACRO();
    READ_FCL_COUNT;
    return;
}

//
// Function that performs the Complex control as part of the Fast Current Loop
// with SDFM sensing current for absolute encoder
//
void FCL_runSDFMAbsEncComplexCtrl(void)
{
    register float32_t  clarke1Alpha, clarke1Beta;
    register float32_t  park1Sine, park1Cosine;
    register float32_t  piidc, piids;
    SVGEN               svgen2;

    //
    //  MACRO to :-
    //      1. read QEP position
    //      2. get current feed back
    //      3. do the Clarke
    //
    FCL_ABS_ENC_POSITION_SDFM_CURRENT_CLARKE_MACRO();

    //
    //    PARK Transformation
    //    Connect inputs of the CMPLX module and call the CMPLX controller
    //
    Q_cla.iqErr = pi_iq.ref -
                  ((clarke1Beta * park1Cosine) - (clarke1Alpha * park1Sine));
    Q_cla.idErr = pi_id.ref -
                  ((clarke1Alpha * park1Cosine) + (clarke1Beta * park1Sine));

    SETGPIO18_HIGH;
    Cla1ForceTask3();               // Iq loop - complex control - CLA
    SETGPIO18_LOW;
    complexCtrl();                  // Id loop - complex control - CPU

    piidc = pi_id.out * park1Cosine;
    piids = pi_id.out * park1Sine;

    //
    //  Wait for CMPLX IQ calc in CLA to complete
    //
    SETGPIO18_HIGH;
    while((HWREGH(PIECTRL_BASE + PIE_O_IFR11) & PIE_IFR11_INTX3) == false);
    SETGPIO18_LOW;

    //
    //  Perform the inverse park and connect inputs of the SVGEN_DQ module
    //
    svgen2.Ualpha = ( piidc - (pi_iq.out * park1Sine)) * FCL_params.carrierMid;
    svgen2.Ubeta  = ((pi_iq.out * park1Cosine) + piids) * FCL_params.cmidsqrt3;

    //
    //  MACRO to :-
    //      1. do SVGEN
    //      2. do PWMupdates
    //
    FCL_SVGEN_PWM_PDATE_MACRO();
    READ_FCL_COUNT;
    return;
}

//
// Wrap up function to be called by the user application at the completion of
// Fast Current Loop in Complex Control Mode for QEP encoder
//
void FCL_runComplexCtrlWrap (void)
{
    float32_t Vbase    = FCL_params.Vdcbus * 1.15 / 2;
    float32_t invZbase = FCL_params.Ibase / Vbase;
    float32_t WTs      = speed1.Speed * FCL_params.Wbase * FCL_params.tSamp;

    //
    // CLA_forceTasks(CLA1_BASE, CLA_TASKFLAG_4);
    //
    Cla1ForceTask4();

    D_cpu.cosWTs  = Q_cla.cosWTs = __cos(WTs);
    D_cpu.sinWTs  = Q_cla.sinWTs = __sin(WTs);

    D_cpu.expVal  = expf(-(FCL_params.Rd / FCL_params.Ld) * FCL_params.tSamp);
    Q_cla.expVal  = expf(-(FCL_params.Rq / FCL_params.Lq) * FCL_params.tSamp);

    D_cpu.kDirect = FCL_params.Rd * invZbase *
             __sqrt(2.0F - 2.0F * __cos(FCL_params.wccD * FCL_params.tSamp)) /
                (1.0F - D_cpu.expVal);
    Q_cla.kDirect = FCL_params.Rq * invZbase *
             __sqrt(2.0F - 2.0F * __cos(FCL_params.wccQ * FCL_params.tSamp)) /
             (1.0F - Q_cla.expVal);

    //
    // To save CPU cycles and speed up calcn, carry over math is done within
    //    this wrap function and 'carryOver' carries it to the next iteration
    // Bemf calc is rolled in to the Q calcs as the speed and flux does not
    //    change much between iterations - equation tweaked to fit here
    //
    D_cpu.carryOver = (-D_cpu.kDirect * Q_cla.idErr * D_cpu.expVal);
    Q_cla.carryOver = (-Q_cla.kDirect * Q_cla.iqErr * Q_cla.expVal) +
                      (FCL_params.BemfK * (speed1.Speed - speedWePrev));

    speedWePrev     = speed1.Speed;

    //
    // to pass on the id and iq current feedback back to user,
    // update them in the wrap function
    //
    pi_id.fbk = pi_id.ref - Q_cla.idErr;
    pi_iq.fbk = pi_iq.ref - Q_cla.iqErr;

    //
    // clear CLA task flags 1-3
    //
    HWREGH(PIECTRL_BASE + PIE_O_IFR11) &= ~(PIE_IFR11_INTX1 |
                                            PIE_IFR11_INTX3);

    //
    //give enough time before clearing INTx4 for CLA TASK 4 completion
    //
    while((HWREGH(PIECTRL_BASE + PIE_O_IFR11) & PIE_IFR11_INTX4) == false);
    HWREGH(PIECTRL_BASE + PIE_O_IFR11) &= ~PIE_IFR11_INTX4;

    return;
}

//
// Wrap up function to be called by the user application at the completion of
// Fast Current Loop in Complex Control Mode for absolute encoder
//
void FCL_runAbsEncComplexCtrlWrap (void)
{
    float32_t Vbase    = FCL_params.Vdcbus * 1.15 / 2;
    float32_t invZbase = FCL_params.Ibase / Vbase;
    float32_t WTs      = speedWe * FCL_params.Wbase * FCL_params.tSamp;

    D_cpu.cosWTs  = Q_cla.cosWTs = __cos(WTs);
    D_cpu.sinWTs  = Q_cla.sinWTs = __sin(WTs);

    D_cpu.expVal  = expf(-(FCL_params.Rd / FCL_params.Ld) * FCL_params.tSamp);
    Q_cla.expVal  = expf(-(FCL_params.Rq / FCL_params.Lq) * FCL_params.tSamp);

    D_cpu.kDirect = FCL_params.Rd * invZbase *
              __sqrt(2.0F - 2.0F * __cos(FCL_params.wccD * FCL_params.tSamp)) /
               (1.0F - D_cpu.expVal);
    Q_cla.kDirect = FCL_params.Rq * invZbase *
              __sqrt(2.0F - 2.0F * __cos(FCL_params.wccQ * FCL_params.tSamp)) /
              (1.0F - Q_cla.expVal);

    //
    // To save CPU cycles and speed up calcn, carry over math is done within
    //    this wrap function and 'carryOver' carries it to the next iteration
    // Bemf calc is rolled in to the Q calcs as the speed and flux does not
    //    change much between iterations - equation tweaked to fit here
    //
    D_cpu.carryOver = (-D_cpu.kDirect * Q_cla.idErr * D_cpu.expVal);
    Q_cla.carryOver = (-Q_cla.kDirect * Q_cla.iqErr * Q_cla.expVal) +
                      (FCL_params.BemfK * (speedWe - speedWePrev));

    speedWePrev     = speedWe;

    //
    // to pass on the id and iq current feedback back to user,
    // update them in the wrap function
    //
    pi_id.fbk = pi_id.ref - Q_cla.idErr;
    pi_iq.fbk = pi_iq.ref - Q_cla.iqErr;

    //
    // clear CLA task flags 3
    //
    HWREGH(PIECTRL_BASE + PIE_O_IFR11) &= ~PIE_IFR11_INTX3;

    return;
}

//
// end of file
//
