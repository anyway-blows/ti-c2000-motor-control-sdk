//#############################################################################
// FILE:   fcl_cla_dm.h
// TITLE:  Header file to be shared between example and library for CLA data.
//
// Group:          C2000
// Target Family:  F2837x/F2838x/F28004x
//
//#############################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:25 CDT 2021 $
// $Copyright: Copyright (C) 2013-2017 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//#############################################################################

#ifndef FCL_CLA_DM_H
#define FCL_CLA_DM_H

//
// includes
//

#ifndef F2838x_DEVICE
#include "F28x_Project.h"
#else
#include "f28x_project.h"
#endif

#include "qep_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

//
// defines
//

//
// typedefs
//


//
// globals
//

//
// function prototypes
//
extern uint32_t FCL_getSwVersion(void);

//
// Function to be called by the user application after the completion of
// Fast current Loop to wrap the QEP feedback completion.
// This is used only in FCL_LEVE2
//
static inline void FCL_runQEPWrap_M1(void)
{
    //
    // poll INT11.1 IFR flag, CLA1_1
    //
    while((HWREGH(PIECTRL_BASE + PIE_O_IFR11) & PIE_IFR11_INTX1) == false);

    //
    // force CLA task 4
    // CLA_forceTasks(CLA1_BASE, CLA_TASKFLAG_4);
    //
    Cla1ForceTask4();

    //
    // poll INT11.4 IFR flag, CLA1_4
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

static inline void FCL_runQEPWrap_M2(void)
{
    //
    // poll INT11.1 IFR flag, CLA1_1
    //
    while((HWREGH(PIECTRL_BASE + PIE_O_IFR11) & PIE_IFR11_INTX5) == false);

    //
    // force CLA task 8
    // CLA_forceTasks(CLA1_BASE, CLA_TASKFLAG_8);
    //
    Cla1ForceTask8();

    //
    // poll INT11.4 IFR flag, CLA1_4
    //
    while((HWREGH(PIECTRL_BASE + PIE_O_IFR11) & PIE_IFR11_INTX8) == false);

    //
    // clear INT11.1 and 11.4 IFR flags
    //
    HWREGH(PIECTRL_BASE + PIE_O_IFR11) &= ~(PIE_IFR11_INTX5 |
                                            PIE_IFR11_INTX8);
    //
    // acknowledge PIE group 11 interrupt
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP11);
}

#ifdef __cplusplus
}
#endif // extern "C"

#endif // end of FCL_CLA_DM_H definition
