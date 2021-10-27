//#############################################################################
//
// FILE:   pto_qepdiv.h
//
// TITLE:  Prototypes and Definitions for the Position Manager PTO
//         QEPDiv Library
//
//
//#############################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:27 CDT 2021 $
// $Copyright:
// Copyright (C) 2017-2020 Texas Instruments Incorporated
//
//     http://www.ti.com/ ALL RIGHTS RESERVED
// $
//#############################################################################

//
// Library of functions
//
#include <stdint.h>

//
// Function Prototypes
//
extern void pto_qepdiv_setupPeriph(void);
extern uint16_t pto_qepdiv_config(uint16_t divider, uint16_t indexWidth);
extern void pto_qepdiv_startOperation(uint16_t run);
extern void pto_qepdiv_reset(void);
extern void pto_qepdiv_initCLBXBAR(void);
extern void pto_qepdiv_resetCLB(void);

//
// End of File
//
