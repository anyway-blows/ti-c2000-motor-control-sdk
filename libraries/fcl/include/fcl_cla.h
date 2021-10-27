//#############################################################################
//
// FILE:    fcl_cla.h
//
// TITLE:   Header file to be shared between example and library for CLA data.
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

#ifndef FCL_CLA_H
#define FCL_CLA_H

//
// includes
//
#ifndef F2838x_DEVICE
#include "F28x_Project.h"
#else
#include "f28x_project.h"
#endif

#include "qep_defs.h"

#ifndef F28_DATA_TYPES
#define F28_DATA_TYPES
typedef short           Cint16;
typedef long            Cint32;
typedef unsigned short  CUint16;
typedef unsigned long   CUint32;
typedef float           Cfloat32;
typedef long double     Cfloat64;
#endif


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
// Define the below type def to give configurable QEP access to the FCL lib
//
typedef union
{
    volatile struct EQEP_REGS *ptr;  // Aligned to lower 16-bits
    uint32_t pad;                    // 32-bits
} CLA_QEP_PTR;

extern CLA_QEP_PTR ClaQep;

//
// globals
//

#ifdef __cplusplus
}
#endif // extern "C"

#endif // end of FCL_CLA_H definition
