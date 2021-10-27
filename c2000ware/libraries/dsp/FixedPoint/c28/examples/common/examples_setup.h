#ifndef _EXAMPLES_SETUP_H_
#define _EXAMPLES_SETUP_H_
//#############################################################################
//! \file /$FOLDER_EXAMPLES$/common/examples_setup.h
//!
//! \brief  Initialization routines for the Fixed Point library examples
//! \date   Sep 9, 2014
//
//  Group:          C2000
//  Target Family:  C28x
//
// (C)Copyright $YEAR$, Texas Instruments, Inc.
//#############################################################################
// $TI Release: C28x Fixed Point DSP Library v1.22.00.00.21 $
// $Release Date: Fri Feb 12 19:16:27 IST 2021 $
// $Copyright: Copyright (C) 2014-2021 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//#############################################################################

//*****************************************************************************
// includes
//*****************************************************************************
//#include "DSP28x_Project.h"
#include <stdint.h>
#include <float.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

//*****************************************************************************
// defines
//*****************************************************************************
#define CPU_FRQ_150MHZ	1

//*****************************************************************************
// typedefs
//*****************************************************************************

//*****************************************************************************
// globals
//*****************************************************************************

//*****************************************************************************
// function prototypes
//*****************************************************************************
// \brief Initialize system clocks
//
void FIXPT_DSP_initSystemClocks(void);

// \brief Initialize Enhanced PIE
//
void FIXPT_DSP_initEpie(void);

// \brief Initialize Flash
//
void FIXPT_DSP_initFlash(void);


#ifdef __cplusplus
}
#endif // extern "C"

#endif //end of _EXAMPLES_SETUP_H_ definition
