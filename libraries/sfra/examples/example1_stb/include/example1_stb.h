//#############################################################################
//
// FILE:   example1_stb.h
//
// TITLE: Example 1 Software Test Bench for SFRA Library
//
//#############################################################################
// $TI Release: C2000 Software Frequency Response Analyzer Library v1.40.00.00 $
// $Release Date: Tue Sep 21 16:33:07 CDT 2021 $
// $Copyright:
// Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
//
// ALL RIGHTS RESERVED
// $
//#############################################################################

#ifndef EXAMPLE1_STB_H
#define EXAMPLE1_STB_H

#ifdef __cplusplus

extern "C" {
#endif

//
// the includes
//
#include <stdint.h>

#ifndef __TMS320C28XX_CLA__
#include <math.h>
#else
#include <CLAmath.h>
#endif

#include "sfra_examples_settings.h"
#include "sfra_examples_hal.h"

#include "DCLF32.h"

#include "sfra_f32.h"
#include "sfra_gui_scicomms_driverlib.h"


//
// defines
//


//
// System Settings
//

//
// typedefs
//


//
// globals
//

extern DCL_PI gi;
extern volatile float32_t gi_out;
extern volatile float32_t gi_out_prev;
//
// Reference variables
// current set point
//
extern volatile float32_t ac_cur_ref;
//
//flag to close the loop
//
extern volatile int32_t closeGiLoop;

//
// the function prototypes
//

#ifndef __TMS320C28XX_CLA__
void setupSFRA();
void globalVariablesInit();
#else
#endif

#pragma FUNC_ALWAYS_INLINE(controlCode)
//
//control Code
//
static inline void controlCode(void)
{
    setProfilingGPIO();

    if(closeGiLoop == 1)
    {
        //
        //self test SFRA
        //
        gi_out = DCL_runPI_C2(&gi, SFRA_F32_inject(ac_cur_ref),
                            gi_out_prev );

    }

    //
    // self test SFRA
    //
    SFRA_F32_collect((float *)&gi_out, (float *)&gi_out_prev);
    gi_out_prev = gi_out;

    clearPWMInterruptFlag(C28x_CONTROLISR_INTERRUPT_TRIG_PWM_BASE);
    resetProfilingGPIO();

}


#ifdef __cplusplus
}
#endif                                  /* extern "C" */


#endif
