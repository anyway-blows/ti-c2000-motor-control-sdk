//#############################################################################
//
// FILE:  stl_osc_hr.h
//
// TITLE: Diagnostic Library Oscillator HRPWM software module header
//
//#############################################################################
// $TI Release: C2000 Diagnostic Library v2.01.00 $
// $Release Date: Fri Feb 12 19:23:23 IST 2021 $
// $Copyright:
// Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef STL_OSC_HR_H
#define STL_OSC_HR_H

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//
// Includes
//
#include "stl_util.h"
#include "hrpwm.h"
#include "sfo_v8.h"

//*****************************************************************************
//
//! \addtogroup stl_osc_hr Oscillator HRPWM API Functions
//!
//! The code for this module is contained in <tt>source/stl_osc_hr.c</tt>, with
//! <tt>include/stl_osc_hr.h</tt> containing the API declarations for use by
//! applications.
//!
//! # Error Injection #
//!
//! Here are techniques to inject error:
//!
//! \b STL_OSC_HR_testSFO()
//! - Set mepMax and mepMin the same value so that the MEP scale factor
//!   calculated by SFO calibration will be out of range.
//! - Disable HRPWM clock by clearing HRPWM bit in the PCLKCR0 register.
//! - Set sfoDelay to 0 to cause timeout.
//!
//! @{
//
//*****************************************************************************

//
// Defines
//
#define STL_OSC_HR_PASS              0U
#define STL_OSC_HR_FAIL              1U
#define STL_OSC_HR_SFO_INCOMPLETE    0   // Calibration is incomplete
#define STL_OSC_HR_SFO_COMPLETE      1   // Calibration is complete
#define STL_OSC_HR_SFO_ERROR         2   // Calibration resulted in MEP steps
                                         // greater than 255

//
//! \brief Defines the OSC HR test object
//!
typedef struct
{
    uint32_t              ePWMBase;          //!< EPWM Base
    HRPWM_Channel         channel;           //!< HRPWM channel
    HRPWM_MEPEdgeMode     mepEdgeMode;       //!< Edge(s) controlled by MEP
    int16_t               mepMin;            //!< MEP lower bound
    int16_t               mepMax;            //!< MEP upper bound
    uint32_t              sfoDelay;          //!< SFO function delay
} STL_OSC_HR_Obj;

//
//! \brief Defines the OSC HR test handle
//!
typedef STL_OSC_HR_Obj * STL_OSC_HR_Handle;

//
// Prototypes
//

//*****************************************************************************
//
//! \brief Tests the HRPWM SFO library's calibration process to ensure
//! execution completion and verify value of MEP scale factor
//!
//! \param oscHRHandle is a pointer to the OSC HR object.
//!
//! This function runs calibration using the SFO Library Software to calculate
//! an appropriate Micro Edge Positioner (MEP) scale factor for HRPWM-supported
//! ePWM modules to verify this value.
//!
//! The ePWM module used for the test is selected by \e ePWMBase member of the
//! OSC HR test object. The valid range of inputs is from EPWM1_BASE to
//! EPWMx_BASE where x is the value specified by PWM_CH - 1. PWM_CH is defined
//! in the SFO library header file SFO_V8.h. The channel member can be
//! configured to select between channels A and B. The \e mepEdgeMode value is
//! configured for the PWM edges to be controlled by MEP. The values \e mepMin
//! and \e mepMax define the minimum and maximum bounds of the expected MEP
//! scale factor range. The expected MEP scale factor for EPWMCLK of 100 MHz,
//! given a step size of 150 ps, is about 66 MEP steps.
//! (1/EPWMCLK)/Step size = (1/100 MHz)/150 ps is about 66.
//!
//! The \e sfoDelay value is the delay count required for the SFO() to finish
//! calibration. The typical EPWM cycles required for SFO() to complete
//! calibration if called repeatedly without interrupts is 130,000
//! EPWMCLK cycles. Please refer to \b Appendix \b A: \b SFO \b Library
//! \b Software of the Technical Reference Manual HRPWM chapter and the device
//! datasheet for more details. The repetition rate at which SFO() needs to be
//! executed depends on the application. If there is not a sufficiently large
//! interval between SFO() calls, the SFO() will return and not advance to the
//! next stage. The execution will only advance to the next stage if the
//! previous call's execution has completed and SFO() is called again. This
//! interval can be experimentally calculated for a particular application.
//!
//! \note The SFO library binary is provided in the C2000Ware calibration
//! libraries. The source code however is not publicly released. It may be
//! made available in some cases upon request through an FAE.
//!
//! \return If the MEP scale factor calculated by the calibration falls within
//! the MEP min and max values, the function returns \b STL_OSC_HR_PASS.
//! If the calibration fails to complete in the specified delay, if SFO()
//! returns an error status, or if the calculated MEP scale factor is outside
//! the range, this function returns \b STL_OSC_HR_FAIL.
//
//*****************************************************************************
extern uint16_t STL_OSC_HR_testSFO(const STL_OSC_HR_Handle oscHRHandle);

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif  //  STL_OSC_HR_H

//
// End of File
//
