//#############################################################################
//
// FILE:           abs2qep.h
//
// Description:    Example for the abs2qep library.
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
// Included Files
//

#include "pto_abs2qep.h"

//
// Check results with the eQEP Counter receiving
// the PTO.
// Define the allowed delta between calculated
// absolute and incremental angles.  This is used
// to determine pass or fail reflected in the
// variables pass_count and fail_count
//
#define CHECK_EQEP_POSCTR       1
#define DELTA_THRESHOLD      0.9f

//
// EPWM timer period for a 100us position sample
//
#define EPWM3_TIMER_TBPRD   5000U

//
// Function prototypes
//
extern void pto_setupGPIO(void);
extern void pto_configEQEP(void);
extern void pto_initAbs2QEP(void);
extern uint32_t pto_generateTestAbsPosition(uint32_t current_position);
extern void pto_checkPosition(int32_t incremental_position,                    \
                              uint32_t absolute_position);
extern void pto_configEPWM(void);
extern __interrupt void pto_EPWM3ISR(void);

//
// End of File
//

