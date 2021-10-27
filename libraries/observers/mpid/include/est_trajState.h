/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

//! \file   modules/vib_comp/src/32b/vib_comp_priv.h
//! \brief Defines the structures for the VIB_COMP object 
//!
//! (C) Copyright 2015, Texas Instruments, Inc.

#ifndef _EST_TRAJSTATE_H_
#define _EST_TRAJSTATE_H_

// **************************************************************************
// the includes

#ifdef __TMS320C28XX_CLA__
#include "libraries/math/include/CLAmath.h"
#else
#include <math.h>
#endif // __TMS320C28XX_CLA__

#include "libraries/math/include/math.h"

//!
//!
//! \addtogroup EST
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs


// **************************************************************************
// the function prototypes

//! \brief     Configures the controller for each of the estimator states
//! \param[in] handle      The estimator (EST) handle
//! \param[in] pUserParams
extern void EST_configureTrajState(EST_Handle handle, USER_Params *pUserParams,
                                   PI_Handle piHandle_spd,
                                   PI_Handle piHandle_Id, PI_Handle piHandle_Iq);


//! \brief     Sets up the trajectory generator
//! \param[in] handle                The trajectory generator (EST_Traj) handle
//! \param[in] Iq_A
//! \param[in] targetValue_spd_Hz    The target speed value during run time, Hz
//! \param[in] targetValue_Id_A      The target Id current value during run time, A
extern void EST_setupTrajState(EST_Handle handle,
                               const float32_t Iq_A,
                               const float32_t targetValue_spd_Hz,
                               const float32_t targetValue_Id_A);

// **************************************************************************

#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _EST_TRAJSTATE_H_ definition

