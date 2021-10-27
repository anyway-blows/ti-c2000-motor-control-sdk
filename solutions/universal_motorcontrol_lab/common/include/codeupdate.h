//#############################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:28 CDT 2021 $
// $Copyright:
// Copyright (C) 2017-2019 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef CODE_UPDATE_H
#define CODE_UPDATE_H

//
//! \file   /solutions/fast_uni_lab/common/include/codeupdate.h
//!
//! \brief  header file to be included in all labs
//!         support for universal lab with F28002x
//!
//

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

//*****************************************************************************
//
//! \addtogroup CODE UPDATE
//! @{
//
//*****************************************************************************

// Included Files
#include "driverlib.h"
#include "device.h"

#ifdef _F28002x
//
// Include Flash API include file
//
#include "F021_F28002x_C28x.h"

// Bank0 Sector start addresses
#define FlashStartAddress           0x80000U
#define Bzero_Sector0_start         0x80000U
#define Bzero_Sector1_start         0x81000U
#define Bzero_Sector2_start         0x82000U
#define Bzero_Sector3_start         0x83000U
#define Bzero_Sector4_start         0x84000U
#define Bzero_Sector5_start         0x85000U
#define Bzero_Sector6_start         0x86000U
#define Bzero_Sector7_start         0x87000U
#define Bzero_Sector8_start         0x88000U
#define Bzero_Sector9_start         0x89000U
#define Bzero_Sector10_start        0x8A000U
#define Bzero_Sector11_start        0x8B000U
#define Bzero_Sector12_start        0x8C000U
#define Bzero_Sector13_start        0x8D000U
#define Bzero_Sector14_start        0x8E000U
#define Bzero_Sector15_start        0x8F000U
#define FlashEndAddress             0x8FFFFU
#endif // _F28002x

//-----------------------------------------------------------------------------
#ifdef _F28004x
//
// Include Flash API include file
//
#include "F021_F28004x_C28x.h"

// Bank0 Sector start addresses
#define Bzero_Sector0_start         0x80000
#define Bzero_Sector1_start         0x81000
#define Bzero_Sector2_start         0x82000
#define Bzero_Sector3_start         0x83000
#define Bzero_Sector4_start         0x84000
#define Bzero_Sector5_start         0x85000
#define Bzero_Sector6_start         0x86000
#define Bzero_Sector7_start         0x87000
#define Bzero_Sector8_start         0x88000
#define Bzero_Sector9_start         0x89000
#define Bzero_Sector10_start        0x8A000
#define Bzero_Sector11_start        0x8B000
#define Bzero_Sector12_start        0x8C000
#define Bzero_Sector13_start        0x8D000
#define Bzero_Sector14_start        0x8E000
#define Bzero_Sector15_start        0x8F000

// Bank1 Sector start addresses
#define Bone_Sector0_start          0x90000
#define Bone_Sector1_start          0x91000
#define Bone_Sector2_start          0x92000
#define Bone_Sector3_start          0x93000
#define Bone_Sector4_start          0x94000
#define Bone_Sector5_start          0x95000
#define Bone_Sector6_start          0x96000
#define Bone_Sector7_start          0x97000
#define Bone_Sector8_start          0x98000
#define Bone_Sector9_start          0x99000
#define Bone_Sector10_start         0x9A000
#define Bone_Sector11_start         0x9B000
#define Bone_Sector12_start         0x9C000
#define Bone_Sector13_start         0x9D000
#define Bone_Sector14_start         0x9E000
#define Bone_Sector15_start         0x9F000
#endif // _F28004x

//Sector length in number of 16bits
#define Sector8KB_u16length         0x1000U

//Sector length in number of 32bits
#define Sector8KB_u32length         0x800U

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

#endif // end of CODE_UPDATE_H defines
