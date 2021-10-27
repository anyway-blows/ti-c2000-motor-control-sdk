//#############################################################################
//
// FILE:  sta_user.h
//
// TITLE: Self Test Application User header
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

#ifndef STA_USER_H
#define STA_USER_H

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
// Included files for tests.
//
#include "stl_can_ram.h"
#include "stl_pie_ram.h"
#include "stl_march.h"
#include "stl_osc_ct.h"
#include "stl_osc_hr.h"
#include "stl_util.h"

//*****************************************************************************
//
//! \addtogroup sta_user STA_User API Functions
//!
//! This module contains the declaration and initialization of the various
//! objects and handles required by SDL modules tested in this application.
//!
//! The code for this module is contained in <tt>sta_user.c</tt>, with
//! <tt>sta_user.h</tt> containing the API declarations.
//!
//! @{
//
//*****************************************************************************

//
// Defines
//

//
// March example data buffer length
//
#define STA_USER_MARCH_DATA_SIZE        16U

//
// CAN message RAM March test copy buffer length
//
#define STA_USER_MARCH_CAN_COPY_SIZE    320U

//
// Addresses of CAN A message RAM objects for use as an example. Note that the
// message RAM objects are placed at offsets of 0x20, although in RDA mode only
// up through 0x10 is used.
//
#define STA_USER_MARCH_CANA_OBJ32_START_ADDR                                   \
            (CANA_BASE + STL_CAN_RAM_ADDR_OFFSET)
#define STA_USER_MARCH_CANA_OBJ4_END_ADDR                                      \
            (CANA_BASE + STL_CAN_RAM_ADDR_OFFSET + (0x20U * 4U) + 0x10U)

#define STA_USER_MARCH_CANA_OBJ6_START_ADDR                                    \
            (CANA_BASE + STL_CAN_RAM_ADDR_OFFSET + (0x20U * 6U))
#define STA_USER_MARCH_CANA_OBJ15_END_ADDR                                     \
            (CANA_BASE + STL_CAN_RAM_ADDR_OFFSET + (0x20U * 15U) + 0x10U)

//
// PIE RAM example entry
//
#define STA_USER_PIE_RAM_ENTRY ((INT_FLASH_CORR_ERR >> STL_PIE_RAM_VECT_ID_S)*2)

//
// PIE RAM handler test interrupt and corresponding interrupt group mask
//
#define STA_USER_PIE_TEST_INT               INT_EPWM7
#define STA_USER_PIE_TEST_INT_GROUP_M                                          \
    (1U << (((uint16_t)(STA_USER_PIE_TEST_INT & STL_PIE_RAM_TABLE_COL_M) >>    \
             STL_PIE_RAM_TABLE_COL_S) - 1U))

//
// CRC test golden CRC of the DCSM OTP
//
#define STA_USER_DCSM_OTP_GOLDEN_CRC        0x76BC8E9BUL

//
// CAN message RAM example golden CRC
//
#define STA_USER_CAN_RAM_GOLDEN_CRC         0x7E8D8672UL

//
// Oscillator CPU Timer test values
//
// 13,107,000 cycles at DEVICE_SYSCLK_FREQ
//
#define STA_USER_OSC_DELAY_US    (13107000UL / (DEVICE_SYSCLK_FREQ / 1000000UL))
#define STA_USER_OSC_MIN_COUNT   13100000UL
#define STA_USER_OSC_MAX_COUNT   13115000UL

//
// Oscillator HRPWM test values
//
// Note that 6666666666L comes from the 150ps MEP step size.
//
#define STA_USER_OSC_HR_MEP       (6666666666L / (DEVICE_SYSCLK_FREQ))
#define STA_USER_OSC_HR_MEP_MIN   (STA_USER_OSC_HR_MEP - 10U)
#define STA_USER_OSC_HR_MEP_MAX   (STA_USER_OSC_HR_MEP + 10U)

//
// Globals
//

//
// March inject error test object and buffers
//
extern STL_March_InjectErrorObj STA_User_marchErrorObj;
extern STL_March_InjectErrorHandle STA_User_marchErrorHandle;
extern uint16_t STA_User_marchTestData[STA_USER_MARCH_DATA_SIZE];
extern uint16_t STA_User_marchTestDataCopy[STA_USER_MARCH_DATA_SIZE];

//
// CAN message RAM March test copy buffer
//
extern uint16_t STA_User_marchTestDataCANCopy[STA_USER_MARCH_CAN_COPY_SIZE];

//
// CAN parity logic test variables
//
extern bool STA_User_canInterruptFlag;
extern uint16_t STA_User_parityErrorCode;
extern uint16_t STA_User_canErrorReturnVal;;

//
// OSC Timer2 test object
//
extern STL_OSC_CT_Obj STA_User_oscTimer2Obj;
extern STL_OSC_CT_Handle STA_User_oscTimer2Handle;

//
// Oscillator HRPWM test object
//
extern STL_OSC_HR_Obj STA_User_oscHRObj;
extern STL_OSC_HR_Handle STA_User_oscHRHandle;

//
// Function Prototypes
//

//*****************************************************************************
//
//! This function initializes the inject error object for March13N test.
//!
//! \return None
//
//*****************************************************************************
extern void STA_User_initMarch(void);

//*****************************************************************************
//
//! This function initializes the Oscillator Timer2 test objects.
//!
//! \return None
//
//*****************************************************************************
extern void STA_User_initOSCTimer2Test(void);

//*****************************************************************************
//
//! This function initializes the object for the Oscillator HRPWM test.
//!
//! \return None
//
//*****************************************************************************
extern void STA_User_initOSCHRTest(void);

//*****************************************************************************
//
//! CAN ISR to execute upon parity error detection in CAN message RAM.
//!
//! \return None
//
//*****************************************************************************
extern __interrupt void STA_User_canParityErrorISR(void);

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

#endif

#endif // STA_USER_H

//
// End of File
//
