//#############################################################################
//
// FILE:  sta_user.c
//
// TITLE: Self Test Application User source
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

//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "sta_user.h"
#include "sta_comm.h"
#include "sta_util.h"

//
// Globals
//

//
// March inject error test object and buffers
//
STL_March_InjectErrorObj STA_User_marchErrorObj;
STL_March_InjectErrorHandle STA_User_marchErrorHandle;

#pragma DATA_SECTION(STA_User_marchTestData, "ramm0");
#pragma DATA_ALIGN(STA_User_marchTestData, 2)
uint16_t STA_User_marchTestData[STA_USER_MARCH_DATA_SIZE];

#pragma DATA_SECTION(STA_User_marchTestDataCopy, "ramgs0");
#pragma DATA_ALIGN(STA_User_marchTestDataCopy, 2)
uint16_t STA_User_marchTestDataCopy[STA_USER_MARCH_DATA_SIZE];

//
// CAN message RAM March test copy buffer
//
#pragma DATA_SECTION(STA_User_marchTestDataCANCopy, "ramgs0");
#pragma DATA_ALIGN(STA_User_marchTestDataCANCopy, 2)
uint16_t STA_User_marchTestDataCANCopy[STA_USER_MARCH_CAN_COPY_SIZE];

//
// CAN parity logic test variables
//
bool STA_User_canInterruptFlag;
uint16_t STA_User_parityErrorCode;
uint16_t STA_User_canErrorReturnVal;

//
// OSC Timer2 test object
//
STL_OSC_CT_Obj STA_User_oscTimer2Obj;
STL_OSC_CT_Handle STA_User_oscTimer2Handle;

//
// Oscillator HRPWM test object, including ePWM and MEP_ScaleFactor required by
// SFO library.
//
STL_OSC_HR_Obj STA_User_oscHRObj;
STL_OSC_HR_Handle STA_User_oscHRHandle;
int32_t MEP_ScaleFactor;
volatile uint32_t ePWM[9] = {0,
                             EPWM1_BASE,
                             EPWM2_BASE,
                             EPWM3_BASE,
                             EPWM4_BASE,
                             EPWM5_BASE,
                             EPWM6_BASE,
                             EPWM7_BASE,
                             EPWM8_BASE};

//*****************************************************************************
//
// STA_User_initMarch(void)
//
//*****************************************************************************
void STA_User_initMarch(void)
{
    STA_User_marchErrorObj.address    = (uint32_t)STA_User_marchTestData;
    STA_User_marchErrorObj.ramSection = MEMCFG_SECT_M0;
    STA_User_marchErrorObj.xorMask    = 0x00000000U;
    STA_User_marchErrorObj.testMode   = MEMCFG_TEST_FUNCTIONAL;

    STA_User_marchErrorHandle = &STA_User_marchErrorObj;
}

//*****************************************************************************
//
// STA_User_initOSCTimer2Test(void)
//
//*****************************************************************************
void STA_User_initOSCTimer2Test(void)
{
    STA_User_oscTimer2Obj.minCount    = STA_USER_OSC_MIN_COUNT;
    STA_User_oscTimer2Obj.maxCount    = STA_USER_OSC_MAX_COUNT;
    STA_User_oscTimer2Obj.clockSource = CPUTIMER_CLOCK_SOURCE_SYS;
    STA_User_oscTimer2Obj.prescaler   = CPUTIMER_CLOCK_PRESCALER_1;

    STA_User_oscTimer2Handle = &STA_User_oscTimer2Obj;
}

//*****************************************************************************
//
// STA_User_initOSCHRTest(void)
//
//*****************************************************************************
void STA_User_initOSCHRTest(void)
{
    STA_User_oscHRObj.ePWMBase       = EPWM1_BASE;
    STA_User_oscHRObj.channel        = HRPWM_CHANNEL_A;
    STA_User_oscHRObj.mepEdgeMode    = HRPWM_MEP_CTRL_RISING_EDGE;
    STA_User_oscHRObj.mepMax         = (int16_t)STA_USER_OSC_HR_MEP_MAX;
    STA_User_oscHRObj.mepMin         = (int16_t)STA_USER_OSC_HR_MEP_MIN;
    STA_User_oscHRObj.sfoDelay       = 2000;

    STA_User_oscHRHandle = &STA_User_oscHRObj;
}

//*****************************************************************************
//
// STA_User_canParityErrorISR(void)
//
//*****************************************************************************
__interrupt void STA_User_canParityErrorISR(void)
{
    STA_User_canErrorReturnVal = STL_CAN_RAM_checkErrorStatus(CANA_BASE);
    STA_User_canInterruptFlag = true;
    STA_User_parityErrorCode = HWREGH(CANA_BASE + CAN_O_PERR);

    CAN_clearGlobalInterruptStatus(CANA_BASE, CAN_GLOBAL_INT_CANINT0);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

//
// End of File
//
