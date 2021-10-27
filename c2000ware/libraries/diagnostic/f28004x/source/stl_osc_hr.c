//#############################################################################
//
// FILE:  stl_osc_hr.c
//
// TITLE: Diagnostic Library Oscillator HRPWM software module source
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
// Includes
//
#include "stl_osc_hr.h"

//
// Global used by the SFO() library.
//
extern int32_t MEP_ScaleFactor;

//*****************************************************************************
//
// STL_OSC_HR_testSFO(const STL_OSC_HR_Handle oscHRHandle)
//
//*****************************************************************************
uint16_t STL_OSC_HR_testSFO(const STL_OSC_HR_Handle oscHRHandle)
{
    int16_t sfoStatus, sfoReturnValue;
    uint32_t autoConversionEnabled;
    uint32_t delayCount = oscHRHandle->sfoDelay;
    uint16_t testStatus = STL_OSC_HR_FAIL;
    uint16_t prevEdge;
    bool loopBreak;

    MEP_ScaleFactor = 0;

    //
    // Save previous high-resolution edge(s) controlled by Micro Edge
    // Positioner (MEP) and set the new edges for given channel.
    //
    prevEdge = ((HWREGH(oscHRHandle->ePWMBase + HRPWM_O_HRCNFG) & (0x3U <<
                (uint16_t)oscHRHandle->channel)) >>
                (uint16_t)oscHRHandle->channel);
    HRPWM_setMEPEdgeSelect(oscHRHandle->ePWMBase, oscHRHandle->channel,
                           oscHRHandle->mepEdgeMode);

    //
    // Get status on whether auto-conversion is enabled.
    //
   autoConversionEnabled = (HWREGH(oscHRHandle->ePWMBase + HRPWM_O_HRCNFG) &
                            HRPWM_HRCNFG_AUTOCONV);

    //
    // Enable the MEP to automatically scale HRMSTEP.
    //
    HRPWM_enableAutoConversion(oscHRHandle->ePWMBase);

    //
    // Call SFO() until calibration is complete.
    //
    sfoReturnValue = (int16_t)SFO();
    sfoStatus = sfoReturnValue;
    while(delayCount != 0U)
    {
        if(sfoStatus == STL_OSC_HR_SFO_INCOMPLETE)
        {
            sfoReturnValue = (int16_t)SFO();
            sfoStatus = sfoReturnValue;
            loopBreak = false;
        }
        else if(sfoStatus == STL_OSC_HR_SFO_COMPLETE)
        {
            //
            // Check if MEP_ScaleFactor is within the specified range.
            //
            if((MEP_ScaleFactor >= oscHRHandle->mepMin) &&
                (MEP_ScaleFactor <= oscHRHandle->mepMax))
            {
                //
                // Return pass.
                //
                testStatus = STL_OSC_HR_PASS;
                loopBreak = true;
            }
            else
            {
                //
                // MEP scale factor is out of range.
                //
                STL_Util_setErrorFlag(STL_UTIL_OSC_HR_MEP_RANGE);
                testStatus = STL_OSC_HR_FAIL;
                loopBreak = true;
            }
        }
        else
        {
            //
            // SFO calibration failed to calculate scale factor.
            //
            STL_Util_setErrorFlag(STL_UTIL_OSC_HR_SFO);
            testStatus = STL_OSC_HR_FAIL;
            loopBreak = true;
        }
        if(loopBreak == true)
        {
            break;
        }
        delayCount--;
    }

    if(delayCount == 0U)
    {
        //
        // SFO took too long to execute.
        //
        STL_Util_setErrorFlag(STL_UTIL_OSC_HR_DELAY);
        testStatus = STL_OSC_HR_FAIL;
    }

    //
    // Restore edge controlled by MEP for given channel.
    //
    HRPWM_setMEPEdgeSelect(oscHRHandle->ePWMBase, oscHRHandle->channel,
                           (HRPWM_MEPEdgeMode)prevEdge);

    //
    // Disable the MEP from automatically scaling HRMSTEP if it was originally
    // disabled.
    //
    if(autoConversionEnabled == 0U)
    {
        HRPWM_disableAutoConversion(oscHRHandle->ePWMBase);
    }
    return(testStatus);
}

//
// End of File
//

