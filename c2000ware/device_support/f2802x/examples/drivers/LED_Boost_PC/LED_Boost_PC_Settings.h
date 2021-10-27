//###########################################################################
//
// FILE:   f2802x_examples/LED_Boost_CapTouch/LED_Boost_CapTouch_Settings.h
//
// TITLE:  Settings for the LED Boosterpack CapTouch Example
//
//###########################################################################
// $TI Release: F2802x Support Library v3.04.00.00 $
// $Release Date: Fri Feb 12 19:12:19 IST 2021 $
// $Copyright:
// Copyright (C) 2009-2021 Texas Instruments Incorporated - http://www.ti.com/
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
//###########################################################################

#ifndef _PROJSETTINGS_H
#define _PROJSETTINGS_H

//**************************************************************
//  NOTE: WHEN CHANGING THIS FILE PLEASE REBUILD ALL
//**************************************************************

void comp(void);

//
// Incremental Build options for System check-out
//
#define INCR_BUILD 2    //1 - Open-Loop
                        //2 - Closed-Loop Current
                        //3 - 

//
// System Settings
//
#define HistorySize 8       // Number of samples averaged for use in GUI
#define DMAX        700     // High clamping value for normal operation = DMAX/1000
#define DMIN        0       // Low clamping value for normal operation = DMIN/1000
#define uSec100     6000    // 100 uS

#endif

//
// End of File
//

