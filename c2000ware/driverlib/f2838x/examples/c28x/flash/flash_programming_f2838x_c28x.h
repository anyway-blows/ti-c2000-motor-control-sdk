//###########################################################################
//
// FILE:   flash_programming_f2838x_c28x.h
//
// TITLE:  A set of Constant Values for the F2838x C28x Family.
//
//#############################################################################
// $TI Release: F2838x Support Library v3.04.00.00 $
// $Release Date: Fri Feb 12 19:08:49 IST 2021 $
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

#ifndef FLASH_PROGRAMMING_F2838X_C28X_H
#define FLASH_PROGRAMMING_F2838X_C28X_H

//
// Bank0 Sector start addresses
//
#define FlashStartAddress           0x80000
#define Bzero_Sector0_start	        0x80000
#define Bzero_Sector1_start	        0x82000
#define Bzero_Sector2_start	        0x84000
#define Bzero_Sector3_start	        0x86000
#define Bzero_Sector4_start	        0x88000
#define Bzero_Sector5_start	        0x90000
#define Bzero_Sector6_start	        0x98000
#define Bzero_Sector7_start	        0xA0000
#define Bzero_Sector8_start	        0xA8000
#define Bzero_Sector9_start	        0xB0000
#define Bzero_Sector10_start	    0xB8000
#define Bzero_Sector11_start	    0xBA000
#define Bzero_Sector12_start	    0xBC000
#define Bzero_Sector13_start	    0xBE000
#define FlashEndAddress             0xBFFFF

//
// Sector length in number of 32bits
//
#define Sector16KB_u32length   0x1000
#define Sector64KB_u32length   0x4000

#endif /* FLASH_PROGRAMMING_F2838X_C28X_H */
