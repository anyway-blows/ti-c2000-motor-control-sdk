//#############################################################################
//
// FILE:   F28002x.h
//
// TITLE:  A set of Constant Values for the F28002x Family.
//
//#############################################################################
// $TI Release: F28002x Support Library v3.04.00.00 $
// $Release Date: Fri Feb 12 18:58:34 IST 2021 $
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

#ifndef F28002x_H
#define F28002x_H

/*!
    \brief Specifies the Offset to the TI OTP
*/

#define F021_PROGRAM_TIOTP_OFFSET    0x00070000U //TI OTP start on C28x



/* Final values to be determined */

/*!
    \brief Maximum FClck Value
    \This value is not used in API anymore
    \instead this value is programmed in TI OTP for API usage
*/
#define F021_FCLK_MAX    0xAU


/*!
    \brief PGM_OSU Max Value
*/
#define F021_PGM_OSU_MAX    0xFFU

/*!
    \brief PGM_OSU Min Value
*/
#define F021_PGM_OSU_MIN    0x02U

/*!
    \brief ERA_OSU Max Value
*/
#define F021_ERA_OSU_MAX    0xFFU

/*!
    \brief ERA_OSU Min Value
*/
#define F021_ERA_OSU_MIN    0x02U

/*!
    \brief ADD_EXZ Max Value
*/
#define F021_ADD_EXZ_MAX    0x0FU

/*!
    \brief ADD_EXZ Min Value
*/
#define F021_ADD_EXZ_MIN    0x02U

/*!
    \brief EXE_VALD Max Value
*/
#define F021_EXE_VALD_MAX    0x0FU

/*!
    \brief EXE_VALD Min Value
*/
#define F021_EXE_VALD_MIN    0x02U

/*!
    \brief PROG_PUL_WIDTH Max Value
*/
#define F021_PROG_PUL_WIDTH_MAX    0xFFFFU

/*!
    \brief PROG_PUL_WIDTH Min Value
*/
#define F021_PROG_PUL_WIDTH_MIN    0x0002U

/*!
    \brief ERA_PUL_WIDTH Max Value
*/
#define F021_ERA_PUL_WIDTH_MAX    0xFFFFFFFFU

/*!
    \brief ERA_PUL_WIDTH Min Value
*/
#define F021_ERA_PUL_WIDTH_MIN    0x00000002U

/*!
 *  FMC memory map defines
 */
#if defined (_F28002x)
    #define F021_FLASH_MAP_BEGIN      0x80000U
    #define F021_FLASH_MAP_END        0x8FFFFU
    #define F021_OTP_MAP_BEGIN        0x78000U  //Customer OTP start
    #define F021_OTP_MAP_END          0x783FFU   //Customer OTP End
    #define F021_OTPECC_MAP_BEGIN     0x1071000U
    #define F021_OTPECC_MAP_END       0x107107FU
    #define F021_FLASHECC_MAP_BEGIN   0x1080000U
    #define F021_FLASHECC_MAP_END     0x1081FFFU
#endif

/*!
    \brief Define to map the direct access to the FMC registers.
*/
    #define F021_CPU0_REGISTER_ADDRESS 0x0005F800U

/*!
 *  Specific TI OTP Offsets
 */
    #define F021_TIOTP_PER_BANK_SIZE 0x400U
    #define F021_TIOTP_SETTINGS_BASE 0xA8U

/* 32-bit CLKSRCCTL1 Register */
    #define  CLKSRCCTL1      (*(volatile uint32*)(0x5D208U))

#endif /* F28002x_H */
