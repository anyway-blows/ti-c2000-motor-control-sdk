//#############################################################################
//
// FILE:  sta_comm.h
//
// TITLE: Self Test Application Communication header
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

#ifndef STA_COMM_H
#define STA_COMM_H

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
// Included Files
//
#include "driverlib.h"
#include "device.h"

//*****************************************************************************
//
//! \addtogroup sta_comm STA_Comm API Functions
//!
//! This module handles the configuration of the SCIA module for communication
//! over the board USB-to-UART adapter. This path will allow the application to
//! send test pass/fail status to a terminal program.
//!
//! The code for this module is contained in <tt>sta_comm.c</tt>, with
//! <tt>sta_comm.h</tt> containing the API declarations.
//!
//! @{
//
//*****************************************************************************

//
// Defines
//
#define STA_COMM_BAUD_RATE  9600    // Currently selected baud rate for UART

//
// Function Prototypes
//

//*****************************************************************************
//
//! \brief Configures SCIA for serial communication.
//!
//! This function configures SCIA for communication to a serial communication
//! terminal for 9600 baud rate, 8-N-1.
//!
//! \return None.
//
//*****************************************************************************
extern void STA_Comm_configSCIA(void);

//*****************************************************************************
//
//! \brief Transmits data via SCIA.
//!
//! \param msg is a \0 terminated array of characters
//!
//! This function transmits the \b msg via SCIA to a serial communication
//! terminal.
//!
//! \return None.
//
//*****************************************************************************
extern void STA_Comm_transmitData(unsigned char *msg);

#endif // STA_COMM_H

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

//
// End of File
//

