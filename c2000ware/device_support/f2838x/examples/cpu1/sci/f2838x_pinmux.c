//*****************************************************************************
//
// f2838x_pinmux.c - Function to write the generated pin mux values to the
//                    appropriate registers.
// Created using TI Pinmux 1.0.658 on 2/27/2019 at 10:15:05 AM.
//
//*****************************************************************************
//
// Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
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
//
//*****************************************************************************
//
// NOTE!! This file uses bit field structures and macros from C2000Ware.
// This function is provided for your convenience and to serve as an example
// of the use of the generated header file, but its use is not required.
//
// To download C2000Ware:  http://www.ti.com/tool/C2000Ware
//
//*****************************************************************************

#include "f28x_project.h"
#include "f2838x_pinmux.h"

//*****************************************************************************
//
// Configures the pin mux registers, using the generated register values.
//
// This function writes the values generated by the pin mux tool to their
// corresponding GPIO control registers. These generated values should be found
// in the generated "f2838x_pinmux.h."
//
//*****************************************************************************
void
GPIO_setPinMuxConfig(void)
{
    Uint32 lockValA;
    Uint32 lockValB;
    Uint32 lockValC;
    Uint32 lockValD;
    Uint32 lockValE;

    EALLOW;

    //
    // Save the current value of the GPIO lock registers
    //
    lockValA = GpioCtrlRegs.GPALOCK.all;
    lockValB = GpioCtrlRegs.GPBLOCK.all;
    lockValC = GpioCtrlRegs.GPCLOCK.all;
    lockValD = GpioCtrlRegs.GPDLOCK.all;
    lockValE = GpioCtrlRegs.GPELOCK.all;

    //
    // Unlock the GPIO control registers
    //
    GpioCtrlRegs.GPALOCK.all = 0x00000000;
    GpioCtrlRegs.GPBLOCK.all = 0x00000000;
    GpioCtrlRegs.GPCLOCK.all = 0x00000000;
    GpioCtrlRegs.GPDLOCK.all = 0x00000000;
    GpioCtrlRegs.GPELOCK.all = 0x00000000;

    //
    // Clear the mux register fields that are about to be changed
    //
    GpioCtrlRegs.GPAGMUX1.all	&= ~GPAMUX1_MASK;
    GpioCtrlRegs.GPAGMUX2.all	&= ~GPAMUX2_MASK;
    GpioCtrlRegs.GPAMUX1.all	&= ~GPAMUX1_MASK;
    GpioCtrlRegs.GPAMUX2.all	&= ~GPAMUX2_MASK;
    GpioCtrlRegs.GPBGMUX1.all	&= ~GPBMUX1_MASK;
    GpioCtrlRegs.GPBGMUX2.all	&= ~GPBMUX2_MASK;
    GpioCtrlRegs.GPBMUX1.all	&= ~GPBMUX1_MASK;
    GpioCtrlRegs.GPBMUX2.all	&= ~GPBMUX2_MASK;
    GpioCtrlRegs.GPCGMUX1.all	&= ~GPCMUX1_MASK;
    GpioCtrlRegs.GPCGMUX2.all	&= ~GPCMUX2_MASK;
    GpioCtrlRegs.GPCMUX1.all	&= ~GPCMUX1_MASK;
    GpioCtrlRegs.GPCMUX2.all	&= ~GPCMUX2_MASK;
    GpioCtrlRegs.GPDGMUX1.all	&= ~GPDMUX1_MASK;
    GpioCtrlRegs.GPDGMUX2.all	&= ~GPDMUX2_MASK;
    GpioCtrlRegs.GPDMUX1.all	&= ~GPDMUX1_MASK;
    GpioCtrlRegs.GPDMUX2.all	&= ~GPDMUX2_MASK;
    GpioCtrlRegs.GPEGMUX1.all	&= ~GPEMUX1_MASK;
    GpioCtrlRegs.GPEMUX1.all	&= ~GPEMUX1_MASK;

    //
    // Write pin muxing to mux registers
    //
    GpioCtrlRegs.GPAGMUX1.all	|=  GPAGMUX1_VALUE;
    GpioCtrlRegs.GPAGMUX2.all	|=  GPAGMUX2_VALUE;
    GpioCtrlRegs.GPAMUX1.all	|=  GPAMUX1_VALUE;
    GpioCtrlRegs.GPAMUX2.all	|=  GPAMUX2_VALUE;
    GpioCtrlRegs.GPBGMUX1.all	|=  GPBGMUX1_VALUE;
    GpioCtrlRegs.GPBGMUX2.all	|=  GPBGMUX2_VALUE;
    GpioCtrlRegs.GPBMUX1.all	|=  GPBMUX1_VALUE;
    GpioCtrlRegs.GPBMUX2.all	|=  GPBMUX2_VALUE;
    GpioCtrlRegs.GPCGMUX1.all	|=  GPCGMUX1_VALUE;
    GpioCtrlRegs.GPCGMUX2.all	|=  GPCGMUX2_VALUE;
    GpioCtrlRegs.GPCMUX1.all	|=  GPCMUX1_VALUE;
    GpioCtrlRegs.GPCMUX2.all	|=  GPCMUX2_VALUE;
    GpioCtrlRegs.GPDGMUX1.all	|=  GPDGMUX1_VALUE;
    GpioCtrlRegs.GPDGMUX2.all	|=  GPDGMUX2_VALUE;
    GpioCtrlRegs.GPDMUX1.all	|=  GPDMUX1_VALUE;
    GpioCtrlRegs.GPDMUX2.all	|=  GPDMUX2_VALUE;
    GpioCtrlRegs.GPEGMUX1.all	|=  GPEGMUX1_VALUE;
    GpioCtrlRegs.GPEMUX1.all	|=  GPEMUX1_VALUE;

    //
    // Write pin analog mode select to registers
    //
    GpioCtrlRegs.GPBAMSEL.all	&= ~GPBAMSEL_MASK;
    GpioCtrlRegs.GPBAMSEL.all	|= GPBAMSEL_VALUE;

    //
    // Restore GPIO lock register values
    //
    GpioCtrlRegs.GPALOCK.all = lockValA;
    GpioCtrlRegs.GPBLOCK.all = lockValB;
    GpioCtrlRegs.GPCLOCK.all = lockValC;
    GpioCtrlRegs.GPDLOCK.all = lockValD;
    GpioCtrlRegs.GPELOCK.all = lockValE;

    EDIS;
}
