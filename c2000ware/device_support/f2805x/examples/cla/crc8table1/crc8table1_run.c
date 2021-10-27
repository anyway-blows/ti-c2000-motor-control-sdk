//###########################################################################
//
// FILE:  crc8table1_run.c
//
// TITLE: CRC8 Table-generation Algorithm Run
//
// DESCRIPTION: Call Task 1
//
//###########################################################################
// $TI Release: F2805x Support Library v2.02.00.00 $
// $Release Date: Fri Feb 12 19:14:47 IST 2021 $
// $Copyright:
// Copyright (C) 2012-2021 Texas Instruments Incorporated - http://www.ti.com/
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

//
// Included Files
//
#include "DSP28x_Project.h"
#include "F2805x_Cla_defines.h"

//
// Include the test header file whose name is based on the test name
// which is defined by the macro TEST on the command line
//
#include XSTRINGIZE(XCONCAT(TEST_NAME, _shared.h))

void ram_enable(void);
void ram_disable(void);

void test_run(void)
{
    Uint16 i;

    //
    // Assign the RAM to the CPU
    // Clear out the table
    // Assign the RAM back to the CLA
    //
    ram_disable();
    for (i = 0; i < 256; i++) table[i] = 0x12345678;
    ram_enable();


    Cla1ForceTask1andWait();

#if 0
    Cla1ForceTask2andWait();

    Cla1ForceTask3andWait();

    Cla1ForceTask4andWait();

    Cla1ForceTask5andWait();

    Cla1ForceTask6andWait();

    Cla1ForceTask7andWait();

    Cla1ForceTask8andWait();
#endif
}

void ram_disable()
{
    EALLOW;
    Cla1Regs.MMEMCFG.bit.RAM0E  = 0;
    Cla1Regs.MMEMCFG.bit.RAM1E  = 0;
    EDIS;

    asm("   NOP");
    asm("   NOP");
    asm("   NOP");
    asm("   NOP");
    return;
}

void ram_enable()
{
    EALLOW;
    Cla1Regs.MMEMCFG.bit.RAM0E  = 1;
    Cla1Regs.MMEMCFG.bit.RAM1E  = 1;
    EDIS;

    asm("   NOP");
    asm("   NOP");
    asm("   NOP");
    asm("   NOP");
}