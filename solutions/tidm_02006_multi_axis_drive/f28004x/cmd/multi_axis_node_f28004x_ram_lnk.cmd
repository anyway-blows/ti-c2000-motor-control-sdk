//#############################################################################
//
// FILE:    multi_axis_slave_f28004x_ram_lnk.cmd
//
//#############################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:28 CDT 2021 $
// $Copyright:
// Copyright (C) 2017-2021 Texas Instruments Incorporated - http://www.ti.com/
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
// In addition to this memory linker command file,
// add the header linker command file directly to the project.
// The header linker command file is required to link the
// peripheral structures to the proper locations within
// the memory map.
//
// For BIOS applications add:      F28004x_Headers_BIOS.cmd
// For nonBIOS applications add:   F28004x_Headers_nonBIOS.cmd
//
// The user must define CLA_C in the project linker settings if using the
// CLA C compiler
// Project Properties -> C2000 Linker -> Advanced Options -> Command File
// Preprocessing -> --define
#ifdef CLA_C
// Define a size for the CLA scratchpad area that will be used
// by the CLA compiler for local symbols and temps
// Also force references to the special symbols that mark the
// scratchpad are.
CLA_SCRATCHPAD_SIZE = 0x100;
--undef_sym=__cla_scratchpad_end
--undef_sym=__cla_scratchpad_start
#endif //CLA_C

MEMORY
{
   /* BEGIN is used for the "boot to SARAM" bootloader mode   */
   BEGIN            : origin = 0x000000, length = 0x000002
   BOOT_RSVD        : origin = 0x000002, length = 0x0000F3     /* Part of M0, BOOT rom will use this for stack */
   RAMM0            : origin = 0x0000F5, length = 0x00030B
   RAMM1            : origin = 0x000400, length = 0x000400     /* on-chip RAM block M1 */

   CLA1_MSGRAMLOW   : origin = 0x001480, length = 0x000080
   CLA1_MSGRAMHIGH  : origin = 0x001500, length = 0x000080

   RAMLS0           : origin = 0x008000, length = 0x000800
   RAMLS1           : origin = 0x008800, length = 0x000800
   RAMLS2           : origin = 0x009000, length = 0x000800
   RAMLS3           : origin = 0x009800, length = 0x000800
   RAMLS4           : origin = 0x00A000, length = 0x000800
   RAMLS5           : origin = 0x00A800, length = 0x000800
   RAMLS6           : origin = 0x00B000, length = 0x000800
   RAMLS7           : origin = 0x00B800, length = 0x000800

   RAMGS0           : origin = 0x00C000, length = 0x002000
   RAMGS1           : origin = 0x00E000, length = 0x002000
   RAMGS2           : origin = 0x010000, length = 0x002000
   RAMGS3           : origin = 0x012000, length = 0x002000

   RESET            : origin = 0x3FFFC0, length = 0x000002

   FPUTABLES   		: origin = 0x3FA200, length = 0x000800	 /* FPU Tables in Boot ROM */
}

/*You can arrange the .text, .cinit, .const, .pinit, .switch and .econst to FLASH when RAM is filled up.*/
SECTIONS
{
   codestart        : > BEGIN
   .text            : >> RAMGS0 | RAMGS1 | RAMGS2, ALIGN(4)
   .cinit           : > RAMGS0 | RAMGS1 | RAMGS2, ALIGN(4)
   .switch          : > RAMGS0 | RAMGS1 | RAMGS2, ALIGN(4)
   .reset           : > RESET, TYPE = DSECT /* not used, */
   .stack           : > RAMM1

   .bss             : >> RAMLS2 | RAMLS3, ALIGN(4)
   .bss:output      : >> RAMLS2 | RAMLS3, ALIGN(4)
   .init_array      : > RAMGS0 | RAMGS1 | RAMGS2, ALIGN(4)
   .const           : > RAMGS0 | RAMGS1 | RAMGS2, ALIGN(4)
   .data            : >> RAMLS2 | RAMLS3, ALIGN(4)
   .sysmem          : >> RAMLS2 | RAMLS3

   // CLA Sections
   Cla1Prog         : >> RAMLS0, ALIGN(4)

   ClaData          : > RAMLS1, ALIGN(4)


   Cla1ToCpuMsgRAM  : > CLA1_MSGRAMLOW
   CpuToCla1MsgRAM  : > CLA1_MSGRAMHIGH

#ifdef CLA_C
   //
   // Must be allocated to memory the CLA has write access to
   //
   CLAscratch       :
                        { *.obj(CLAscratch)
                        . += CLA_SCRATCHPAD_SIZE;
                        *.obj(CLAscratch_end) } >  RAMLS2

   .scratchpad      : > RAMLS1
   .bss_cla		    : > RAMLS1

   .const_cla	    : > RAMLS0
#endif //CLA_C

   .TI.ramfunc      : >> RAMLS4 | RAMLS5 | RAMLS6 | RAMLS7, ALIGN(4)
   dclfuncs  		: >> RAMLS4 | RAMLS5 | RAMLS6 | RAMLS7, align(2)

   ramInitVars 		: > RAMM0
   ramgs0 			: > RAMGS3

   /* SFRA specific sections */
   SFRA_F32_Data    : > RAMGS3, ALIGN = 64

   /* Allocate FPU math areas: */
   FPUmathTables    : > FPUTABLES, TYPE = NOLOAD
}

//===========================================================================
// End of file.
//===========================================================================
