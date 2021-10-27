//#############################################################################
//
// FILE:    multi_axis_fsi_slave_f28002x_ram_lnk.cmd
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
// For BIOS applications add:      F28002x_Headers_BIOS.cmd
// For nonBIOS applications add:   F28002x_Headers_nonBIOS.cmd
//

MEMORY
{
   /* BEGIN is used for the "boot to SARAM" bootloader mode   */
   BEGIN             : origin = 0x00000000, length = 0x00000002
   BOOT_RSVD         : origin = 0x00000002, length = 0x00000126
   RAMM0             : origin = 0x00000128, length = 0x000002D8
/* RAMM1             : origin = 0x00000400, length = 0x00000380 */
/* BOOT_RSVD_SYSBIOS : origin = 0x00000780, length = 0x00000080 */
   RAMM1             : origin = 0x00000400, length = 0x00000400

/* RAMLS4            : origin = 0x0000A000, length = 0x00000800 */
/* RAMLS5            : origin = 0x0000A800, length = 0x00000800 */
/* RAMLS6            : origin = 0x0000B000, length = 0x00000800 */
/* RAMLS7            : origin = 0x0000B800, length = 0x00000800 */
   /* Combining the LS4,5,6, 7 RAMs */
   RAMLS4567         : origin = 0x0000A000, length = 0x00002000

   RAMGS0            : origin = 0x0000C000, length = 0x00000800

   RESET             : origin = 0x003FFFC0, length = 0x00000002

   FPUTABLES   		 : origin = 0x003F2A70, length = 0x0000081A	 /* FPU Tables in Boot ROM */
}

/*You can arrange the .text, .cinit, .const, .pinit, .switch and .econst to FLASH when RAM is filled up.*/
SECTIONS
{
   codestart        : > BEGIN
   .text            : > RAMLS4567
   .cinit           : > RAMLS4567
   .switch          : > RAMLS4567
   .reset           : > RESET, TYPE = DSECT /* not used, */
   .stack           : > RAMM0

   .bss             : >> RAMGS0 | RAMM1
   .bss:output      : >> RAMGS0 | RAMM1
   .bss:cio         : >> RAMGS0 | RAMM1
   .data            : >> RAMGS0 | RAMM1
   .sysmem          : >> RAMGS0 | RAMM1

   .init_array      : > RAMLS4567, ALIGN(4)
   .const           : > RAMLS4567, ALIGN(4)

   /* Allocate FPU math areas: */
   FPUmathTables    : > FPUTABLES, TYPE = NOLOAD

   .TI.ramfunc      : > RAMLS4567

   /* SFRA specific sections */
   SFRA_F32_Data    : >> RAMGS0 | RAMM1, ALIGN(64)
   sfra_data    	: > RAMGS0

   dclfuncs  		: > RAMLS4567, align(2)

   ramInitVars 		: > RAMM0
   ramgs0 			: > RAMGS0
   testBufs  		: > RAMGS0
}

//===========================================================================
// End of file.
//===========================================================================
