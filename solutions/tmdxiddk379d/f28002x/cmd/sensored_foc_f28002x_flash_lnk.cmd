//#############################################################################
//
// FILE:    multi_axis_fsi_slave_f28002x_flash_lnk.cmd.cmd
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
   BOOT_RSVD         : origin = 0x00000002, length = 0x00000126
   RAMM0             : origin = 0x00000128, length = 0x000002D8
   RAMM1             : origin = 0x00000400, length = 0x00000380     /* on-chip RAM block M1 */
   BOOT_RSVD_SYSBIOS : origin = 0x00000780, length = 0x00000080

/* RAMLS4            : origin = 0x0000A000, length = 0x00000800 */
/* RAMLS5            : origin = 0x0000A800, length = 0x00000800 */
/* RAMLS6            : origin = 0x0000B000, length = 0x00000800 */
   /* Combining the LS4,5,6 RAMs */
   RAMLS456          : origin = 0x0000A000, length = 0x00001800

   RAMLS7            : origin = 0x0000B800, length = 0x00000800
   RAMGS0            : origin = 0x0000C000, length = 0x00000800

   BEGIN             : origin = 0x00080000, length = 0x00000002

   /* Flash sectors */
   /* BANK 0 */
   FLASH_BANK0_SEC0  : origin = 0x00080002, length = 0x00000FFE	/* on-chip Flash */
   FLASH_BANK0_SEC1  : origin = 0x00081000, length = 0x00001000	/* on-chip Flash */
   FLASH_BANK0_SEC2  : origin = 0x00082000, length = 0x00001000	/* on-chip Flash */
   FLASH_BANK0_SEC3  : origin = 0x00083000, length = 0x00001000	/* on-chip Flash */
   FLASH_BANK0_SEC4  : origin = 0x00084000, length = 0x00001000	/* on-chip Flash */
   FLASH_BANK0_SEC5  : origin = 0x00085000, length = 0x00001000	/* on-chip Flash */
   FLASH_BANK0_SEC6  : origin = 0x00086000, length = 0x00001000	/* on-chip Flash */
   FLASH_BANK0_SEC7  : origin = 0x00087000, length = 0x00001000	/* on-chip Flash */
   FLASH_BANK0_SEC8  : origin = 0x00088000, length = 0x00001000	/* on-chip Flash */
   FLASH_BANK0_SEC9  : origin = 0x00089000, length = 0x00001000	/* on-chip Flash */
   FLASH_BANK0_SEC10 : origin = 0x0008A000, length = 0x00001000	/* on-chip Flash */
   FLASH_BANK0_SEC11 : origin = 0x0008B000, length = 0x00001000	/* on-chip Flash */
   FLASH_BANK0_SEC12 : origin = 0x0008C000, length = 0x00001000	/* on-chip Flash */
   FLASH_BANK0_SEC13 : origin = 0x0008D000, length = 0x00001000	/* on-chip Flash */
   FLASH_BANK0_SEC14 : origin = 0x0008E000, length = 0x00001000	/* on-chip Flash */
   FLASH_BANK0_SEC15 : origin = 0x0008F000, length = 0x00001000	/* on-chip Flash */

/* FLASHBANK1        : origin = 0x00080000, length = 0x00010000 */
/* BOOTROM_EXT       : origin = 0x003F8000, length = 0x00007FC0 */
   BOOTROM     		 : origin = 0x003FF27C, length = 0x00000D44
   RESET             : origin = 0x003FFFC0, length = 0x00000002

   FPUTABLES   		 : origin = 0x003F2A70, length = 0x0000081A	 /* FPU Tables in Boot ROM */
}


SECTIONS
{
   /* Setup for "boot to SARAM" mode:
      The codestart section (found in DSP28_CodeStartBranch.asm)
      re-directs execution to the start of user code.  */
   codestart        : > BEGIN, ALIGN(8)
   .text            : >> FLASH_BANK0_SEC0 | FLASH_BANK0_SEC1 | FLASH_BANK0_SEC2, ALIGN(8)
   .cinit           : > FLASH_BANK0_SEC3, ALIGN(4)
   .switch          : > FLASH_BANK0_SEC3, ALIGN(4)
   .reset           : > RESET,	TYPE = DSECT /* not used, */
   .stack           : > RAMM0

   .bss             : >> RAMLS7 | RAMM1
   .bss:output      : >> RAMLS7 | RAMM1
   .bss:cio         : >> RAMLS7 | RAMM1
   .data            : >> RAMLS7 | RAMM1
   .sysmem          : >> RAMLS7 | RAMM1


   .init_array      : > FLASH_BANK0_SEC4, ALIGN(4)

   /* Initalized sections go in Flash */
   .const           : > FLASH_BANK0_SEC4, ALIGN(4)

   /* Allocate FPU math areas: */
   FPUmathTables    : > FPUTABLES, TYPE = NOLOAD

   .TI.ramfunc : {} LOAD = FLASH_BANK0_SEC5 | FLASH_BANK0_SEC6,
                    RUN = RAMLS456,
                    LOAD_START(RamfuncsLoadStart),
                    LOAD_SIZE(RamfuncsLoadSize),
                    LOAD_END(RamfuncsLoadEnd),
                    RUN_START(RamfuncsRunStart),
                    RUN_SIZE(RamfuncsRunSize),
                    RUN_END(RamfuncsRunEnd),
					ALIGN(2)

    /* SFRA specific sections */
   SFRA_F32_Data    : > RAMGS0, ALIGN(64)
   sfra_data    	: > RAMGS0

   dclfuncs    : {} LOAD = FLASH_BANK0_SEC5 | FLASH_BANK0_SEC6,
                    RUN = RAMLS456,
                    LOAD_START(dclfuncsLoadStart),
                    LOAD_SIZE(dclfuncsLoadSize),
                    LOAD_END(dclfuncsLoadEnd),
                    RUN_START(dclfuncsRunStart),
                    RUN_SIZE(dclfuncsRunSize),
                    RUN_END(dclfuncsRunEnd),
					ALIGN(2)

   ramInitVars 		: > RAMM0
   ramgs0 			: > RAMGS0
   testBufs			: > RAMLS7
}

//===========================================================================
// End of file.
//===========================================================================
