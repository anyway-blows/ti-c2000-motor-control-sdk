//#############################################################################
//
// FILE:    multi_axis_lead_ram_lnk_cpu1.cmd
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
// peripheral structures to the proper locations within the memory map.
//
// The header linker files are found in <base>\2838x_headers\cmd
//
// For BIOS applications add:      f2838x_Headers_BIOS.cmd
// For nonBIOS applications add:   f2838x_Headers_nonBIOS.cmd
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
   BOOT_RSVD        : origin = 0x000002, length = 0x0001AE     /* Part of M0, BOOT rom will use this for stack */
   RAMM0            : origin = 0x0001B1, length = 0x00024F
   RAMM1            : origin = 0x000400, length = 0x0003F8     /* on-chip RAM block M1 */
/* RAMM1_RSVD       : origin = 0x0007F8, length = 0x000008 */  /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */

   RAMD0            : origin = 0x00C000, length = 0x000800
   RAMD1            : origin = 0x00C800, length = 0x000800
   RAMLS0           : origin = 0x008000, length = 0x000800
   RAMLS1           : origin = 0x008800, length = 0x000800
   RAMLS2           : origin = 0x009000, length = 0x000800
   RAMLS3           : origin = 0x009800, length = 0x000800
   RAMLS4           : origin = 0x00A000, length = 0x000800
   RAMLS5           : origin = 0x00A800, length = 0x000800
   RAMLS6           : origin = 0x00B000, length = 0x000800
   RAMLS7           : origin = 0x00B800, length = 0x000800
   RAMGS0           : origin = 0x00D000, length = 0x001000
   RAMGS1           : origin = 0x00E000, length = 0x001000
   RAMGS2           : origin = 0x00F000, length = 0x001000
/* RAMGS4           : origin = 0x011000, length = 0x001000 */
/* RAMGS5           : origin = 0x012000, length = 0x001000 */
/* RAMGS6           : origin = 0x013000, length = 0x001000 */
/* RAMGS7           : origin = 0x014000, length = 0x001000 */
/* RAMGS8           : origin = 0x015000, length = 0x001000 */
/* RAMGS9           : origin = 0x016000, length = 0x001000 */
/* RAMGS10          : origin = 0x017000, length = 0x001000 */
/* RAMGS11          : origin = 0x018000, length = 0x001000 */
/* RAMGS12          : origin = 0x019000, length = 0x001000 */
/* RAMGS13          : origin = 0x01A000, length = 0x001000 */
/* RAMGS14          : origin = 0x01B000, length = 0x001000 */
   RAMGS3_14        : origin = 0x010000, length = 0x00C000
   RAMGS15          : origin = 0x01C000, length = 0x000FF8
/* RAMGS15_RSVD     : origin = 0x01CFF8, length = 0x000008 */  /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */


   CPU1TOCPU2RAM    : origin = 0x03A000, length = 0x000800
   CPU2TOCPU1RAM    : origin = 0x03B000, length = 0x000800

   CPUTOCMRAM       : origin = 0x039000, length = 0x000200
   CMTOCPURAM       : origin = 0x038000, length = 0x000200

   CPUTOCMRAM_ECAT  : origin = 0x039200, length = 0x000200
   CMTOCPURAM_ECAT  : origin = 0x038200, length = 0x000200

   CANA_MSG_RAM     : origin = 0x049000, length = 0x000800
   CANB_MSG_RAM     : origin = 0x04B000, length = 0x000800
   RESET           	: origin = 0x3FFFC0, length = 0x000002

   CLA1_MSGRAMLOW   : origin = 0x001480, length = 0x000080
   CLA1_MSGRAMHIGH  : origin = 0x001500, length = 0x000080
}


SECTIONS
{
   codestart        : >  BEGIN
   .text            : >> RAMLS4 | RAMLS5 | RAMLS6 | RAMLS7 | RAMD0 | RAMD1
   .cinit           : >  RAMLS4 | RAMLS5 | RAMLS6 | RAMLS7 | RAMD0 | RAMD1
   .switch          : >  RAMM0
   .reset           : >  RESET, TYPE = DSECT /* not used, */
   .stack           : >  RAMM1

   .init_array      : >> RAMLS4 | RAMLS5 | RAMLS6 | RAMLS7 | RAMD0 | RAMD1, ALIGN(8)
   .bss             : >> RAMLS2 | RAMLS3
   .bss:output      : >> RAMLS2 | RAMLS3
   .const           : >> RAMLS4 | RAMLS5 | RAMLS6 | RAMLS7 | RAMD0 | RAMD1
   .data            : >> RAMLS2 | RAMLS3
   .sysmem          : >> RAMLS2 | RAMLS3

   MSGRAM_CPU1_TO_CPU2   > CPU1TOCPU2RAM, type=NOINIT
   MSGRAM_CPU2_TO_CPU1   > CPU2TOCPU1RAM, type=NOINIT

   MSGRAM_CPU_TO_CM      > CPUTOCMRAM, type=NOINIT
   MSGRAM_CM_TO_CPU      > CMTOCPURAM, type=NOINIT

   MSGRAM_CPU_TO_CM_ECAT > CPUTOCMRAM_ECAT, type=NOINIT
   MSGRAM_CM_TO_CPU_ECAT > CMTOCPURAM_ECAT, type=NOINIT

    /* CLA specific sections */
   Cla1Prog         : >> RAMLS0

   ClaData			: >> RAMLS1

   Cla1ToCpuMsgRAM  : > CLA1_MSGRAMLOW,  type=NOINIT
   CpuToCla1MsgRAM  : > CLA1_MSGRAMHIGH, type=NOINIT

   ramInitVars 		: > RAMM0
   ramgs0 			: > RAMGS0, type=NOINIT
   ramgs1 			: > RAMGS1, type=NOINIT
   fsi_data			: > RAMGS2
   {
   		multi_axis_lead_comms.obj (.bss)
   }

   /* SFRA specific sections */
   SFRA_F32_Data	: > RAMGS2, ALIGN = 64
   DLOG_Data		: > RAMGS3_14, ALIGN = 64

#ifdef CLA_C
   /* CLA C compiler sections */
   //
   // Must be allocated to memory the CLA has write access to
   //
   CLAscratch       :
                     { *.obj(CLAscratch)
                     . += CLA_SCRATCHPAD_SIZE;
                     *.obj(CLAscratch_end) } >  RAMLS1

   .scratchpad      : > RAMLS1
   .bss_cla		    : > RAMLS1
   .const_cla	    : > RAMLS0
#endif //CLA_C

	dclfuncs  		: >> RAMLS4 | RAMLS5 | RAMLS6 | RAMLS7 | RAMD0 | RAMD1, align(2)
    .TI.ramfunc 	: >> RAMLS4 | RAMLS5 | RAMLS6 | RAMLS7 | RAMD0 | RAMD1

}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
