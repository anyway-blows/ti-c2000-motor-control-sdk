// TI File $Revision:
// Checkin $Date:
//###########################################################################
//
// FILE:    C28M35n_28x_RAM_lnk.cmd
//
// TITLE:   Linker Command File For 28M35n examples that run out of RAM
//
//          This ONLY includes all SARAM blocks on the 28M35n device.
//          This does not include flash or OTP.
//
//          Keep in mind that L0 and L1 are protected by the code
//          security module.
//
//          What this means is in most cases you will want to move to
//          another memory map file which has more memory defined.
//
//###########################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:28 CDT 2021 $
//###########################################################################

/* ======================================================
// In addition to this memory linker command file,
// add the header linker command file directly to the project.
// The header linker command file is required to link the
// peripheral structures to the proper locations within
// the memory map.
//
// The header linker files are found in <base>\28M35x_headers\cmd
//
// For BIOS applications add:      F2837x_Headers_BIOS.cmd
// For nonBIOS applications add:   F2837x_Headers_nonBIOS.cmd
========================================================= */

CLA_SCRATCHPAD_SIZE = 0x100;
--undef_sym=__cla_scratchpad_end
--undef_sym=__cla_scratchpad_start

MEMORY
{
PAGE 0 :
   /* BEGIN is used for the "boot to SARAM" bootloader mode   */

   BEGIN           	: origin = 0x000000, length = 0x000002
   RAMM0           	: origin = 0x000002, length = 0x0003FE
   RAMLS0LS1     : origin = 0x008000, length = 0x001000

   RAMLS4LS5		   : origin = 0x00A000, length = 0x001000

   RAMGS456789		: origin = 0x010000, length = 0x006000
   RAMGS2GS3      : origin = 0x00E000, length = 0x002000

   RESET           	: origin = 0x3FFFC0, length = 0x000002

PAGE 1 :

   BOOT_RSVD       : origin = 0x000002, length = 0x00004E     /* Part of M0, BOOT rom will use this for stack */
   RAMM1           : origin = 0x000400, length = 0x000400     /* on-chip RAM block M1 */
   CLA1_MSGRAMLOW  : origin = 0x001480, length = 0x000080
   CLA1_MSGRAMHIGH : origin = 0x001500, length = 0x000080

   RAMD0D1     : origin = 0x00B000, length = 0x001000

   RAMLS2		: origin = 0x009000, length = 0x000800
   RAMLS3		: origin = 0x009800, length = 0x000800

   RAMGS0GS1      : origin = 0x00C000, length = 0x002000

//   RAMGS3      : origin = 0x00F000, length = 0x001000
   RAMGS10     : origin = 0x016000, length = 0x001000
   RAMGS11     : origin = 0x017000, length = 0x001000
   RAMGS12     : origin = 0x018000, length = 0x001000
   RAMGS13     : origin = 0x019000, length = 0x001000
   RAMGS14     : origin = 0x01A000, length = 0x001000
   RAMGS15     : origin = 0x01B000, length = 0x001000
   
   CPU2TOCPU1RAM   : origin = 0x03F800, length = 0x000400
   CPU1TOCPU2RAM   : origin = 0x03FC00, length = 0x000400

   CLA1DEBUG    : origin = 0x001440, length = 0x000040     /* CLA Debug registers */

}



SECTIONS
{
   codestart        : > BEGIN,     PAGE = 0
   ramfuncs         : > RAMGS2GS3      PAGE = 0
   .TI.ramfunc 		: > RAMGS2GS3      PAGE = 0

   .text            : >>RAMLS0LS1|RAMGS456789,   PAGE = 0
   .cinit           : > RAMM0,     PAGE = 0
   .pinit           : > RAMM0,     PAGE = 0
   .switch          : > RAMM0,     PAGE = 0
   .reset           : > RESET,     PAGE = 0, TYPE = DSECT /* not used, */

   .stack           : > RAMM1,     PAGE = 1
   .ebss            : > RAMD0D1|RAMGS0GS1,     PAGE = 1
   .econst          : > RAMD0D1|RAMGS10,     PAGE = 1
   .esysmem         : > RAMD0D1,     PAGE = 1
   Filter_RegsFile  : > RAMGS0GS1,	   PAGE = 1

   //warn this is not right!
   Cla1Prog         : > RAMLS4LS5, PAGE=0

   ClaData          : > RAMLS3, PAGE=1, ALIGN=2

   Cla1ToCpuMsgRAM  : > CLA1_MSGRAMLOW,   PAGE = 1
   CpuToCla1MsgRAM  : > CLA1_MSGRAMHIGH,  PAGE = 1
   //
   // Must be allocated to memory the CLA has write access to
   //
   CLAscratch       :
                        { *.obj(CLAscratch)
                        . += CLA_SCRATCHPAD_SIZE;
                        *.obj(CLAscratch_end) } >  RAMLS2,  PAGE = 1, ALIGN=2

   .bss_cla		    : > RAMLS2,   PAGE = 1
   .const_cla	    : > RAMLS2,   PAGE = 1
   Cla1DebugRegsFile     : > CLA1DEBUG,    PAGE = 1

   /* The following section definitions are required when using the IPC API Drivers */ 
    GROUP : > CPU1TOCPU2RAM, PAGE = 1 
    {
        PUTBUFFER 
        PUTWRITEIDX 
        GETREADIDX 
    }
    
    GROUP : > CPU2TOCPU1RAM, PAGE = 1
    {
        GETBUFFER :    TYPE = DSECT
        GETWRITEIDX :  TYPE = DSECT
        PUTREADIDX :   TYPE = DSECT
    }  
    
}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
