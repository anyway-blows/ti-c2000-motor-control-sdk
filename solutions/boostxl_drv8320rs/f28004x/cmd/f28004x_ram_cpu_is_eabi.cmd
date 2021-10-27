/*
// TI File $Revision: /main/3 $
// Checkin $Date: Agu 1, 2017   13:45:43 $
//
// FILE:    F280049_RAM_CPU.cmd
//
// TITLE:   Linker Command File For F280049 examples that run out of RAM
//
//
//          Keep in mind that L0,L1,L2,L3 and L4 are protected by the code
//          security module.
//
//          What this means is in most cases you will want to move to
//          another memory map file which has more memory defined.
//
*/

/*========================================================= */
/* Define the memory block start/length for the F2806x
   PAGE 0 will be used to organize program sections
   PAGE 1 will be used to organize data sections

   Notes:
         Memory blocks on F280049 are uniform (ie same
         physical memory) in both PAGE 0 and PAGE 1.
         That is the same memory region should not be
         defined for both PAGE 0 and PAGE 1.
         Doing so will result in corruption of program
         and/or data.

         Contiguous SARAM memory blocks can be combined
         if required to create a larger memory block.
*/

MEMORY
{
PAGE 0 :
PAGE 1 :
   BEGIN           	: origin = 0x000000, length = 0x000002
   RESET            : origin = 0x3FFFC0, length = 0x000002

   RAMGS1_3         : origin = 0x00E000, length = 0x006000
   RAMLS4_7         : origin = 0x00A000, length = 0x002000

/*   FLASHB0_SA		: origin = 0x080002, length = 0x00FFFE	*/	/* on-chip Flash */
/*   FLASHB1_SA   	: origin = 0x090000, length = 0x010000	*/	/* on-chip Flash */

   BOOT_RSVD        : origin = 0x000002, length = 0x0000F3      /* Part of M0, BOOT rom will use this for stack */
   RAMM0_1        	: origin = 0x0000F5, length = 0x00070B

/* CLA1             : origin = 0x001400, length = 0x000080	*/ /* Defined in headers cmd file*/

   RAMGS0_A         : origin = 0x00C000, length = 0x002000

   RAMLS0_1         : origin = 0x008000, length = 0x001000		/* Can't be used, reserved for FAST object */
   RAMLS2_3         : origin = 0x009100, length = 0x000F00		/* */

   CLA1MSGRAMLOW    : origin = 0x001480, length = 0x000080
   CLA1MSGRAMHIGH   : origin = 0x001500, length = 0x000080

}

SECTIONS
{
   codestart        : > BEGIN,     		 ALIGN(4)
   .TI.ramfunc      : > RAMLS4_7,        ALIGN(4)
   .text            : > RAMGS1_3,        ALIGN(4)
   .cinit           : > RAMGS1_3,        ALIGN(4)

   .switch          : > RAMGS1_3,        ALIGN(4)
   .const           : > RAMGS1_3
   .data            : > RAMGS1_3, 		 ALIGN(4)

   .reset           : > RESET,           TYPE = DSECT

	/* Digital Controller Library functions */
	dclfuncs		: > RAMGS1_3
	dcl32funcs		: > RAMGS1_3

   .stack           : > RAMM0_1
   .bss             : > RAMGS0_A
   .bss:output      : > RAMGS0_A
   .init_array      : > RAMGS0_A, 		 ALIGN(4)
   .sysmem          : > RAMGS0_A

   .bss_cla         : > RAMLS2_3
   ClaData          : > RAMLS2_3, 		 ALIGN=2

   Cla1Prog         : > RAMLS4_7
   Cla1Prog2        : > RAMLS4_7


   Cla1ToCpuMsgRAM  : > CLA1MSGRAMLOW
   CpuToCla1MsgRAM  : > CLA1MSGRAMHIGH

   .const_cla       : > RAMLS2_3

   .scratchpad      : > RAMLS2_3
}

SECTIONS
{
	sysctrl_data	: > RAMGS0_A
	ctrl_data		: > RAMM0_1
}

SECTIONS
{
	datalog_data	: > RAMGS0_A
	graph_data		: > RAMGS0_A
}
