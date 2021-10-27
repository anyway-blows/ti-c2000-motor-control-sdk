/*
// TI File $Revision: /main/3 $
// Checkin $Date: Agu 1, 2017   13:45:43 $
//
// FILE:    F280049_flash_CPU.cmd
//
// TITLE:   Linker Command File For F280049 examples that run out of Flash
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
   /* BEGIN is used for the "boot to Flash" bootloader mode   */
   BEGIN           	: origin = 0x080000, length = 0x000002

   RESET            : origin = 0x3FFFC0, length = 0x000002

   RAMGS1_3         : origin = 0x00E000, length = 0x006000
   RAMLS4_7         : origin = 0x00A000, length = 0x002000

   FLASHB0_SA		: origin = 0x080002, length = 0x00FFFE		/* on-chip Flash */
   FLASHB1_SA   	: origin = 0x090000, length = 0x010000		/* on-chip Flash */


   BOOT_RSVD        : origin = 0x000002, length = 0x0000F3      /* Part of M0, BOOT rom will use this for stack */
   RAMM0           	: origin = 0x0000F5, length = 0x00030B
   RAMM1           	: origin = 0x000400, length = 0x000400

   RAMGS0_A         : origin = 0x00C000, length = 0x002000

   RAMLS0_1         : origin = 0x008000, length = 0x001000		/* Can't be used, reserved for FAST object */
   RAMLS2_3         : origin = 0x009100, length = 0x000F00		/* */

   CLA1MSGRAMLOW    : origin = 0x001480, length = 0x000080
   CLA1MSGRAMHIGH   : origin = 0x001500, length = 0x000080
}

SECTIONS
{
	codestart       : > BEGIN, 			ALIGN(4)

   .TI.ramfunc      : LOAD = FLASHB0_SA,
                      RUN = RAMGS1_3,
                      LOAD_START(RamfuncsLoadStart),
                      LOAD_SIZE(RamfuncsLoadSize),
                      LOAD_END(RamfuncsLoadEnd),
                      RUN_START(RamfuncsRunStart),
                      RUN_SIZE(RamfuncsRunSize),
                      RUN_END(RamfuncsRunEnd),
                      ALIGN(4)

   .text            : > FLASHB0_SA,      ALIGN(4)
   .cinit           : > FLASHB0_SA,      ALIGN(4)
   .switch          : > FLASHB0_SA,      ALIGN(4)
   .const           : > FLASHB0_SA,      ALIGN(4)

   .reset           : > RESET,           TYPE = DSECT
   .stack           : > RAMM0

   .bss             : > RAMGS0_A
   .bss:output      : > RAMGS0_A
   .init_array      : > RAMGS0_A, 		 ALIGN(4)
   .sysmem          : > RAMGS0_A
   .data            : > RAMGS0_A, 		 ALIGN(4)

   .bss_cla         : > RAMLS2_3

   Cla1Prog         : > FLASHB0_SA,
                      RUN = RAMLS4_7,
                      LOAD_START(Cla1ProgLoadStart),
                      RUN_START(Cla1ProgRunStart),
                      LOAD_SIZE(Cla1ProgLoadSize),
                      ALIGN(4)

   Cla1Prog2        : > FLASHB0_SA,
                      RUN = RAMLS4_7,
                      LOAD_START(Cla1Prog2LoadStart),
                      RUN_START(Cla1Prog2RunStart),
                      LOAD_SIZE(Cla1Prog2LoadSize),
                      ALIGN(4)

   Cla1ToCpuMsgRAM  : > CLA1MSGRAMLOW
   CpuToCla1MsgRAM  : > CLA1MSGRAMHIGH

   .const_cla       : > FLASHB0_SA,
                      RUN = RAMLS4_7,
                      RUN_START(Cla1ConstRunStart),
                      LOAD_START(Cla1ConstLoadStart),
                      LOAD_SIZE(Cla1ConstLoadSize)

}

SECTIONS
{
	sysctrl_data	: > RAMGS0_A
	ctrl_data		: > RAMGS0_A
	est_data		: > RAMGS0_A

}

SECTIONS
{
	datalog_data	: > RAMGS0_A
	graph_data		: > RAMGS0_A
}
