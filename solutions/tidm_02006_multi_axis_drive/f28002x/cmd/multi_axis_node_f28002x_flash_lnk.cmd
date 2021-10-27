/*
// TI File $Revision: /main/3 $
// Checkin $Date: Agu 1, 2017   13:45:43 $
//
// FILE:    F280025_RAM_CPU.cmd
//
// TITLE:   Linker Command File For F280025 examples that run out of Flash
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
/* Define the memory block start/length for the F28002x
   PAGE 0 will be used to organize program sections
   PAGE 1 will be used to organize data sections

   Notes:
         Memory blocks on F280025 are uniform (ie same
         physical memory) in both PAGE 0 and PAGE 1.
         That is the same memory region should not be
         defined for both PAGE 0 and PAGE 1.
         Doing so will result in corruption of program
         and/or data.

         Contiguous SARAM memory blocks can be combined
         if required to create a larger memory block.
*///#############################################################################
// This file belongs to the DCSM testing project for f28002x device.
// It is intended to be a part of the test directory only.
//#############################################################################
//                              DISCLAIMER
//#############################################################################

-stack 0x120

MEMORY
{
   BEGIN           	: origin = 0x00080000, length = 0x000002
   BOOT_RSVD		: origin = 0x00000000, length = 0x00000050

   RAMLS4P        	  : origin = 0x0000A000, length = 0x00000800
/* RAMLS6    		  : origin = 0x0000B000, length = 0x00000800 */
/* RAMLS7        	  : origin = 0x0000B800, length = 0x00000800 */
   RAMLS67    		  : origin = 0x0000AF00, length = 0x00001100

/* FLASHBANK0_SECT0	  : origin = 0x00080002, length = 0x00000FFE */
   FLASHBANK0_BOOT	  : origin = 0x00080002, length = 0x00000FFE	/* remote update */
/* FLASHBANK0_SECT1	  : origin = 0x00081000, length = 0x00001000 */
/* FLASHBANK0_SECT2	  : origin = 0x00082000, length = 0x00001000 */
/* FLASHBANK0_SECT3	  : origin = 0x00083000, length = 0x00001000 */
/* FLASHBANK0_SECT4	  : origin = 0x00084000, length = 0x00001000 */
/* FLASHBANK0_SECT5	  : origin = 0x00085000, length = 0x00001000 */
/* FLASHBANK0_SECT6	  : origin = 0x00086000, length = 0x00001000 */
/* FLASHBANK0_SECT7	  : origin = 0x00087000, length = 0x00001000 */
/* FLASHBANK0_SECT8	  : origin = 0x00088000, length = 0x00001000 */
/* FLASHBANK0_SECT9	  : origin = 0x00089000, length = 0x00001000 */
/* FLASHBANK0_SECT10  : origin = 0x0008A000, length = 0x00001000 */
/* FLASHBANK0_SECT11  : origin = 0x0008B000, length = 0x00001000 */
/* FLASHBANK0_SECT12  : origin = 0x0008C000, length = 0x00001000 */
/* FLASHBANK0_SECT13  : origin = 0x0008D000, length = 0x00001000 */
/* FLASHBANK0_SECT14  : origin = 0x0008E000, length = 0x00001000 */
   FLASHBANK0_CODE	  : origin = 0x00081000, length = 0x0000E000	/* control code */

/* FLASHBANK0_SECT15  : origin = 0x0008F000, length = 0x00001000 */
   FLASHBANK0_DATA 	  : origin = 0x0008F000, length = 0x00001000	/* constant data */

   TI_OTP1			  : origin = 0x00070000, length = 0x000003F0
   TI_OTP1_SECDC	  : origin = 0x000703F0, length = 0x00000010
   USER_OTP1		  : origin = 0x00078000, length = 0x00000400

   SCC_Z1			  : origin = 0x003E8000, length = 0x00002800
   SCC_Z2			  : origin = 0x003EA800, length = 0x00000800
   INSTASPIN_CODE	  : origin = 0x003EB000, length = 0x00005000	/* Reserve for InstaSPIN */

   RESET           	  : origin = 0x003FFFC0, length = 0x00000002

   RAMM0S          	  : origin = 0x00000050, length = 0x00000130	/* stack */
   RAMM1D         	  : origin = 0x00000180, length = 0x00000680	/* on-chip RAM block M1 */
   RAMGS0     	   	  : origin = 0x0000C000, length = 0x00000800

   RAMLS5        	  : origin = 0x0000A800, length = 0x00000700	/* Reserve for InstaSPIN */

   FPUTABLES   		  : origin = 0x003F2A70, length = 0x0000081A	/* FPU Tables in Boot ROM */
}


SECTIONS
{
   .reset           : > RESET, 				   	TYPE = DSECT
   codestart		: > BEGIN,     		 	   	ALIGN(4)

   GROUP
   {
       .TI.ramfunc {
#if defined(SFRA_ENABLE)
       -l sfra_f32_tmu_coff.lib<sfra_f32_tmu_collect.obj> (.text)
       -l sfra_f32_tmu_coff.lib<sfra_f32_tmu_inject.obj> (.text)
#endif
       }
       ramfuncs
       	/* Digital Controller Library functions */
       dclfuncs
       dcl32funcs
   }          LOAD = FLASHBANK0_CODE
              RUN = RAMLS67,
              LOAD_START(RamfuncsLoadStart),
              LOAD_SIZE(RamfuncsLoadSize),
              LOAD_END(RamfuncsLoadEnd),
              RUN_START(RamfuncsRunStart),
              RUN_SIZE(RamfuncsRunSize),
              RUN_END(RamfuncsRunEnd),
              ALIGN(2)

	ctrlfuncs : {
	            }
	          LOAD = FLASHBANK0_CODE
              RUN = RAMLS4P,
              LOAD_START(ctrlfuncsLoadStart),
              LOAD_SIZE(ctrlfuncsLoadSize),
              LOAD_END(ctrlfuncsLoadEnd),
              RUN_START(ctrlfuncsRunStart),
              RUN_SIZE(ctrlfuncsRunSize),
              RUN_END(ctrlfuncsRunEnd),
              ALIGN(2)

   .text            : > FLASHBANK0_CODE,	ALIGN(8)
   .cinit           : > FLASHBANK0_CODE,	ALIGN(4)
   .switch          : > FLASHBANK0_CODE,	ALIGN(4)
   .cio				: > FLASHBANK0_CODE
   .pinit           : > FLASHBANK0_CODE,	ALIGN(4)
   .const           : > FLASHBANK0_CODE,  	ALIGN(4)
   .init_array      : > FLASHBANK0_CODE, 	ALIGN(4)

   .stack           : > RAMM0S
   .bss             : > RAMM1D
   .bss:output      : > RAMM1D
   .bss:cio         : > RAMM1D
   .data            : > RAMM1D
   .sysmem          : > RAMM1D

   ramInitVars 		: > RAMM0S

	b0_ti_otp_data			 : > TI_OTP1
	b0_ti_otp_data_secdc	 : > TI_OTP1_SECDC
	b0_user_otp_data 		 : > USER_OTP1
    .zone1_execute_code		 : > RAMLS4P
	.zone2_execute_code		 : > RAMLS4P

    SCCLZ1					 : > SCC_Z1
    SCCLZ2					 : > SCC_Z2
    .secrom_code			 : > INSTASPIN_CODE

    .data_flashbank1_sectb	 : > FLASHBANK0_DATA

	/*  Allocate IQ math areas: */
    IQmath           		 : > FLASHBANK0_CODE
   	IQmathTables     		 : > FLASHBANK0_CODE

	/* Allocate FPU math areas: */
   	FPUmathTables    		 : > FPUTABLES, TYPE = NOLOAD
}

SECTIONS
{
    prms_data 		      : > FLASHBANK0_DATA

    user_data 		      : > RAMM1D
	sys_data     	      : > RAMM1D

    ctrl_data		      : > RAMM1D
	foc_data 		      : > RAMM1D
    motor_data		      : > RAMGS0
/*   #ifdef FSI_ENABLE */
    fsi_data	          : > RAMGS0
    {
		multi_axis_node_comms.obj (.bss)
    }
/*    #endif  */

}

SECTIONS
{
   /* SFRA specific sections */
    SFRA_F32_Data    	  : > RAMGS0, ALIGN = 64
	sfra_data	          : > RAMGS0
	vibc_data	          : > RAMGS0
	datalog_data	      : > RAMGS0
	dmaBuf_data			  : > RAMGS0
}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
