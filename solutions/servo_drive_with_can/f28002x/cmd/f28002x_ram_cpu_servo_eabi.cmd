/*
// FILE:    f28002x_ram_cpu_servo_eabi.cmd
//
// TITLE:   Linker Command File For F280025 examples that run out of Flash
//
//
//          Keep in mind that L4, L5,L6 and L7 are protected by the code
//          security module.
//
//          What this means is in most cases you will want to move to
//          another memory map file which has more memory defined.
//
*/

MEMORY
{
   BEGIN           	  : origin = 0x00000000, length = 0x00000002
   BOOT_RSVD	      : origin = 0x00000002, length = 0x00000126

/* RAMLS4        	  : origin = 0x0000A000, length = 0x00000800 */
/* RAMLS5        	  : origin = 0x0000A800, length = 0x00000800 */
/* RAMLS6    		  : origin = 0x0000B000, length = 0x00000800 */
/* RAMLS7        	  : origin = 0x0000B800, length = 0x00000800 */
   RAMLS4567    	  : origin = 0x0000A000, length = 0x00002300

   BOOTROM			  : origin = 0x003F0000, length = 0x00008000
   BOOTROM_EXT		  : origin = 0x003F8000, length = 0x00007FC0
   RESET           	  : origin = 0x003FFFC0, length = 0x00000002

   RAMM0S          	  : origin = 0x00000128, length = 0x00000118	/* stack */
   RAMM1D         	  : origin = 0x00000240, length = 0x000005B8	/* on-chip RAM block M0 part & M1 */
   RAMM1_RSVD         : origin = 0x000007F8, length = 0x00000008    /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */

   RAMGS0     	   	  : origin = 0x0000C300, length = 0x000004F8
   RAMGS0_RSVD        : origin = 0x0000C7F8, length = 0x00000008    /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */
}


SECTIONS
{
   .reset           : > RESET, 				TYPE = DSECT
   codestart		: > BEGIN,     		 	ALIGN(4)
   .TI.ramfunc      : > RAMLS4567

   .text            : > RAMLS4567,			ALIGN(8)
   .cinit           : > RAMLS4567,			ALIGN(4)
   .switch          : > RAMLS4567,			ALIGN(4)
   .cio				: > RAMLS4567
   .pinit           : > RAMLS4567,			ALIGN(4)
   .const           : > RAMLS4567,  		ALIGN(4)
   .init_array      : > RAMLS4567, 			ALIGN(4)

   .stack           : > RAMM0S
   .bss             : > RAMM1D
   .bss:output      : > RAMM1D
   .bss:cio         : > RAMM1D
   .data            : > RAMM1D
   .sysmem          : > RAMM1D
}

SECTIONS
{
    prms_data 		      : > RAMLS4567

    user_data 		      : > RAMM1D
	foc_data 		      : > RAMM1D
    ctrl_data		      : > RAMM1D
}

SECTIONS
{
	vibc_data	          : > RAMGS0
	dmaBuf_data			  : > RAMGS0

    datalog_data	      : > RAMGS0

    sfra_data			  : > RAMGS0
    SFRA_F32_Data		  : > RAMGS0

    graph_data		  	  : > RAMGS0
}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
