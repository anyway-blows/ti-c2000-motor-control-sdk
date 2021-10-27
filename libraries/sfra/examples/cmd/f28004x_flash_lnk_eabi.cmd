
MEMORY
{
   /* BEGIN is used for the "boot to Flash" bootloader mode   */

   BEGIN           	: origin = 0x080000, length = 0x000002
   RAMM0           	: origin = 0x0000F5, length = 0x00030B

   RAMLS2LS3LS4LS5  : origin = 0x009000, length = 0x001800

   RAMLS6      : origin = 0x00B000, length = 0x000800

   RAMGS0GS1        : origin = 0x00C000, length = 0x002000


   RESET           	: origin = 0x3FFFC0, length = 0x000002

   /* Flash sectors */
   /* BANK 0 */
   FLASH_BANK0_SEC0  : origin = 0x080002, length = 0x000FFE	/* on-chip Flash */
   FLASH_BANK0_SEC1  : origin = 0x081000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK0_SEC2  : origin = 0x082000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK0_SEC3  : origin = 0x083000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK0_SEC4  : origin = 0x084000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK0_SEC5  : origin = 0x085000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK0_SEC6  : origin = 0x086000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK0_SEC7  : origin = 0x087000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK0_SEC8  : origin = 0x088000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK0_SEC9  : origin = 0x089000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK0_SEC10 : origin = 0x08A000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK0_SEC11 : origin = 0x08B000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK0_SEC12 : origin = 0x08C000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK0_SEC13 : origin = 0x08D000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK0_SEC14 : origin = 0x08E000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK0_SEC15 : origin = 0x08F000, length = 0x001000	/* on-chip Flash */

   /* BANK 1 */
   FLASH_BANK1_SEC0  : origin = 0x090000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC1_2_3_4  : origin = 0x091000, length = 0x004000	/* on-chip Flash */
   FLASH_BANK1_SEC5  : origin = 0x095000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC6  : origin = 0x096000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC7  : origin = 0x097000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC8_9_10  : origin = 0x098000, length = 0x03000	/* on-chip Flash */
   FLASH_BANK1_SEC11 : origin = 0x09B000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC12 : origin = 0x09C000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC13 : origin = 0x09D000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC14 : origin = 0x09E000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC15 : origin = 0x09F000, length = 0x001000	/* on-chip Flash */

   BOOT_RSVD       : origin = 0x000002, length = 0x0000F3     /* Part of M0, BOOT rom will use this for stack */

   RAMM1           : origin = 0x000400, length = 0x000400     /* on-chip RAM block M1 */

   RAMLS0LS1       : origin = 0x008000, length = 0x001000

   RAMLS7      : origin = 0x00B800, length = 0x000800

   RAMGS2      : origin = 0x010000, length = 0x002000
   RAMGS3      : origin = 0x012000, length = 0x002000
}


SECTIONS
{
   codestart        : > BEGIN,     ALIGN(4)
   .text            : >>FLASH_BANK1_SEC1_2_3_4,  ALIGN(4)
   .cinit           : > FLASH_BANK1_SEC1_2_3_4,  ALIGN(4)
   .pinit           : > FLASH_BANK0_SEC1,    ALIGN(4)
   .switch          : > FLASH_BANK0_SEC1,    ALIGN(4)
   .reset           : > RESET,    TYPE = DSECT /* not used, */

   .stack           : > RAMM1

#if defined(__TI_EABI__)
   GROUP
   {
       .TI.ramfunc
       {
       }
       ramfuncs

   } LOAD = FLASH_BANK0_SEC6,
         RUN = RAMGS0GS1,
         LOAD_START(RamfuncsLoadStart),
         LOAD_SIZE(RamfuncsLoadSize),
         LOAD_END(RamfuncsLoadEnd),
         RUN_START(RamfuncsRunStart),
         RUN_SIZE(RamfuncsRunSize),
         RUN_END(RamfuncsRunEnd),
         ALIGN(4)


    GROUP
    {
        isrcodefuncs
        dclfuncs
    }    LOAD = FLASH_BANK1_SEC8_9_10,
         RUN =  RAMLS2LS3LS4LS5,
         LOAD_START(isrcodefuncsLoadStart),
         LOAD_SIZE(isrcodefuncsLoadSize),
         LOAD_END(isrcodefuncsLoadEnd),
         RUN_START(isrcodefuncsRunStart),
         RUN_SIZE(isrcodefuncsRunSize),
         RUN_END(isrcodefuncsRunEnd),
         ALIGN(4)

    .const_cla       :  LOAD = FLASH_BANK1_SEC8_9_10,
                       RUN = RAMLS2LS3LS4LS5,
                       RUN_START(Cla1ConstRunStart),
                       LOAD_START(Cla1ConstLoadStart),
                       LOAD_SIZE(Cla1ConstLoadSize)

                       /* CLA specific sections */
    Cla1Prog        : LOAD = FLASH_BANK1_SEC8_9_10,
                      RUN = RAMLS2LS3LS4LS5,
                      LOAD_START(Cla1ProgLoadStart),
                      RUN_START(Cla1ProgRunStart),
                      LOAD_SIZE(Cla1ProgLoadSize),
                      ALIGN(4)


    .bss            : >> RAMGS3
    .data 			: > RAMGS3 | RAMLS7 | RAMGS2

    .init_array      : > FLASH_BANK0_SEC9, ALIGN(4)

     /* Initalized sections go in Flash */
   .const           : > FLASH_BANK0_SEC9, ALIGN(4)
#else
	GROUP
   {
       .TI.ramfunc
       {
       }
       ramfuncs

   } LOAD = FLASH_BANK0_SEC6,
         RUN = RAMGS0GS1,
         LOAD_START(_RamfuncsLoadStart),
         LOAD_SIZE(_RamfuncsLoadSize),
         LOAD_END(_RamfuncsLoadEnd),
         RUN_START(_RamfuncsRunStart),
         RUN_SIZE(_RamfuncsRunSize),
         RUN_END(_RamfuncsRunEnd),
         ALIGN(4)

   GROUP
    {
        isrcodefuncs
        dclfuncs
    }    LOAD = FLASH_BANK1_SEC8_9_10,
         RUN =  RAMLS2LS3LS4LS5,
         LOAD_START(_isrcodefuncsLoadStart),
         LOAD_SIZE(_isrcodefuncsLoadSize),
         LOAD_END(_isrcodefuncsLoadEnd),
         RUN_START(_isrcodefuncsRunStart),
         RUN_SIZE(_isrcodefuncsRunSize),
         RUN_END(_isrcodefuncsRunEnd),
         ALIGN(4)

	.const_cla       :  LOAD = FLASH_BANK1_SEC8_9_10,
	           RUN = RAMLS2LS3LS4LS5,
	           RUN_START(_Cla1ConstRunStart),
	           LOAD_START(_Cla1ConstLoadStart),
	           LOAD_SIZE(_Cla1ConstLoadSize),

                       /* CLA specific sections */
    Cla1Prog        : LOAD = FLASH_BANK1_SEC8_9_10,
                      RUN = RAMLS2LS3LS4LS5,
                      LOAD_START(_Cla1ProgLoadStart),
                      RUN_START(_Cla1ProgRunStart),
                      LOAD_SIZE(_Cla1ProgLoadSize),
                      ALIGN(4)

    .ebss            : >> RAMGS3
    .esysmem         : > RAMGS2
    .econst          : > FLASH_BANK0_SEC4, ALIGN(4)

#endif

    SFRA_F32_Data		: > RAMGS2, ALIGN = 64

    SFRA_Data		: > RAMGS2, ALIGN = 64

 	FPUmathTables	: > RAMGS3 | RAMGS2

   .scratchpad      : > RAMLS0LS1
   .bss_cla         : > RAMLS0LS1
   controlVariables : > RAMLS0LS1

}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
