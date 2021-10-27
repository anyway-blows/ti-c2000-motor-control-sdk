
MEMORY
{
PAGE 0 :
   /* BEGIN is used for the "boot to Flash" bootloader mode   */

   BEGIN           	: origin = 0x08EFF0, length = 0x000002
   RAMM0           	: origin = 0x0000F4, length = 0x00030C

   RAMLS03        	: origin = 0x008000, length = 0x002000
   RAMLS4      		: origin = 0x00A000, length = 0x000800
   RESET           	: origin = 0x3FFFC0, length = 0x000002

   /* Flash sectors */
   /* BANK 0 */
   FLASH_BANK0_SEC0  : origin = 0x080000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK0_SEC1  : origin = 0x081000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK0_SEC2  : origin = 0x082008, length = 0x000FF8	/* on-chip Flash */
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
   FLASH_BANK0_SEC14 : origin = 0x08E000, length = 0x000FF0	/* on-chip Flash */
   FLASH_BANK0_SEC15 : origin = 0x08EFF2, length = 0x00100E	/* on-chip Flash */

   /* BANK 1 */
   FLASH_BANK1_SEC0  : origin = 0x090000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC1  : origin = 0x091000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC2  : origin = 0x092000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC3  : origin = 0x093000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC4  : origin = 0x094000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC5  : origin = 0x095000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC6  : origin = 0x096000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC7  : origin = 0x097000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC8  : origin = 0x098000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC9  : origin = 0x099000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC10 : origin = 0x09A000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC11 : origin = 0x09B000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC12 : origin = 0x09C000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC13 : origin = 0x09D000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC14 : origin = 0x09E000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC15 : origin = 0x09F000, length = 0x000FF0	/* on-chip Flash */

//   FLASH_BANK1_SEC15_RSVD : origin = 0x09FFF0, length = 0x000010  /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */

PAGE 1 :

   BOOT_RSVD       : origin = 0x000002, length = 0x0000F1     /* Part of M0, BOOT rom will use this for stack */
   RAMM1           : origin = 0x000400, length = 0x000360     /* on-chip RAM block M1 */
   FLASH_API_ROM_RSVD : origin = 0x760, length = 0x000020     /* Required by Flash API from ROM */

   RAMLS5      : origin = 0x00A800, length = 0x000800
   RAMLS6      : origin = 0x00B000, length = 0x000800
   RAMLS7      : origin = 0x00B800, length = 0x000800

   RAMGS0      : origin = 0x00C000, length = 0x002000
   RAMGS1      : origin = 0x00E000, length = 0x002000
   RAMGS2      : origin = 0x010000, length = 0x002000
   RAMGS3      : origin = 0x012000, length = 0x001FF8
//   RAMGS3_RSVD : origin = 0x013FF8, length = 0x000008     /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */
}


SECTIONS
{
   codestart        : > BEGIN,     PAGE = 0, ALIGN(4)
   .text            : >>FLASH_BANK0_SEC2 | FLASH_BANK0_SEC3 | FLASH_BANK0_SEC4,   PAGE = 0, ALIGN(4)
   .cinit           : > FLASH_BANK0_SEC2,     PAGE = 0, ALIGN(4)
   .pinit           : > FLASH_BANK0_SEC2,     PAGE = 0, ALIGN(4)
   .switch          : > FLASH_BANK0_SEC2,     PAGE = 0, ALIGN(4)
   .reset           : > RESET,     PAGE = 0, TYPE = DSECT /* not used, */

   .cio             : > RAMLS03,    PAGE = 0
   .stack           : > RAMM1,     PAGE = 1
   .ebss            : > RAMLS6,    PAGE = 1
   .esysmem         : > RAMLS6,    PAGE = 1
   .econst          : > FLASH_BANK0_SEC5,    PAGE = 0, ALIGN(4)

   ramgs0           : > RAMGS0,    PAGE = 1
   ramgs1           : > RAMGS1,    PAGE = 1

   GROUP
   {
       .TI.ramfunc
   } LOAD = FLASH_BANK0_SEC2,
     RUN = RAMLS03,
     LOAD_START(_RamfuncsLoadStart),
     LOAD_SIZE(_RamfuncsLoadSize),
     LOAD_END(_RamfuncsLoadEnd),
     RUN_START(_RamfuncsRunStart),
     RUN_SIZE(_RamfuncsRunSize),
     RUN_END(_RamfuncsRunEnd),
     PAGE = 0, ALIGN(4)
	 

   DataBufferSection : > RAMM1, PAGE = 1, ALIGN(4)
}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
