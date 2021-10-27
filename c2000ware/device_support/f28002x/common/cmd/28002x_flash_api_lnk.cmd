-stack 0x380

MEMORY
{
   BOOT_RSVD        : origin = 0x00000002, length = 0x00000126
   RAMM0            : origin = 0x00000128, length = 0x000002D8

   RAMLS4567        : origin = 0x0000A000, length = 0x00002000

   RESET            : origin = 0x003FFFC0, length = 0x00000002

   FLASHBANK1       : origin = 0x00080000, length = 0x00010000
   BOOTROM          : origin = 0x003F0000, length = 0x00008000
   BOOTROM_EXT      : origin = 0x003F8000, length = 0x00007FC0


   RAMM1            : origin = 0x00000400, length = 0x00000380     /* on-chip RAM block M1 */
   BOOT_RSVD_SYSBIOS: origin = 0x00000780, length = 0x00000080
   RAMGS0           : origin = 0x0000C000, length = 0x00000800
}


SECTIONS
{
   .text            : > RAMLS4567
   .TI.ramfunc      : > RAMLS4567
   .cinit           : > RAMM0
   .switch          : > RAMM0
   .reset           : > RESET,                  TYPE = DSECT /* not used, */
   .cio             : > RAMGS0
   codestart        : > RAMGS0

   .stack           : > RAMM1
   .bss             : > RAMLS4567
   .bss:output      : > RAMLS4567
   .init_array      : > RAMM0
   .const           : > RAMLS4567
   .data            : > RAMLS4567
   .sysmem          : > RAMLS4567

    ramgs0 : > RAMGS0, type=NOINIT
    ramgs1 : > RAMGS0, type=NOINIT

    DataBufferSection : > RAMGS0, ALIGN(8)
}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
