/*
//###########################################################################
//
// FILE:    F2838x_cpu01_secure_rom_lnk.cmd
//
// TITLE:   CPU1 secure rom linker command file
//
//
//###########################################################################
// $TI Release:  $
// $Release Date:  $
//###########################################################################
*/

--retain '(SCCLZ1)'
--retain '(SCCLZ2)'
--retain '(DcsmBank0Z1RegsFile)'
--retain '(DcsmBank0Z2RegsFile)'
--retain '(DcsmCommonRegsFile)'
--retain '(DcsmCommon2RegsFile)'

MEMORY
{
	   Z1_SCC_ROM_SFW      	: origin = 0x3E8000, length = 0x002000 //, fill = 0xFFFF     /* Zone 1 Secure ROM FW upgrade reserved*/
	   Z1_SCC_ROM	   		: origin = 0x3EA000, length = 0x000800 //, fill = 0xFFFF     /* Zone 1 Secure-Copy/CRC Code Secure ROM (~4KB) */
	   Z2_SCC_ROM      		: origin = 0x3EA800, length = 0x000800 //, fill = 0xFFFF     /* Zone 2 Secure-Copy/CRC Code Secure ROM (~4KB)*/
	   Z1_SECURE_ROM   		: origin = 0x3EB000, length = 0x004FF0 //, fill = 0xFFFF     /* Z1 Secure EXE ROM - InstaSPIN Fast (~28KB) (Free ~12KB)*/
	   RESERVED      		: origin = 0x3EFFF0, length = 0x000010 //, fill = 0xFFFF     /* Reserving last 8 words as DCSM may not generate reset */
                                                                                         /* when accessed from unsecure region */
       
       DCSM_BANK0_Z1        : origin = 0x05F000, length = 0x000030
       DCSM_BANK0_Z2        : origin = 0x05F040, length = 0x000030
       DCSM_COMMON          : origin = 0x05F070, length = 0x000010     /* Common Dual code security module registers */
       DCSM_COMMON2         : origin = 0x05F080, length = 0x000008     /* Common Dual code security module registers */
}

SECTIONS
{
       SCCLZ1:          load = Z1_SCC_ROM
       SCCLZ2:          load = Z2_SCC_ROM

       DcsmBank0Z1RegsFile        : > DCSM_BANK0_Z1
       DcsmBank0Z2RegsFile        : > DCSM_BANK0_Z2
       DcsmCommonRegsFile         : > DCSM_COMMON
       DcsmCommon2RegsFile        : > DCSM_COMMON2
}


