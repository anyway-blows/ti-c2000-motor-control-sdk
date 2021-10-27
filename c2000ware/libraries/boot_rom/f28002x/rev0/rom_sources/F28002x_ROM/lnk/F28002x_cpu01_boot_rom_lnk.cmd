/*
//###########################################################################
//
// FILE:    F28002x_cpu01_boot_rom_lnk.cmd
//
// TITLE:   CPU1 boot rom linker command file
//
//
//###########################################################################
// $TI Release:  $
// $Release Date:  $
//###########################################################################
*/

//
// Required for EABI retention of unreferenced sections
//
--retain '(.CPU1Version)'

--retain '(FPUfftTables)'
--retain '(FPUmathTables)'

--retain '(.test_signature1)'
--retain '(.bootrom_func)'
--retain '(.CKSUMLOC)'
--retain '(.CKSUMFUNCS)'

--retain '(AES_TABLE_TE0)'
--retain '(AES_TABLE_TE1)'
--retain '(AES_TABLE_TE2)'
--retain '(AES_TABLE_TE3)'
--retain '(AES_TABLE_TE4)'

--retain '(AES_TABLE_TD0)'
--retain '(AES_TABLE_TD1)'
--retain '(AES_TABLE_TD2)'
--retain '(AES_TABLE_TD3)'
--retain '(AES_TABLE_TD4)'

--retain '(.romFlashApiTable)'

MEMORY
{
    //
    // ROM Memories
    //
    ROM_SIGNATURE			: origin = 0x3F0000, length = 0x000002
    
    AES_TABLES_TE0          : origin = 0x3F0002, length = 0x000200
    AES_TABLES_TE1          : origin = 0x3F0202, length = 0x000200
    AES_TABLES_TE2          : origin = 0x3F0402, length = 0x000200
    AES_TABLES_TE3          : origin = 0x3F0602, length = 0x000200
    AES_TABLES_TE4          : origin = 0x3F0802, length = 0x000200
    
    AES_TABLES_TD0          : origin = 0x3F0A02, length = 0x000200
    AES_TABLES_TD1          : origin = 0x3F0C02, length = 0x000200
    AES_TABLES_TD2          : origin = 0x3F0E02, length = 0x000200
    AES_TABLES_TD3          : origin = 0x3F1002, length = 0x000200
    AES_TABLES_TD4          : origin = 0x3F1202, length = 0x000200
        
    IQMATH                  : origin = 0x3F1402, length = 0x00166D
	
	FPU32_FAST_TABLES     	: origin = 0x3F2A70, length = 0x00081a
	FPU32_TWIDDLE_TABLES    : origin = 0x3F328A, length = 0x000DF8
	
    BOOT                    : origin = 0x3F4082, length = 0x003FD0
	INTERRUPT_HANDLERS      : origin = 0x3F8052, length = 0x0001AE
	CPU_FAST_DATA_ROM	    : origin = 0x3F8200, length = 0x000100

	BOOT_PBIST_CHECKSUM	    : origin = 0x3F8300, length = 0x000042
	RESERVED_01      	    : origin = 0x3F8342, length = 0x00003E
    CHECKSUM                : origin = 0x3F8380, length = 0x000042

    FLASHAPI_IN_ROM		    : origin = 0x3F83C2, length = 0x000B00
    FLASHAPI_APITABLE       : origin = 0x3F8EC2, length = 0x000028
    
    CRCTABLE_ROM		    : origin = 0x3F8EEA, length = 0x000008
    VERSION    			    : origin = 0x3F8EF2, length = 0x000002

    RESERVED_02			    : origin = 0x3F8EF4, length = 0x0070CA
    
    VECS       			    : origin = 0x3FFFBE, length = 0x000042
    
    //
    // RAM Memories
    //
    CPU1_BROM_STATUS        : origin = 0x002,    length = 0x000002
    CPU1_BROM_BOOTMODE      : origin = 0x004,    length = 0x000002
    CPU1_PBIST_STATUS       : origin = 0x006,    length = 0x000002
    BSS       			    : origin = 0x008,    length = 0x000020
    STACK      			    : origin = 0x028,    length = 0x0000E0
    FLASHAPI_EBSS		    : origin = 0x108,    length = 0x000020
}

SECTIONS
{
    IQmathTables : load = IQMATH
    {
         -l IQmath_fpu32_eabi.lib
    }
	
    GROUP : load = FPU32_TWIDDLE_TABLES, ALIGN(2)
    {
         FPUfftTables
    }

    GROUP : load = FPU32_FAST_TABLES, ALIGN(2)
    {
        FPUmathTables
    }
	
    GROUP : load = INTERRUPT_HANDLERS
    {
        CPU1BROM_DEFAULT_HANDLER
   		CPU1BROM_PIE_MISMATCH_HANDLER
        CPU1BROM_NMI_HANDLER
        CPU1BROM_ITRAP_HANDLER
    }

    FLASHAPI_IN_ROM : load = FLASHAPI_IN_ROM,               ALIGN(2)
    {
        -lFlashAPI_F28002x_FPU32.lib(.text)
        -lrts2800_fpu32_eabi.lib(.text)
    }

    FLASHAPI_EBSS : load = FLASHAPI_EBSS
    {
    	-lFlashAPI_F28002x_FPU32.lib(.bss)
    }

	.romFlashApiTable:      load = FLASHAPI_APITABLE
	.test_signature1: 	    load = ROM_SIGNATURE		
	.bootrom_func: 	        load = CRCTABLE_ROM //Will come from secure CRC lib
	.InitBoot: 		        load = BOOT			
	.text: 			        load = BOOT     				
	.CKSUMFUNCS: 		    load = BOOT,					ALIGN(2)
	.Isr: 				    load = BOOT
	.BootVecs: 		        load = VECS     				
	.PBIST_CKSUMLOC:        load = BOOT_PBIST_CHECKSUM,  	ALIGN(2)
	.CKSUMLOC: 		        load = CHECKSUM,  				ALIGN(2)
	.CPU1Version:		    load = VERSION
	.stack: 			    load = STACK    				
	.bss:  			        load = BSS      				
	.const:			        load = BOOT,					ALIGN(2)
	.cinit:                 load = BOOT,                    ALIGN(2)
    .switch:                load = BOOT,                    ALIGN(2)
    AES_TABLE_TE0           load = AES_TABLES_TE0,          ALIGN(2)
    AES_TABLE_TE1           load = AES_TABLES_TE1,          ALIGN(2)
    AES_TABLE_TE2           load = AES_TABLES_TE2,          ALIGN(2)
    AES_TABLE_TE3           load = AES_TABLES_TE3,          ALIGN(2)
    AES_TABLE_TE4           load = AES_TABLES_TE4,          ALIGN(2)
    AES_TABLE_TD0           load = AES_TABLES_TD0,          ALIGN(2)
    AES_TABLE_TD1           load = AES_TABLES_TD1,          ALIGN(2)
    AES_TABLE_TD2           load = AES_TABLES_TD2,          ALIGN(2)
    AES_TABLE_TD3           load = AES_TABLES_TD3,          ALIGN(2)
    AES_TABLE_TD4           load = AES_TABLES_TD4,          ALIGN(2)
    BootStatusVariable:     load = CPU1_BROM_STATUS
    UserBootModeVariable:   load = CPU1_BROM_BOOTMODE
    PBISTStatusVariable:    load = CPU1_PBIST_STATUS
}


