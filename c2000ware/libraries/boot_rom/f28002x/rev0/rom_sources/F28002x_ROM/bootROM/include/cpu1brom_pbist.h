//###########################################################################
//
// FILE:   pbist.h
//
// TITLE:  PBIST Definitions
//
//###########################################################################
// $TI Release: $
// $Release Date: $
//###########################################################################



#ifndef F28002X_TOPOLINO_PBIST_H
#define F28002X_TOPOLINO_PBIST_H


//---------------------------------------------------------------------------------------------------------
//                                          CONSTANT DEFINITIONS
//---------------------------------------------------------------------------------------------------------

                                                                // PBIST register addresses
#define PBIST_DLRT                          0x5E364             // PBIST Data Logger Register
#define PBIST_STR                           0x5E36C             // Resume testing
#define PBIST_PACT                          0x5E380             // PBIST Activate Register
#define PBIST_OVERRIDE                      0x5E388             // PBIST Override Register
#define PBIST_FAIL_STATUS_0                 0x5E390             // Fail Status - Port 0
#define PBIST_FAIL_STATUS_1                 0x5E394             // Fail Status - Port 1
#define PBIST_ALGO                          0x5E3C4             // PBIST Algorithm Register
#define PBIST_RINFOL                        0x5E3C8             // RAM Info Mask Register Lower
#define PBIST_RINFOU                        0x5E3CC             // RAM Info Mask Register Higher

#define PBIST_PIE12_IER                     (0x0CE0U + 0x18U)   // PBIST Interrupt Enable Register
#define PBIST_PIE12_IFR                     (0x0CE0U + 0x19U)   // PBIST Interrupt Flag Register

                                                                // ERROR CODES - Used as return values
#define PBIST_ALWAYSFAIL_TIMEOUT_ERROR       0xFF00FF00U        // Expected fail test did not complete
#define PBIST_ALWAYSFAIL_INCORRECT_OPERATION 0xFF11FF01U
#define PBIST_DOUBLEINT_TIMEOUT_ERROR        0xFF22FF02U
#define PBIST_MEMORYTEST_TIMEOUT_ERROR       0xFF33FF03U

#define PBIST_MEMORY_TEST_FAIL_ERROR         0xFF44FF04U

#define PBIST_MEMORY_LS_INITDONE_ERROR       0xFFAAFF0AU
#define PBIST_MEMORY_GX_INITDONE_ERROR       0xFFBBFF0BU
#define PBIST_MEMORY_M0M1_INITDONE_ERROR     0xFFCCFF0CU
#define ISOLATE_MEM_INIT_ERROR               16

#define PBIST_MASK_KEEP_ERROR_CODE_UPPER     0xFFFF0000U

#define PBIST_MEMORY_TEST_IN_PROGRESS        0xFAAB1234UL       // Return status while memory testing is in progress
#define PBIST_MEMORY_TEST_PASS               0xFAABDEEDUL       // Marker for successful completion of mem test


                                                                // Timeout values for various test configs
#define TIMEOUT_COUNT_FOR_ALWAYS_FAIL       1000UL
#define TIMEOUT_COUNT_FOR_FLUSHOUT          25000UL
#define TIMEOUT_COUNT_FOR_MEMORY_TEST       150000UL

#define INIT_MEM_LS7TOLS4                   0xF0UL              // Local Shared only LS7, LS6, LS5 and LS4 supported
#define INIT_MEM_GS0                        0x1UL               // Global Shared memory, only GS1 supported
#define INIT_MEM_M0M1                       0x3UL               // For Initialize Memories M0 and M1

#define MASK_LS7TOLS4                       INIT_MEM_LS7TOLS4
#define MASK_GS0                            INIT_MEM_GS0
#define MASK_MEM_M0M1                       INIT_MEM_M0M1

#define CHECK_LSINIT_DONE                   INIT_MEM_LS7TOLS4
#define CHECK_GSINIT_DONE                   INIT_MEM_GS0
#define CHECK_DXINIT_DONE                   INIT_MEM_M0M1

#define TIMEOUT_OCCURRED                    0L

                                                                // ALGO register bits description
#define PBIST_ALGO_TRXR_M0PBIST_ROM         ((uint32_t)1 << 0)  // ALGORITHM ID for PBIST ROM test

#define PBIST_ALGO_TRXR_M0SECR0             ((uint32_t)1 << 1)  // ROM:  m0SECR0
#define PBIST_ALGO_TRXR_M0SECR1             ((uint32_t)1 << 2)  // ROM:  m0SECR1

#define PBIST_ALGO_TRXR_M0BOOTR0            ((uint32_t)1 << 3)  // ROM:  m0BOOTR0
#define PBIST_ALGO_TRXR_M0BOOTR1            ((uint32_t)1 << 4)  // ROM:  m0BOOTR1
#define PBIST_ALGO_TRXR_M0BOOTR2            ((uint32_t)1 << 5)  // ROM:  m0BOOTR2
#define PBIST_ALGO_TRXR_M0BOOTR3            ((uint32_t)1 << 6)  // ROM:  m0BOOTR3

#define PBIST_ALGO_TRXR_M0HWBISTR0          ((uint32_t)1 << 7)  // ROM:  m0HWBISTR0
#define PBIST_ALGO_TRXR_M0HWBISTR1          ((uint32_t)1 << 8)  // ROM:  m0HWBISTR1

#define PBIST_ALGO_TRXR_M0FLTESTR           ((uint32_t)1 << 9)  // ROM:  m0FLTESTR

#define PBIST_ALGO_TRXR_M0TMURS1I           ((uint32_t)1 << 10) // ROM:  m0TMURS1i 
#define PBIST_ALGO_TRXR_M0TMURS2I           ((uint32_t)1 << 11) 
#define PBIST_ALGO_TRXR_M0TMURY0I           ((uint32_t)1 << 12) // ROM:  m0TMURY0i 

#define PBIST_ALGO_M13N_SINGLE_PORT_SRAM    ((uint32_t)1 << 13) // RAM:  march13n_post 
#define PBIST_ALGO_M13N_TWO_PORT_SRAM       ((uint32_t)1 << 14) // RAM:  march13n_post


                                                                // RINFOL register bits description
#define PBIST_RINFOL_TRXR_M0PBIST_ROM       ((uint32_t)1 << 0)  // RINFOL ID for PBIST ROM test
              
#define PBIST_RINFOL_TRXR_M0SECR0           ((uint32_t)1 << 1)  // ROM: m0SECR0
#define PBIST_RINFOL_TRXR_M0SECR1           ((uint32_t)1 << 2)  // ROM: m0SECR1
              
#define PBIST_RINFOL_TRXR_M0BOOTR0          ((uint32_t)1 << 3)  // ROM: m0BOOTR0
#define PBIST_RINFOL_TRXR_M0BOOTR1          ((uint32_t)1 << 4)  // ROM: m0BOOTR1
#define PBIST_RINFOL_TRXR_M0BOOTR2          ((uint32_t)1 << 5)  // ROM: m0BOOTR2
#define PBIST_RINFOL_TRXR_M0BOOTR3          ((uint32_t)1 << 6)  // ROM: m0BOOTR3
              
#define PBIST_RINFOL_TRXR_M0HWBISTR0        ((uint32_t)1 << 7)  // ROM: m0HWBISTR0
#define PBIST_RINFOL_TRXR_M0HWBISTR1        ((uint32_t)1 << 8)  // ROM: m0HWBISTR1
              
#define PBIST_RINFOL_TRXR_M0FLTESTR         ((uint32_t)1 << 9)  // ROM: m0FLTESTR
              
#define PBIST_RINFOL_TRXR_M0TMURS1I         ((uint32_t)1 << 10) // ROM: m0TMURS1i 
#define PBIST_RINFOL_TRXR_M0TMURS2I         ((uint32_t)1 << 11) 
#define PBIST_RINFOL_TRXR_M0TMURY0I         ((uint32_t)1 << 12) // ROM: m0TMURY0i 



#define PBIST_RINFOL_M13N_SINGLE_PORT_SRAM  ((uint32_t)1 << 13) // RAM: march13n_post 
#define PBIST_RINFOL_M13N_TWO_PORT_SRAM     ((uint32_t)1 << 14) // RAM: march13n_post

                     
#define PBIST_ALGO_ROM                      (PBIST_ALGO_TRXR_M0PBIST_ROM | \
                                             PBIST_ALGO_TRXR_M0SECR0     | \
                                             PBIST_ALGO_TRXR_M0SECR1     | \
                                             PBIST_ALGO_TRXR_M0HWBISTR0  | \
                                             PBIST_ALGO_TRXR_M0HWBISTR1  | \
                                             PBIST_ALGO_TRXR_M0FLTESTR   | \
                                             PBIST_ALGO_TRXR_M0TMURS1I   | \
                                             PBIST_ALGO_TRXR_M0TMURS2I   | \
                                             PBIST_ALGO_TRXR_M0TMURY0I   )
                    
#define PBIST_ALGO_RAM                      (PBIST_ALGO_M13N_SINGLE_PORT_SRAM | \
                                             PBIST_ALGO_M13N_TWO_PORT_SRAM )
                                             
#define PBIST_ALGO_ALL_MEM                  (PBIST_ALGO_ROM |\
                                             PBIST_ALGO_RAM )
                                             
#define PBIST_RINFOL_ROM                    (PBIST_RINFOL_TRXR_M0PBIST_ROM | \
                                             PBIST_RINFOL_TRXR_M0SECR0     | \
                                             PBIST_RINFOL_TRXR_M0SECR1     | \
                                             PBIST_RINFOL_TRXR_M0BOOTR0    | \
                                             PBIST_RINFOL_TRXR_M0BOOTR1    | \
                                             PBIST_RINFOL_TRXR_M0BOOTR2    | \
                                             PBIST_RINFOL_TRXR_M0BOOTR3    | \
                                             PBIST_RINFOL_TRXR_M0HWBISTR0  | \
                                             PBIST_RINFOL_TRXR_M0HWBISTR1  | \
                                             PBIST_RINFOL_TRXR_M0FLTESTR   | \
                                             PBIST_RINFOL_TRXR_M0TMURS1I   | \
                                             PBIST_RINFOL_TRXR_M0TMURS2I   | \
                                             PBIST_RINFOL_TRXR_M0TMURY0I   )

#define PBIST_RINFOL_RAM                    (PBIST_RINFOL_M13N_SINGLE_PORT_SRAM | \
                                             PBIST_RINFOL_M13N_TWO_PORT_SRAM )

#define PBIST_RINFOL_ALL_MEM                (PBIST_RINFOL_ROM |\
                                             PBIST_RINFOL_RAM )
                                             
                                                                // For always-fail -> Executing MARCH13n on ROM
#define PBIST_RINFOL_FAIL_ROM_MEM_GROUP     PBIST_ALGO_TRXR_M0SECR0
                                                                // ALGO GROUP --> March13n
#define PBIST_ALGO_ROM_FAIL_ALGO            PBIST_ALGO_M13N_SINGLE_PORT_SRAM    


                                                                // Macros for setting bits within a 32 bit register
#define U32_SET_BIT0                        0x00000001UL
#define U32_SET_BIT1                        0x00000002UL
#define U32_SET_BIT2                        0x00000004UL
#define U32_SET_BIT3                        0x00000008UL
#define U32_SET_BIT4                        0x00000010UL
#define U32_SET_BIT5                        0x00000020UL
#define U32_SET_BIT6                        0x00000040UL
#define U32_SET_BIT7                        0x00000080UL
#define U32_SET_BIT8                        0x00000100UL
#define U32_SET_BIT9                        0x00000200UL
#define U32_SET_BIT10                       0x00000400UL
#define U32_SET_BIT11                       0x00000800UL
#define U32_SET_BIT12                       0x00001000UL
#define U32_SET_BIT13                       0x00002000UL
#define U32_SET_BIT14                       0x00004000UL
#define U32_SET_BIT15                       0x00008000UL
#define U32_SET_BIT16                       0x00010000UL
#define U32_SET_BIT17                       0x00020000UL
#define U32_SET_BIT18                       0x00040000UL
#define U32_SET_BIT19                       0x00080000UL
#define U32_SET_BIT20                       0x00100000UL
#define U32_SET_BIT21                       0x00200000UL
#define U32_SET_BIT22                       0x00400000UL
#define U32_SET_BIT23                       0x00800000UL
#define U32_SET_BIT24                       0x01000000UL
#define U32_SET_BIT25                       0x02000000UL
#define U32_SET_BIT26                       0x04000000UL
#define U32_SET_BIT27                       0x08000000UL
#define U32_SET_BIT28                       0x10000000UL
#define U32_SET_BIT29                       0x20000000UL
#define U32_SET_BIT30                       0x40000000UL
#define U32_SET_BIT31                       0x80000000UL

#define U32_SET_ALL                         0xFFFFFFFFUL
#define U32_CLEAR_ALL                       0x00000000UL

#define PACT_REG_SET_ENABLE                 U32_SET_BIT0        // Set PBIST Activate Register ENABLE bit
                                                                // PBIST Data Logger Register bit definitions
#define DLRT_REG_CLEAR                      U32_CLEAR_ALL
#define DLRT_REG_IDDQ_TEST                  U32_SET_BIT1                                            
#define DLRT_REG_ROM_TEST                   U32_SET_BIT2        // Writing 1 to this reg bit starts ROM-based testing

#define DLRT_REG_TCK_GATED                  U32_SET_BIT3        // PBIST interrupt and fails driven to registers
#define DLRT_REG_CONFIG_ACC_CPU_PBIST       U32_SET_BIT4        // CPU is configured to access PBIST  
#define DLRT_REG_GONOGO_TEST                U32_SET_BIT9        // Required for ROM based testing
                                            
#define OVERRIDE_REG_RINFO_MEM_OVER         U32_SET_BIT0        // Configure to use the Algorithm register
#define OVERRIDE_REG_ALGO_OVER              U32_SET_BIT3        // Algorithm to use selected ROM specified in RINFO
#define OVERRIDE_REG_CLEAR_OVERRIDES        U32_CLEAR_ALL       // Clear PBIST Overrides

#define STR_REG_START                       U32_SET_BIT0        // Start / Time Stamp Mode Restart.

#define FSRF0_REG_PORT0_TEST_FAILED         0x00000001UL        // One or more memory test failed.
#define FSRF0_REG_PORT0_TEST_PASSED         0x00000000UL        // No Memory test failure occurred  

#define FSRF1_REG_PORT1_TEST_FAILED         0x00000001UL        // One or more memory test failed.
#define FSRF1_REG_PORT1_TEST_PASSED         0x00000000UL        // No Memory test failure occurred   

#define PBIST_CLEAR_INTERRUPTS              0U                  // For clearing the interrupt enable register
#define PBIST_CLEAR_INTERRUPT_FLAGS         0U                  // Clear any residual state on BOOTROM startup
#define PBIST_TEST_COMPLETE                 U32_SET_BIT3        // PBIST_PIE12_IFR is set, test ran to completion       

#endif                                                          // end of F28002X_TOPOLINO_PBIST_H definition

//---------------------------------------------------------------------------------------------------------
// End of File
//---------------------------------------------------------------------------------------------------------

