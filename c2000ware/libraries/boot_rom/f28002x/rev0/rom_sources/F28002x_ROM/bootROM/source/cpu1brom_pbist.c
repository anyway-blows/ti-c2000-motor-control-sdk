//###########################################################################
//
// FILE:    cpu1brom_pbist.c
//
// TITLE:   pbist code for CPU1-Core
//
//
//
//###########################################################################
// $TI Release: $
// $Release Date: $
//###########################################################################


// --------------------------------------------------------------------------------------------------------
//                                           INCLUDE FILES
// --------------------------------------------------------------------------------------------------------
#include "hw_pbist.h"
#include "cpu1bootrom.h"
#include "cpu1brom_pbist.h"

uint32_t PBIST_PORMemoryTest(void);

// --------------------------------------------------------------------------------------------------------
//                                             MACROS
// --------------------------------------------------------------------------------------------------------
		
                                                                // Enable Peripherals Clocks
                                                                // PCLKCR10-DCAN, and
                                                                // PCLKCR22-PBIST.
																
#define PBIST_ENABLE_ALL_PERIPHERAL_CLOCKS()                  \
        {                                                     \
            HWREG(CPUSYS_BASE + SYSCTL_O_PCLKCR10) = 0x1UL;   \
            HWREG(CPUSYS_BASE + SYSCTL_O_PCLKCR22) = 0x1UL;   \
        }                                                     
                                                                // Disable peripherals Clocks
                                                                // PCLKCR10-DCAN and
                                                                // PCLKCR22-PBIST.
															  	
#define PBIST_DISABLE_PERIPHERAL_CLOCKS()                     \
        {                                                     \
            HWREG(CPUSYS_BASE + SYSCTL_O_PCLKCR10) = 0x0UL;   \
            HWREG(CPUSYS_BASE + SYSCTL_O_PCLKCR22) = 0x0UL;   \
        }
                                                                // Soft Peripherals Reset
                                                                // 12-DUMMY to provide enough cycles so that Softpres
                                                                // can take effect. In the beginning, PBIST_ACTIVE = 1,
                                                                // all peripherals be mapped to CPU1 SYSCTL_O_SOFTPRES10 --> DCAN
#define PBIST_SOFT_RESET_PERIPHERALS()                        \
        {                                                     \
            HWREG(DEVCFG_BASE + SYSCTL_O_SOFTPRES10) = 0x1UL; \
            asm(" RPT #14 || NOP");                           \
            HWREG(DEVCFG_BASE + SYSCTL_O_SOFTPRES10) = 0x0UL; \
            asm(" RPT #14 || NOP");                           \
            HWREG(PBIST_BASE + PBIST_O_PACT)         = 0x0UL; \
            asm(" RPT #20 || NOP");                           \
            HWREGH(PIECTRL_BASE + PIE_O_IFR12)       = 0x0U;  \
        }

                                                                // SRAM Init, LS4,5,6,7, GS0 and M0 and M1
#define PBIST_RAM_INIT()                                               \
        {                                                              \
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXINIT) = INIT_MEM_LS7TOLS4; \
            HWREG(MEMCFG_BASE + MEMCFG_O_GSXINIT) = INIT_MEM_GS0;      \
            HWREG(MEMCFG_BASE + MEMCFG_O_DXINIT)  = INIT_MEM_M0M1;     \
            asm(" MOV  @T,   #1225 ");                                 \
            asm(" RPT  @T || NOP ");                                   \
        }

		
//-------------------------------------------------------------------------------------------------
// uint32_t PBIST_PORMemoryTest(void)
//-------------------------------------------------------------------------------------------------
//
//! \brief      Test all on chip ROMs using Triple Read XOR Read (TRXR) memory test algorithm. 
//              and test all on chip RAM using the March13 memory test algorithm.
//              
//! \param[in]  None 
//
//  \return     uint32_t return type using following macros with an encoded PBIST status
//                       1 PBIST_MEMORY_TEST_PASS               - All tests passed successfully
//                       2 PBIST_ALWAYSFAIL_TIMEOUT_ERROR       - Expected fail test did not complete
//                                                                Timeout occurred waiting for interrupt 
//                       3 PBIST_ALWAYSFAIL_INCORRECT_OPERATION - PBIST Fail status set
//
//                       4 PBIST_DOUBLEINT_TIMEOUT_ERROR        - Error waiting for expected
//                                                                second interrupt 
//                       5 PBIST_MEMORYTEST_TIMEOUT_ERROR       - Timeout occurred waiting for
//                                                                memory test to complete
//                       6 PBIST_MEMORY_TEST_FAIL_ERROR         - Memory test failure occurred
//                       7 PBIST_MEMORY_LS_INITDONE_ERROR       - LS Memory initialization failed
//                       8 PBIST_MEMORY_GX_INITDONE_ERROR       - GS Memory initialization failed
//                       9 PBIST_MEMORY_M0M1_INITDONE_ERROR     - M0 and/or M1 memory init failed
//
//
//  \note   1. Test 1: Setup negative test with the expectation that the test must fail.  Running 
//             march13 on ROM, is destined to fail.
//
//          2. 1500 clock cycles are taken for PBIST to start testing and report the first fail and 
//             PBIST to trigger an interrupt. Count gets decremented every 16 cycles according to 
//             the shown loop structure. A safe value for count would be decimal of 750.  
//
//          3. Test 2:  All available ROMs and RAMs are tested. RINFOL (RAM Info Mask Register Low) 
//             is programmed using the PBIST_RINFOL_ALL_MEM macro and the Algorithm register is 
//             programmed using the PBIST_ALGO_ALL_MEM macro.
//
//          4. PBIST test is destructive on all RAMs, hence register pbist_status to maintain
//             the status through the tests.
//
//          5. Comprehend the following error conditions:
//             a. Both PBIST and RAM Init complete successfully, return PBIST_MEMORY_TEST_PASS
//                If( (PBIST == PASS) && (RAM Init == PASS) then return PBIST PASS
//             b. PBIST completes successfully but RAM Init fails
//                If( (PBIST == PASS) && (RAM Init == FAIL) then return RAM FAIL
//             c. PBIST fails but RAM Init passes
//                If( (PBIST == FAIL) && (RAM Init == PASS) then return PBIST FAIL
//             d. PBIST fails and RAM Init fails, create a composite error
//                pbist error is in the upper 16 bits, ram init error is in the lower 16 bits
//-------------------------------------------------------------------------------------------------
uint32_t PBIST_PORMemoryTest(void)
{
    
    register uint32_t pbist_status;                             // register running status, See Note 4
	register int32_t  timeout_count;

    uint32_t return_status;

	
    pbist_status  = PBIST_MEMORY_TEST_IN_PROGRESS;              // Initialize pbist_status to a known state
    return_status = PBIST_MEMORY_TEST_IN_PROGRESS;              // Initialize return_status to a known state
    
    DINT;                                                       // Disable interrupts.
    DRTM;                                                       // Disable realtime mode

    EALLOW;
    PBIST_ENABLE_ALL_PERIPHERAL_CLOCKS();                       // Enable necessary peripheral clocks.
    
                                                                // Test 1: Configuring PBIST for expected fail test
                                                                // Performing March 13n test on TMU ROM.  See Note 1.
    HWREGH(PBIST_PIE12_IER) = PBIST_CLEAR_INTERRUPTS;
    HWREGH(PBIST_PIE12_IFR) = PBIST_CLEAR_INTERRUPT_FLAGS;      // Clear any possible left over hardware state
    HWREG(PBIST_PACT)       = PACT_REG_SET_ENABLE;              // Activate PBIST, begin setup for PBIST usage
                                                                // Configure to Override Algo and RINFO registers
    HWREG(PBIST_OVERRIDE)   = (OVERRIDE_REG_ALGO_OVER | OVERRIDE_REG_RINFO_MEM_OVER); 
                                                               
    HWREG(PBIST_DLRT)       = DLRT_REG_CONFIG_ACC_CPU_PBIST;    // Configure CPU to use PBIST

    HWREG(PBIST_RINFOL)     = PBIST_RINFOL_FAIL_ROM_MEM_GROUP;  //
    HWREG(PBIST_RINFOU)     = 0;                                // RINFO upper is unused/don't care on Topolino
    HWREG(PBIST_ALGO)       = PBIST_ALGO_ROM_FAIL_ALGO;         // Effectively selects M13 for ROM. Expect fail.
    
    HWREG(PBIST_OVERRIDE)   = OVERRIDE_REG_CLEAR_OVERRIDES;     // Use algorithm register and RINFOL
                                                                // Initiate ROM test.  Expect test to complete and fail
    HWREG(PBIST_DLRT)       = (DLRT_REG_CONFIG_ACC_CPU_PBIST | DLRT_REG_ROM_TEST);
    EDIS;

    timeout_count     = TIMEOUT_COUNT_FOR_ALWAYS_FAIL;

    while (PBIST_TEST_COMPLETE != HWREGH(PBIST_PIE12_IFR))      // Expect intrpt flag set on test completion. See Note 2     
    {
        timeout_count--;                                        // If test completion flag not set, wait for timeout
        
        if (TIMEOUT_OCCURRED >= timeout_count)
        {
                                                                // PBIST ctlr has timed-out. Clean up and return an error
            pbist_status = PBIST_ALWAYSFAIL_TIMEOUT_ERROR;
        }
    }
    
    if (PBIST_MEMORY_TEST_IN_PROGRESS == pbist_status)          // Proceed if previous operation did not result in error
    {
                                                                // Ensure expected fault occurred.
        if ((FSRF0_REG_PORT0_TEST_PASSED == (HWREG(PBIST_FAIL_STATUS_0))) &&
            (FSRF1_REG_PORT1_TEST_PASSED == (HWREG(PBIST_FAIL_STATUS_1))))
        {
                                                                // Expected failure did not occur fail status is not set
            pbist_status = PBIST_ALWAYSFAIL_INCORRECT_OPERATION;
        }
    }
    
    if (PBIST_MEMORY_TEST_IN_PROGRESS == pbist_status)          // Proceed if previous operation did not result in error
    {
        EALLOW;
        HWREGH(PBIST_PIE12_IFR) = PBIST_CLEAR_INTERRUPTS;
        
        HWREG(PBIST_DLRT)       = DLRT_REG_CONFIG_ACC_CPU_PBIST | \
                                  DLRT_REG_ROM_TEST             | \
                                  DLRT_REG_IDDQ_TEST;          
        
        HWREG(PBIST_STR)        = STR_REG_START;                // Start / Time Stamp Mode Restart.
        EDIS;

        timeout_count = TIMEOUT_COUNT_FOR_FLUSHOUT;

        while (PBIST_TEST_COMPLETE != HWREGH(PBIST_PIE12_IFR))  // Expect and process second interrupt 
        {
            timeout_count--;
            if(TIMEOUT_OCCURRED >= timeout_count)
            {               
                pbist_status = PBIST_DOUBLEINT_TIMEOUT_ERROR;   // Timeout occurred waiting for second interrupt
            }
        }
    }
                                                                // Test 2:  Configure to test all memories. See Note 3
    if (PBIST_MEMORY_TEST_IN_PROGRESS == pbist_status)          // Proceed if previous operation did not result in error
    {
        EALLOW;
        HWREGH(PBIST_PIE12_IFR) = PBIST_CLEAR_INTERRUPTS;
        HWREG(PBIST_DLRT)       = DLRT_REG_CLEAR;
        HWREG(PBIST_OVERRIDE)   = (OVERRIDE_REG_ALGO_OVER | OVERRIDE_REG_RINFO_MEM_OVER); 
        
        HWREG(PBIST_DLRT)       = DLRT_REG_CONFIG_ACC_CPU_PBIST;// Configure CPU to use PBIST
        
        HWREG(PBIST_RINFOL)     = PBIST_RINFOL_ALL_MEM;         // Configure RAM Info Reg to test all memories
        HWREG(PBIST_RINFOU)     = 0UL;

        HWREG(PBIST_ALGO)       = PBIST_ALGO_ALL_MEM;           // Configure Algorithm Reg to test all memories
        HWREG(PBIST_OVERRIDE)   = OVERRIDE_REG_RINFO_MEM_OVER;  // Configure to use the Algorithm register
		
        HWREG(PBIST_DLRT)       = DLRT_REG_GONOGO_TEST          | \
                                  DLRT_REG_CONFIG_ACC_CPU_PBIST | \
                                  DLRT_REG_ROM_TEST;                
        EDIS;

        timeout_count = TIMEOUT_COUNT_FOR_MEMORY_TEST;
        
        while (PBIST_TEST_COMPLETE != HWREGH(PBIST_PIE12_IFR))  // Check interrupt flag to see if test completed
        {
            timeout_count--;
            if (TIMEOUT_OCCURRED >= timeout_count)
            {                
                pbist_status = PBIST_MEMORYTEST_TIMEOUT_ERROR;  // Timeout occurred waiting for completion interrupt
            }
        }
    }

	if (PBIST_MEMORY_TEST_IN_PROGRESS == pbist_status)          // Proceed if previous operation did not result in error
    {
																// If there is an error, cleanup and report error
		if((FSRF0_REG_PORT0_TEST_FAILED == HWREG(PBIST_FAIL_STATUS_0)) || 
		   (FSRF0_REG_PORT0_TEST_FAILED == HWREG(PBIST_FAIL_STATUS_1)))
		{			
			pbist_status = PBIST_MEMORY_TEST_FAIL_ERROR;        // PBIST Fail status indicates memory test failed
		}
		else
		{
            pbist_status = PBIST_MEMORY_TEST_PASS;              // All memories were tested successfully
		}	
	}
                                                                // All Done! Cleanup and return status of test
    EALLOW;
    PBIST_SOFT_RESET_PERIPHERALS();
    PBIST_DISABLE_PERIPHERAL_CLOCKS();
    PBIST_RAM_INIT();
    EDIS;
                                                                // Error Processing, See Note
    if (CHECK_LSINIT_DONE != (HWREG(MEMCFG_BASE + MEMCFG_O_LSXINITDONE) & MASK_LS7TOLS4))
    {
        return_status = PBIST_MEMORY_LS_INITDONE_ERROR;
    }
    else if (CHECK_GSINIT_DONE != (HWREG(MEMCFG_BASE + MEMCFG_O_GSXINITDONE) & MASK_GS0))
    {
        return_status = PBIST_MEMORY_GX_INITDONE_ERROR;
    }
    else if (CHECK_DXINIT_DONE != (HWREG(MEMCFG_BASE + MEMCFG_O_DXINITDONE)  & MASK_MEM_M0M1))
    {
        return_status = PBIST_MEMORY_M0M1_INITDONE_ERROR;
    }
    else
    {
        return_status = PBIST_MEMORY_TEST_IN_PROGRESS;
    }

    if (PBIST_MEMORY_TEST_IN_PROGRESS == return_status )        // RAM Init completes successfully
    {                                                           // No memory init error, return pbist_status
        return_status = pbist_status;                           // Covers Notes 5a, and 5c
    }
    else
    {                                                           // One of the memory init did not complete
        if (PBIST_MEMORY_TEST_PASS != pbist_status)             // Is there a pbist error as well?
        {                                                       // There was a previous pbist error as well
            pbist_status  = (pbist_status & PBIST_MASK_KEEP_ERROR_CODE_UPPER);
            return_status = (return_status >> ISOLATE_MEM_INIT_ERROR);
            return_status = (return_status | pbist_status);     // composite error, See Note 5d
        }
        else
        {                                                       // There is no error in pbist_status, See Note 5b
                                                                // only memory init failed
        }                                                       // return status already identifies the error
    }

    return (return_status);
}

//-------------------------------------------------------------------------------------------------------
//                                               END OF FILE
//-------------------------------------------------------------------------------------------------------

