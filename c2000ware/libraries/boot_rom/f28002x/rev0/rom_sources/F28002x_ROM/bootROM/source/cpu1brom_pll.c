//###########################################################################
//
// FILE:    cpu1brom_pll.c
//
// TITLE:   PLL Enable and Power up Functions
//
//###########################################################################
// $TI Release: $
// $Release Date: $
//###########################################################################

//
// Included Files
//
#include "cpu1bootrom.h"

static uint32_t pllMultiplier, pllDivider;

//
// Function Prototypes
//
uint16_t BROMDCC_verifySingleShotClock(uint32_t base, DCC_Count0ClockSource clk0src,
                                       DCC_Count1ClockSource clk1src, uint16_t dccCounterSeed0,
                                       uint16_t dccCounterSeed1, uint16_t dccValidSeed0);
								   
//
// CPU1BROM_triggerSysPLLLock - Power up and lock the SYS PLL.
// The "divider" configured in this routine is PLL Output Divider (ODIV)
// and not "SYSCLKDIVSEL".
//
void CPU1BROM_triggerSysPLLLock(uint32_t multiplier, uint32_t divider)
{
	pllMultiplier = multiplier;
	pllDivider = divider;
	
	EALLOW;

    //
    // Bypass PLL from SYSCLK
    //
    HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) &= ~SYSCTL_SYSPLLCTL1_PLLCLKEN;

    //
    // Use INTOSC2 (~10 MHz) as the main PLL clock source
    //
    HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL1) &= ~SYSCTL_CLKSRCCTL1_OSCCLKSRCSEL_M;

	//
	// Turn off PLL and delay for power down
	//
	HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) &= ~SYSCTL_SYSPLLCTL1_PLLEN;

	//
	// Delay 25 cycles
	//
	asm(" MOV    @T,#25 ");
	asm(" RPT    @T \
		  || NOP ");
          
    //
    // Set PLL Multiplier and Output Clock Divider (ODIV)
    //
    HWREG(CLKCFG_BASE + SYSCTL_O_SYSPLLMULT) = 
                     ((HWREG(CLKCFG_BASE + SYSCTL_O_SYSPLLMULT) & 
                     ~(SYSCTL_SYSPLLMULT_ODIV_M | SYSCTL_SYSPLLMULT_IMULT_M)) |
                              (divider << SYSCTL_SYSPLLMULT_ODIV_S) |
                              (multiplier << SYSCTL_SYSPLLMULT_IMULT_S));

	//
	// Enable the sys PLL
	//
	HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) |= SYSCTL_SYSPLLCTL1_PLLEN;

    EDIS;
}

uint16_t CPU1BROM_switchToPLL(void)
{
	uint16_t count = 1024; // timeout
	uint16_t dccCnt0Seed, dccCnt1Seed, dccValid0Seed;
	uint16_t dccStatus;
    uint32_t sysclkdiv;
	
    //
    // Setup DCC Values
    //
    dccCnt0Seed = 90U;
    dccValid0Seed = 20U;
    
    //
    // + below is to convert bit field values to actual divider values 
    //
	dccCnt1Seed = ((uint16_t)((100UL * pllMultiplier)/(pllDivider + 1UL)));
    
	//
	// Wait for the SYSPLL lock counter
	//
	while(((HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLSTS) &
			SYSCTL_SYSPLLSTS_LOCKS) == 0U) && (count != 0U))
	{
		count--;
	}
    
	//
	// Using DCC to verify the PLL clock
	//
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DCC0);
	dccStatus = BROMDCC_verifySingleShotClock(DCC0_BASE,
											  (DCC_Count0ClockSource)DCC_COUNT0SRC_INTOSC2,
											  (DCC_Count1ClockSource)DCC_COUNT1SRC_PLL,
											  dccCnt0Seed, dccCnt1Seed, dccValid0Seed);
	SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_DCC0);
	
	//
    // If DCC failed to verify PLL clock is running correctly, update status 
	// and power down PLL
    //
    if(dccStatus == 0U)
    {
        //
        // Turn off PLL and delay for power down
        //
        EALLOW;
        HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) &= ~SYSCTL_SYSPLLCTL1_PLLEN;
        EDIS;

        //
        // Delay 25 cycles
        //
        asm(" MOV    @T,#25 ");
        asm(" RPT    @T \
              || NOP ");
    }
	else
	{
        //
        // Read clkdiv from OTP. 
        //
        sysclkdiv = (OTP_BOOT_CONFIGURE_WORD & 0x000000FCUL) >> 2;
        
        //
        // If it is invalid value (< /2) set the divider to /2.
        // sysclkdiv == 0 means div by 1, so set it to div by 2 to ensure the 
        // sysclk doesnt go beyond 95Mhz (based on APLL_MULT_38 and APLL_DIV_2 
        // set above).
        //
        if(0UL == sysclkdiv)
        {
            sysclkdiv = 1;
        }

        EALLOW;        
        HWREGH(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) = ((HWREGH(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) &
                                                    ~SYSCTL_SYSCLKDIVSEL_PLLSYSCLKDIV_M) | sysclkdiv);
        
	    //
	    // Switch sysclk to PLL clock
	    //
	    HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) |= SYSCTL_SYSPLLCTL1_PLLCLKEN;
        EDIS;
        
	    //
	    // Delay 25 cycles
	    //
        asm(" MOV    @T,#25 ");
        asm(" RPT    @T \
              || NOP ");
	}
    return (dccStatus);
}

uint16_t BROMDCC_verifySingleShotClock(uint32_t base, DCC_Count0ClockSource clk0src,
                                       DCC_Count1ClockSource clk1src, uint16_t dccCounterSeed0,
                                       uint16_t dccCounterSeed1, uint16_t dccValidSeed0)
{
    uint16_t j = dccCounterSeed1;

    //
    // Clear DONE and ERROR flags
    //
    EALLOW;
    HWREGH(DCC0_BASE + DCC_O_STATUS) = 3U;
    EDIS;

    //
    // Disable DCC
    //
    DCC_disableModule(DCC0_BASE);

    //
    // Disable Error Signal
    //
    DCC_disableErrorSignal(DCC0_BASE);

    //
    // Disable Done Signal
    //
    DCC_disableDoneSignal(DCC0_BASE);

    //
    // Configure Clock Source0 to whatever set as a clock source for PLL
    //
    DCC_setCounter0ClkSource(DCC0_BASE, clk0src);

    //
    // Configure Clock Source1 to PLL
    //
    DCC_setCounter1ClkSource(DCC0_BASE, clk1src);

    //
    // Configure COUNTER-0, COUNTER-1 & Valid Window
    //
    DCC_setCounterSeeds(DCC0_BASE, dccCounterSeed0, dccValidSeed0,
                        dccCounterSeed1);

    //
    // Enable Single Shot mode
    //
    DCC_enableSingleShotMode(DCC0_BASE, DCC_MODE_COUNTER_ZERO);

    //
    // Enable DCC to start counting
    //
    DCC_enableModule(DCC0_BASE);

    //
    // Wait until Error or Done Flag is generated or timeout
    //
    while(((HWREGH(DCC0_BASE + DCC_O_STATUS) &
           (DCC_STATUS_ERR | DCC_STATUS_DONE)) == 0U) && (j != 0U))
    {
       j--;
    }

    //
    // Returns true if DCC completes without error
    //
    return((HWREGH(DCC0_BASE + DCC_O_STATUS) &
            (DCC_STATUS_ERR | DCC_STATUS_DONE)) == DCC_STATUS_DONE);

}

//
// End of File
//
