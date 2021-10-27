//###########################################################################
//
// FILE:    cpu1brom_system_boot.c
//
// TITLE:   CPU1 System Boot
//
// CPU1 System Initialization and associated functions
//
//###########################################################################
// $TI Release: $
// $Release Date:  $
//###########################################################################

//
// Included Files
//
#include "cpu1bootrom.h"
#include "hw_asysctl.h"
//
// Globals
//
#pragma DATA_SECTION(CPU1BROM_bootMode, "UserBootModeVariable");
uint32_t CPU1BROM_bootMode;

#pragma DATA_SECTION(CPU1BROM_pbistStatus, "PBISTStatusVariable");
uint32_t CPU1BROM_pbistStatus;

uint32_t CPU1BROM_flashSingleBitError_lowAddress;
uint32_t CPU1BROM_flashSingleBitError_highAddress;

#pragma DATA_SECTION(CPU1BROM_bootStatus, "BootStatusVariable");
uint32_t CPU1BROM_bootStatus;

extern uint32_t CPU1BROM_itrapAddress;

volatile uint16_t CPU1BROM_nmiStatus;

void WaitBoot(void);
void CPU1BROM_setupDeviceSystems(void);
void CPU1BROM_performDeviceConfiguration(void);
uint32_t CPU1BROM_startSystemBoot(void);
extern unsigned short verify_pbist_checksum_onROM(void);

void CPU1BROM_setPMMTrims(void);
void CPU1BROM_setIntOscTrims(void);
void CPU1BROM_configureAPLL(void);


//
// CPU1BROM_setPMMTrims - Write PMM trims from OTP to analog registers
//
void CPU1BROM_setPMMTrims(void)
{
	//
    // Check PMM Trim key and return if not set
    //
	if(TI_OTP_PMM_LC_TRIM_KEY != TI_OTP_KEY)
    {
        return;
    }
	
	//
	// Read enMASK from TI OTP and Configure. Banking window has to be enabled 
	// before configuring the PMM Trim.
	//
	EALLOW;
	if((TI_OTP_REG_VREGCTL_ENMASK_KEY) == VREGCTL_ENMASK_KEY)
	{
		// OTP bits programmed zero means POR/BOR generation is masked in HW
		// this means no need to implement Blanking period in SW (but has to be enabled)
		if(TI_OTP_REG_VREGCTL_ENMASK_VAL == 0x0U)
		{
			// Blanking window is implemented in HW if the ENMASK bit (bit 15 of VREGCTL) is set to '1'
			// the default of this bit is '0'.
			HWREGH(ANALOGSUBSYS_BASE + BROM_ANALOG_SYSCTL_O_VREGCTL) = 
					((HWREGH(ANALOGSUBSYS_BASE + BROM_ANALOG_SYSCTL_O_VREGCTL) & 0x7FFFU)| 0x8000U);

			//make sure there is a delay of at least 100ns after this
			// device is running at 10MHz +/- efuse trimmed (or not) so a count of so 1 cycle is about
			// 1/10MHz = .1uS = 100ns; so three NOPS should be good enough margin here
			asm(" NOP ");
			asm(" NOP ");
			asm(" NOP ");
		}
	}

	HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_PMMREFTRIM)   = HWREAD_TI_OTP_PMM_REF_TRIM_VALUE;
	HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_PMMVMONTRIM)  = 
                                    ((HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_PMMVMONTRIM) & 
                                      (~ASYSCTL_PMMVMONTRIM_VDDIOMONBORLTRIM_M)) |
                                     (HWREAD_TI_OTP_PMM_VMON_TRIM_VALUE & 
                                      ASYSCTL_PMMVMONTRIM_VDDIOMONBORLTRIM_M));
    EDIS;

    //
    // Calculate the Count for 75 uS at 10MHz +/- 5% MHz efuse trimmed clock,
    // with buffer which is 1225, This would result in 82uS blank window
    //
    asm(" MOV    @T,#1225 ");
    asm(" RPT    @T \
            ||NOP ");
}

//
// CPU1BROM_setIntOscTrims - Write internal oscillator 1 and 2 trims from OTP
//                           to analog registers
//
void CPU1BROM_setIntOscTrims(void)
{
    //
    // Check OSC Trim key and return if not set
    //
    if(HWREAD_TI_OTP_INTOSC_TRIM_KEY != TI_OTP_KEY)
    {
        return;
    }

    EALLOW;

    //
    // Trim order: OSCREF -> INTOSC1 -> INTOSC2
    //
    HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_OSCREFTRIM) = HWREAD_TI_OTP_OSC_REF_TRIM_VALUE;
    HWREG(ANALOGSUBSYS_BASE + ASYSCTL_O_INTOSC1TRIM) = HWREAD_TI_OTP_INTOSC1_TRIM_VALUE;
    HWREG(ANALOGSUBSYS_BASE + ASYSCTL_O_INTOSC2TRIM) = HWREAD_TI_OTP_INTOSC2_TRIM_VALUE;

    EDIS;

    //
    // calculate the Count for 12 uS at 10MHz +/- 7% MHz efuse trimmed clock, 
    // with buffer this will result in 16uS blank window
    //
    asm(" MOV    @T,#178 ");
    asm(" RPT    @T \
          || NOP ");
}

//
// CPU1BROM_configureAPLL() - Write A-PLL trims from OTP to APLL registers
//                            and set APLL analog configuration registers
//
void CPU1BROM_configureAPLL(void)
{
    EALLOW;

    //
    // Check APLL Trim key and trim APLL if set
    //
    if(HWREAD_TI_OTP_APLL_TRIM_KEY == TI_OTP_KEY)
    {
        HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_APLLREFTRIM) = HWREAD_TI_OTP_APLLREF_TRIM_VALUE;

        HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_SYSAPLLLDOTRIM) = HWREAD_TI_OTP_APLL_SYS_TRIM_VALUE & TI_OTP_APLL_OSC_VDD_MASK;
        HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_SYSAPLLOSCTRIM) = ((HWREAD_TI_OTP_APLL_SYS_TRIM_VALUE &
                                                                TI_OTP_APLL_OSC_FREQ_MASK) >> TI_OTP_APLL_OSC_FREQ_SHIFT);

    }

    //
    // Configure APLL when key is set
    //
    if(HWREAD_TI_OTP_APLLCONFIG_KEY == TI_OTP_KEY)
    {
        HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_SYSAPLLCONFIG1) = HWREAD_TI_OTP_APLLCONFIG1_VALUE;
        HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_SYSAPLLCONFIG2) = HWREAD_TI_OTP_APLLCONFIG2_VALUE;
        HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_SYSAPLLCONFIG3) = HWREAD_TI_OTP_APLLCONFIG3_VALUE;
        HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_SYSAPLLCONFIG4) = HWREAD_TI_OTP_APLLCONFIG4_VALUE;
        HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_SYSAPLLCONFIG5) = HWREAD_TI_OTP_APLLCONFIG5_VALUE;
        HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_SYSAPLLCONFIG7) = HWREAD_TI_OTP_APLLCONFIG7_VALUE;
        HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_SYSAPLLCONFIG8) = HWREAD_TI_OTP_APLLCONFIG8_VALUE;
        HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_SYSAPLLCONFIG9) = HWREAD_TI_OTP_APLLCONFIG9_VALUE;
        HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_SYSAPLLCONFIG10) = HWREAD_TI_OTP_APLLCONFIG10_VALUE;
    }

    EDIS;
}

//
// CPU1BROM_setupDeviceSystems - Adjust dividers, setup flash
//                               configurations, flash power up,
//                               and trim PMM/INTOSC/APLL
//
void CPU1BROM_setupDeviceSystems(void)
{
    uint32_t entryAddress;  
    uint16_t flashPowerUpTimeOut;
    
    //
    // Set the divider to /1 before waking up flash
    //
    EALLOW;
    HWREG(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) = 0x0U;
    EDIS;

    //
    // Set flash PSLEEP
    // (20us pump wakeup = PSLEEP * (1/15MHz) * 2)
	// ==>> PSLEEP = 0x96 (Setting to 0xAA with buffer)
    //
    Flash_setPumpWakeupTime(FLASH0CTRL_BASE, CPU1_FLASH_15MHZ_PSLEEP);

    //
    // Set flash wait state
    //
    Flash_setWaitstates(FLASH0CTRL_BASE, CPU1_FLASH_15MHZ_RWAIT);

    //
    // Set flash timeout variable
    //
    flashPowerUpTimeOut = CPU1_FLASH_15MHZ_TIMEOUT_VALUE;

    //
    // Set flash bank power modes to active
    //
    Flash_setPumpPowerMode(FLASH0CTRL_BASE, FLASH_PUMP_PWR_ACTIVE);
    Flash_setBankPowerMode(FLASH0CTRL_BASE, FLASH_BANK, FLASH_BANK_PWR_ACTIVE);

    //
    // Wait until Flash Bank and Pump are ready
    //
    while((HWREGH(FLASH0CTRL_BASE + FLASH_O_FBPRDY) & (FLASH_FBPRDY_PUMPRDY | FLASH_FBPRDY_BANK0RDY)) !=
          (FLASH_FBPRDY_PUMPRDY | FLASH_FBPRDY_BANK0RDY))
    {
        //
        // If ready status is never set, timeout  and 
        // break the loop and continue with boot
        //
        if(flashPowerUpTimeOut == 0U)
        {
            HWREG(CPUSYS_BASE + SYSCTL_O_SIMRESET) = 
                ((SIMRESET_KEY << SYSCTL_SIMRESET_KEY_S) | SYSCTL_SIMRESET_XRSN);
            while(1){}
        }
        else
        {
            asm(" nop");

            flashPowerUpTimeOut-=1U;
        }
    }

    //
    // Read OTP revision to as check that flash powered up correctly
    //
    HWREAD_OTP_VERSION_FOR_BOOT;

    //
    // Trim PMM
    //
    CPU1BROM_setPMMTrims();

    //
    // Trim INTOSC
    //
    CPU1BROM_setIntOscTrims();

    //
    // CPU1 Patch/Escape Point 6
    //
	entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_6;
    if((entryAddress != 0xFFFFFFFFUL) &&
       (entryAddress != 0x00000000UL))
    {
        //
        // If OTP is programmed, then call OTP patch function
        //
        ((void (*)(void))entryAddress)();
    }

    //
    // Configure A-PLL (Trim, Analog Register Config)
    //
    CPU1BROM_configureAPLL();
    
    //
    // Lock APLL Configuration when key is set
    //
    if(HWREAD_TI_OTP_APLLLOCK_KEY == TI_OTP_KEY)
    {
	    EALLOW;
	    HWREG(ANALOGSUBSYS_BASE + ASYSCTL_O_APLLLOCK) = HWREAD_TI_OTP_APLLLOCK_VALUE;
	    EDIS;
	}
	
	//
	// If enabled via OTP, Validate PLL o/p with DCC and Switch sysclk to PLL
	//
	if( (BOOTPIN_CONFIG_KEY == ((OTP_BOOT_CONFIGURE_WORD & 0xFF000000UL) >> 24U))
	      && (BOOT_CONFIGURE_ENABLE_PLL == (OTP_BOOT_CONFIGURE_WORD & 0x3U)) )
	{
        //
        // Set Multiplier and divider for 190MHz and trigger the lock process.
        // This function will not switch the sysclk to PLL output.
        //
        CPU1BROM_triggerSysPLLLock(APLL_MULT_38, APLL_DIV_2);
        
        //
        // CPU1 Patch/Escape Point 15
        //
        entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_15;
        if((entryAddress != 0xFFFFFFFFUL) &&
           (entryAddress != 0x00000000UL))
        {
            //
            // If OTP is programmed, then call OTP patch function
            //
            ((void (*)(void))entryAddress)();
        }
        
        //
        // Enable DC25 - DCC0/1 as it is used by CPU1BROM_switchToPLL().
        // The DC25 configuration will be later overwritten during Device configuration.
        //
        HWREG(DEVCFG_BASE + SYSCTL_O_DC25)  = BROM_DCX_ALWAYS_ENABLED;
    
        //
        // This function will try to switch the sysclk to PLL output.
        //
	    (void)CPU1BROM_switchToPLL();
	}

    //
    // Set Flash wait states to HW default value
    //
    Flash_setWaitstates(FLASH0CTRL_BASE, CPU1_FLASH_DEFAULT_RWAIT);

    //
    // Adjust the PSLEEP for 100MHz
    //
    Flash_setPumpWakeupTime(FLASH0CTRL_BASE, CPU1_FLASH_DEFAULT_PSLEEP);   
    
    //
    // CPU1 Patch/Escape Point 6
    //
    entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_6;
    if((entryAddress != 0xFFFFFFFFUL) &&
       (entryAddress != 0x00000000UL))
    {
        //
        // If OTP is programmed, then call OTP patch function
        //
        ((void (*)(void))entryAddress)();
    }
}

//
// CPU1BROM_performDeviceConfiguration - Set device configuration registers 
//                                       from OTP
//
void CPU1BROM_performDeviceConfiguration(void)
{
    uint32_t entryAddress;

    EALLOW;

    //
    // Set PARTIDL
    //
	HWREG(DEVCFG_BASE + SYSCTL_O_PARTIDL) = HWREAD_TI_OTP_PARTID_L;
	HWREG(DEVCFG_BASE + SYSCTL_O_PARTIDH) = HWREAD_TI_OTP_PARTID_H;

    //
    // DCx - Always Enabled
    //
	HWREG(DEVCFG_BASE + SYSCTL_O_DC4) 	= BROM_DCX_ALWAYS_ENABLED;
	HWREG(DEVCFG_BASE + SYSCTL_O_DC5) 	= BROM_DCX_ALWAYS_ENABLED;
	HWREG(DEVCFG_BASE + SYSCTL_O_DC8) 	= BROM_DCX_ALWAYS_ENABLED;
	HWREG(DEVCFG_BASE + SYSCTL_O_DC9) 	= BROM_DCX_ALWAYS_ENABLED;
	HWREG(DEVCFG_BASE + SYSCTL_O_DC10) 	= BROM_DCX_ALWAYS_ENABLED;
	HWREG(DEVCFG_BASE + SYSCTL_O_DC11) 	= BROM_DCX_ALWAYS_ENABLED;
	HWREG(DEVCFG_BASE + SYSCTL_O_DC14) 	= BROM_DCX_ALWAYS_ENABLED;
	HWREG(DEVCFG_BASE + SYSCTL_O_DC18)  = BROM_DCX_ALWAYS_ENABLED;
    HWREG(DEVCFG_BASE + SYSCTL_O_DC20)  = BROM_DCX_ALWAYS_ENABLED;
	HWREG(DEVCFG_BASE + SYSCTL_O_DC22)  = BROM_DCX_ALWAYS_ENABLED;
	HWREG(DEVCFG_BASE + SYSCTL_O_DC23)  = BROM_DCX_ALWAYS_ENABLED;
	HWREG(DEVCFG_BASE + SYSCTL_O_DC24)  = BROM_DCX_ALWAYS_ENABLED;
	HWREG(DEVCFG_BASE + SYSCTL_O_DC25)  = BROM_DCX_ALWAYS_ENABLED;
	HWREG(DEVCFG_BASE + SYSCTL_O_DC27)  = BROM_DCX_ALWAYS_ENABLED;
	HWREG(DEVCFG_BASE + SYSCTL_O_DC28)  = BROM_DCX_ALWAYS_ENABLED;
    
	//
    // DCx - Configurable
    //	
	HWREG(DEVCFG_BASE + SYSCTL_O_DC1) 	= BROM_DCX_ENABLE_HIGH | TI_OTP_REG_DC01;
	HWREG(DEVCFG_BASE + SYSCTL_O_DC3) 	= BROM_DCX_ENABLE_HIGH | TI_OTP_REG_DC03;
	HWREG(DEVCFG_BASE + SYSCTL_O_DC15)  = BROM_DCX_ENABLE_HIGH | TI_OTP_REG_DC15;
	HWREG(DEVCFG_BASE + SYSCTL_O_DC21)  = BROM_DCX_ENABLE_HIGH | TI_OTP_REG_DC21;

    //
    // load CPUROM_DCx
	//
	HWREG(DEVCFG_BASE + SYSCTL_O_CPUROM_DC1) = BROM_DCX_ENABLE_HIGH | TI_OTP_CPUROM_DC1;
	HWREG(DEVCFG_BASE + SYSCTL_O_CPUROM_DC2) = BROM_DCX_ENABLE_HIGH | TI_OTP_CPUROM_DC2;
	HWREG(DEVCFG_BASE + SYSCTL_O_CPUROM_DC3) = BROM_DCX_ENABLE_HIGH | TI_OTP_CPUROM_DC3;
	HWREG(DEVCFG_BASE + SYSCTL_O_CPUROM_DC4) = BROM_DCX_ENABLE_HIGH | TI_OTP_CPUROM_DC4;

	//
    // Load PKGTYPE - If KEY is programmed in TI_OTP_PKG_TYPE[15:8] == 0x5A
	//
	if(((TI_OTP_PKG_TYPE & 0xFF00U) >> 8U) == PKG_TYPE_KEY)
	{
		//write only LSB 3:0
		HWREG(DEVCFG_BASE + SYSCTL_O_PKGTYPE) = 
                       (((uint32_t)PKG_TYPE_KEY << SYSCTL_PKGTYPE_BROM_KEY_S) |
		                          (TI_OTP_PKG_TYPE & (uint16_t)0x0000F));
	}
	
    //
    // CPU1 Patch/Escape Point 7
    //
    entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_7;
    if((entryAddress != 0xFFFFFFFFUL) &&
       (entryAddress != 0x00000000UL))
    {
        //
        // If OTP is programmed, then call OTP patch function
        //
        ((void (*)(void))entryAddress)();
    }
	    
    //
    // Lock DCx registers
    //
    HWREGH(DEVCFG_BASE + SYSCTL_O_DEVCFGLOCK2) |= 
	                                   SYSCTL_DEVCFGLOCK2_DC_PERCNF_PARTID;
    EDIS;
}

//
// Enable pullups for the unbonded GPIOs on the 80 pin PROBE package.
//
// Unbonded: 39,42,43,61-63
// Bonded: 0-7,9-19,22-35,37,41,44-46


//
static void enable_unbonded_pullups_80_P_pin(void)
{
	//Write 0 to unbonded pin in order to pull-up.
	//if an available pin is already pulled-up, then the pin stays pulled-up.
	//Logical AND with 0 does both of these.
    EALLOW;
    HWREG(GPIOCTRL_BASE + GPIO_O_GPAPUD) &= 0xFFFFFFFFUL;
    HWREG(GPIOCTRL_BASE + GPIO_O_GPBPUD) &= 0x1FFFF27FUL;
    EDIS;
}

//
// Enable pullups for the unbonded GPIOs on the 80 pin Qual package.
//
// Unbonded: 61-63
// Bonded: 0-19,22-35,37,39-46

//
static void enable_unbonded_pullups_80_Q_pin(void)
{
	//Write 0 to unbonded pin in order to pull-up.
	//if an available pin is already pulled-up, then the pin stays pulled-up.
	//Logical AND with 0 does both of these.
    EALLOW;
    HWREG(GPIOCTRL_BASE + GPIO_O_GPAPUD) &= 0xFFFFFFFFUL;
    HWREG(GPIOCTRL_BASE + GPIO_O_GPBPUD) &= 0x1FFFFFFFUL;
    EDIS;
}

//
// Enable pullups for the unbonded GPIOs on the 64 pin Non-Qual package.
//
// Unbonded: 14,15,25-27,30,31,34,42-46,61-63
// Bonded: 0-13,16-19,22-24,28,29,32,33,35,37,39-41
//
static void enable_unbonded_pullups_64_pin(void)
{
	//Write 0 to unbonded pin in order to pull-up.
	//if an available pin is already pulled-up, then the pin stays pulled-up.
	//Logical AND with 0 does both of these.
    EALLOW;
    HWREG(GPIOCTRL_BASE + GPIO_O_GPAPUD) &= 0x31FF3FFFUL;
    HWREG(GPIOCTRL_BASE + GPIO_O_GPBPUD) &= 0x1FFF83FBUL;
    EDIS;
}


//
// Enable pullups for the unbonded GPIOs on the 64 pin Qual package.
//
// Unbonded: 12-15,25-27,30,31,34,42-46,61-63
// Bonded: 0-11,16-19,22-24,28,29,32,33,35,37,39-41
//
static void enable_unbonded_pullups_64_Q_pin(void)
{
	//Write 0 to unbonded pin in order to pull-up.
	//if an available pin is already pulled-up, then the pin stays pulled-up.
	//Logical AND with 0 does both of these.
    EALLOW;
    HWREG(GPIOCTRL_BASE + GPIO_O_GPAPUD) &= 0x31FF0FFFUL;
    HWREG(GPIOCTRL_BASE + GPIO_O_GPBPUD) &= 0x1FFF83FBUL;
    EDIS;
}

//
// Enable pullups for the unbonded GPIOs on the 48 pin Non-Qual package.
//
// Unbonded: 8-15,17,22,23,25-27,30,31,34,39-46,61-63
// Bonded: 0-7,16,18,19,24,28,29,32,33,35,37
//
static void enable_unbonded_pullups_48_QFN_pin(void)
{
	//Write 0 to unbonded pin in order to pull-up.
	//if an available pin is already pulled-up, then the pin stays pulled-up.
	//Logical AND with 0 does both of these.
    EALLOW;
    HWREG(GPIOCTRL_BASE + GPIO_O_GPAPUD) &= 0x31BD33FFUL;
    HWREG(GPIOCTRL_BASE + GPIO_O_GPBPUD) &= 0x1FFF807BUL;
    EDIS;
}

//
// Enable pullups for the unbonded GPIOs on the 48 pin Qual package.
//
// Unbonded: 10,11,14,15,17,22,25-27,30,31,34,39-46,61-63
// Bonded: 0-9,12,13,16,18,19,23,24,28,29,32,33,35,37
//
static void enable_unbonded_pullups_48_QFP_pin(void)
{
	//Write 0 to unbonded pin in order to pull-up.
	//if an available pin is already pulled-up, then the pin stays pulled-up.
	//Logical AND with 0 does both of these.
    EALLOW;
    HWREG(GPIOCTRL_BASE + GPIO_O_GPAPUD) &= 0x313D00FFUL;
    HWREG(GPIOCTRL_BASE + GPIO_O_GPBPUD) &= 0x1FFF807BUL;
    EDIS;
}

//
// Enable pullups for the unbonded GPIOs on the 40 pin Non-Qual package.
//
// Unbonded: 0,9,10,13,14,15,17,25,26,27,34,46,61,62,63
// Bonded: 1-8,11,12,16,24,28,29,30,31,32,33,35,37
//
static void enable_unbonded_pullups_40_pin(void)
{
	//Write 0 to unbonded pin in order to pull-up.
	//if an available pin is already pulled-up, then the pin stays pulled-up.
	//Logical AND with 0 does both of these.
    EALLOW;
    HWREG(GPIOCTRL_BASE + GPIO_O_GPAPUD) &= 0xF13D19FEUL;
    HWREG(GPIOCTRL_BASE + GPIO_O_GPBPUD) &= 0x1FFF807BUL;
    EDIS;
}

static void CPU1BROM_enableUnbondedGpioPullups(void)
{
	uint32_t pin_count;
	
    //
	// 1 : 40_QFN_NQ Topolino
	// 2 : 48_QFP_Q Topolino
	// 3 : 48_QFN_NQ Topolino
	// 6 : 64_2_QFP_Q_Potenza_Topolino
	// 7 : 64_1_QFP_NQ_Potenza_Topolino
	// 8 : 80_2_QFP_Q_Potenza_Topolino
	// 9 : 80_1_QFP_P_Potenza_Topolino
    //
	if(((TI_OTP_PKG_TYPE & 0xFF00U) >> 8U) == PKG_TYPE_KEY)
	{
		pin_count = (TI_OTP_PKG_TYPE & 0x0000FU);

		switch(pin_count)
		{
			case 1U:
			        enable_unbonded_pullups_40_pin();
					break;
			case 2U:
			        enable_unbonded_pullups_48_QFP_pin();
					break;
			case 3U:
			        enable_unbonded_pullups_48_QFN_pin();
					break;
			case 6U:
			        enable_unbonded_pullups_64_Q_pin();
					break;
			case 7U:
			        enable_unbonded_pullups_64_pin();
					break;
			case 8U:
			        enable_unbonded_pullups_80_Q_pin();
					break;
			case 9U:
			        enable_unbonded_pullups_80_P_pin();
					break;
            default:
			        //  
                    // Empty default to comply with MISRA
					//
                    break;
		}
	}
}

//
// CPU1BROM_startSystemBoot - CPU1 System Initialization procedure and boot
//                            selection
//
uint32_t CPU1BROM_startSystemBoot(void)
{
    uint32_t entryAddress;
	uint32_t ZxOtpGpreg2Errsts = 0UL;
	uint32_t ZxOtpGpreg2CjtagNodeId = 0UL;
	uint32_t ZxOtpGpreg2PbistConfig = 0UL;
		
    //
    // On POR or XRS reset, init boot status variable
    // (Status is non-volatile on any other reset and up to user to clear)
    //
    if((HWREG(CPUSYS_BASE + SYSCTL_O_RESC) & 
        (SYSCTL_RESC_POR | SYSCTL_RESC_XRSN)) != 0U)
    {
        CPU1BROM_bootStatus = 0U;
    }

    //
    // On POR - Init PBIST status
    // (Retain status on any other reset)
    //
    if((HWREG(CPUSYS_BASE + SYSCTL_O_RESC) &
        SYSCTL_RESC_POR) == SYSCTL_RESC_POR)
    {
        CPU1BROM_pbistStatus = 0U;
    }

    //
    // Init boot mode selection
    //
    CPU1BROM_bootMode = 0xFFFFFFFFUL;

    //
    // Update boot status - Boot has started
    //
    CPU1BROM_bootStatus |= CPU1_BOOTROM_BOOTSTS_SYSTEM_START_BOOT;

    //
    // Initialize iTrap address
    //
    CPU1BROM_itrapAddress = 0xFFFFFFFFUL;

    //
    // Initialize NMI Status
    //
    CPU1BROM_nmiStatus = 0U;

    //
    // CPU1 Patch/Escape Point 2
    //
    entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_2;
    if((entryAddress != 0xFFFFFFFFUL) &&
       (entryAddress != 0x00000000UL))
    {
        //
        // If OTP is programmed, then call OTP patch function
        //
        ((void (*)(void))entryAddress)();
    }

    //
    // Enable writing to the EALLOW protected registers
    //
    EALLOW;
       
    //
    // Enable NMI
    //
    HWREGH(NMI_BASE + NMI_O_CFG) = NMI_CFG_NMIE;

    //
    // Disable writing to the EALLOW protected registers
    //
    EDIS;

    //
    // Initialize DCSM
    //
    CPU1BROM_initDCSM();

    //
    // Capture any single bit errors after DCSM init
    // If the below addresses are valid then user has to
    // debug these further
    //
    CPU1BROM_flashSingleBitError_lowAddress =
            Flash_getSingleBitErrorAddressLow(FLASH0ECC_BASE);
    CPU1BROM_flashSingleBitError_highAddress =
            Flash_getSingleBitErrorAddressHigh(FLASH0ECC_BASE);
            
    //
    // Update boot status - DCSM init is complete
    //
    CPU1BROM_bootStatus |= CPU1_BOOTROM_DCSM_INIT_COMPLETE;

    //
    // CPU1 Patch/Escape Point 2
    //
    entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_2;
    if((entryAddress != 0xFFFFFFFFUL) &&
       (entryAddress != 0x00000000UL))
    {
        //
        // If OTP is programmed, then call OTP patch function
        //
        ((void (*)(void))entryAddress)();
    }

    //
    // Check Z2 or Z1 GPREG2 key to determine whether to configure
    // Error Status pin and cjtag node it and run PBIST memory test
    // (Z2 takes priority when programmed)
    //	
	if(HWREAD_Z2_OTP_BOOT_GPREG2_KEY == GPREG2_KEY)
	{
		ZxOtpGpreg2Errsts      = HWREAD_Z2_OTP_BOOT_GPREG2_ERRSTS_CONFIG;
		ZxOtpGpreg2CjtagNodeId = HWREAD_Z2_OTP_BOOT_GPREG2_CJTAGNODEID;
		ZxOtpGpreg2PbistConfig = HWREAD_Z2_OTP_BOOT_GPREG2_PBIST_CONFIG;
	}
    else if(HWREAD_Z1_OTP_BOOT_GPREG2_KEY == GPREG2_KEY)
    {
		ZxOtpGpreg2Errsts      = HWREAD_Z1_OTP_BOOT_GPREG2_ERRSTS_CONFIG;
		ZxOtpGpreg2CjtagNodeId = HWREAD_Z1_OTP_BOOT_GPREG2_CJTAGNODEID;
		ZxOtpGpreg2PbistConfig = HWREAD_Z1_OTP_BOOT_GPREG2_PBIST_CONFIG;		
	}
    else
    {
	    //
        // To avoid misra violation
		//
    }


	//
	// Zx OTP - Check for Key
	//
	if( (HWREAD_Z2_OTP_BOOT_GPREG2_KEY == GPREG2_KEY) || 
	                         (HWREAD_Z1_OTP_BOOT_GPREG2_KEY == GPREG2_KEY))
	{
		//
		// CPU1 Patch/Escape Point 3
		//
		entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_3;
		if((entryAddress != 0xFFFFFFFFUL) &&
		   (entryAddress != 0x00000000UL))
		{
			//
			// If OTP is programmed, then call OTP patch function
			//
			((void (*)(void))entryAddress)();
		}

		//
		// Set ERROR_STS pin if enabled by user
		//
		if(ZxOtpGpreg2Errsts == ERRORSTS_PIN_24)
		{
			GPIO_setPinConfig(GPIO_24_ERRORSTS);
			GPIO_lockPortConfig(GPIO_PORT_A, 0x01000000U); //lock pin 24
		}
		else if(ZxOtpGpreg2Errsts == ERRORSTS_PIN_28)
		{
			GPIO_setPinConfig(GPIO_28_ERRORSTS);
			GPIO_lockPortConfig(GPIO_PORT_A, 0x10000000U); //lock pin 28
		}
		else if (ZxOtpGpreg2Errsts == ERRORSTS_PIN_29)
		{
			GPIO_setPinConfig(GPIO_29_ERRORSTS);	//
			GPIO_lockPortConfig(GPIO_PORT_A, 0x20000000U); //lock pin 29
		}
        else
        {
		    //
            // To avoid misra violation
			//
        }

		if(0UL != (HWREG(CPUSYS_BASE + SYSCTL_O_RESC) & 
                  ((uint32_t)SYSCTL_RESC_POR | (uint32_t)SYSCTL_RESC_XRSN)))
		{
			//set CJTAGNODEID[3:0]
            EALLOW;
			HWREG(DEVCFG_BASE + SYSCTL_O_CJTAGNODEID) = 
                              ((HWREG(DEVCFG_BASE + SYSCTL_O_CJTAGNODEID) & 
                               ((uint32_t)0xF0UL)) | (ZxOtpGpreg2CjtagNodeId));
            EDIS;
		}

		if((HWREG(CPUSYS_BASE + SYSCTL_O_RESC) & ((uint32_t)SYSCTL_RESC_POR)) == SYSCTL_RESC_POR)
		{
            uint16_t pllStatus = BROM_PLL_CONFIG_ERROR;
			uint16_t pbistConfig = ((uint16_t)ZxOtpGpreg2PbistConfig) & 0x3U;
			
		    entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_3;
		    if((entryAddress != 0xFFFFFFFFUL) &&
		            (entryAddress != 0x00000000UL))
		    {
		        /*if OTP is programmed, then call OTP function*/
		        ((void (*)(void))entryAddress)();
		    }

            //
            // Call PBIST only if user has asked for it to RUN.
            //
			if(pbistConfig != GPREG2_PBIST_DISABLED)
			{
				//
                // Check GS RAMs to verify they have completed initialization, else
                // wait ~1500 cycles to guarantee RAM init is complete
                //
                if(HWREG(MEMCFG_BASE + MEMCFG_O_GSXINITDONE) == RAM_GSX_NOT_DONE)
                {
                    asm(" MOV    @T,#0x5DC ");
                    asm(" RPT    @T \
                          || NOP ");
                }
				
                if( (pbistConfig == GPREG2_PBIST_RUN_SYSCLK_95MHZ) ||
				    (pbistConfig == GPREG2_PBIST_RUN_SYSCLK_47_5MHZ))
				{
                    //
                    // If OTP is configured as to NOT use PLL in main sequence, 
                    // then PLL Lock would not have been attempted before. 
                    // In such case try to lock the PLL here.
                    //
                    if( (BOOTPIN_CONFIG_KEY != ((OTP_BOOT_CONFIGURE_WORD & 0xFF000000UL) >> 24U))
                          || (BOOT_CONFIGURE_ENABLE_PLL != (OTP_BOOT_CONFIGURE_WORD & 0x3U)) )
					{
						// Setting up for 190MHz
						CPU1BROM_triggerSysPLLLock(APLL_MULT_38, APLL_DIV_2);
                        
                        //
                        // CPU1 Patch/Escape Point 15
                        //
                        entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_15;
                        if((entryAddress != 0xFFFFFFFFUL) &&
                           (entryAddress != 0x00000000UL))
                        {
                            //
                            // If OTP is programmed, then call OTP patch function
                            //
                            ((void (*)(void))entryAddress)();
                        }
						
					    // Switch SYSCLK to PLL (at max of 95Mhz)
					    (void)CPU1BROM_switchToPLL();
					}
					
                    //
                    // If PLL O/P drives SYSCLK, update the divider to get required frequency.
                    //
					if(SYSCTL_SYSPLLCTL1_PLLCLKEN == 
                                   (HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) &
                                                   SYSCTL_SYSPLLCTL1_PLLCLKEN))
					{
                        // Switch sysclk to bypass clock, before changing the divider 
                        // (as there can be cases of switching from lower to higher frequency).
                        EALLOW;
	                    HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) &= (~SYSCTL_SYSPLLCTL1_PLLCLKEN);
                        EDIS;
                    
					    if(pbistConfig == GPREG2_PBIST_RUN_SYSCLK_95MHZ)
                        {
                            //
                            // Set divider for 95MHz (divide by 2)
                            //
                            EALLOW;
                            HWREGH(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) = 
                                ((HWREGH(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) &
                                 ~SYSCTL_SYSCLKDIVSEL_PLLSYSCLKDIV_M) | SYSCLK_DIV_2);
                            EDIS;
                        }
					    else if(pbistConfig == GPREG2_PBIST_RUN_SYSCLK_47_5MHZ)
					    {
                            //
                            // Set divider for 47.5 MHz (divide by 4)
                            //
                            EALLOW;
                            HWREGH(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) = 
                                ((HWREGH(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) &
                                 ~SYSCTL_SYSCLKDIVSEL_PLLSYSCLKDIV_M) | SYSCLK_DIV_4);
                            EDIS;
					    }
                        else
                        {
                            //
                            // To avoid misra violation
			                //   
                        }
                        
                        // Switch sysclk to PLL clock, after changing the divider.
                        EALLOW;
	                    HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) |= SYSCTL_SYSPLLCTL1_PLLCLKEN;
                        EDIS;
					    pllStatus = BROM_PLL_CONFIG_SUCCESS;
					}
				}
	            else if (pbistConfig == GPREG2_PBIST_RUN_PLL_BYPASS) // PLL Bypass
				{
				    // Switch sysclk to bypass clock
                    EALLOW;
	                HWREGH(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) &= (~SYSCTL_SYSPLLCTL1_PLLCLKEN);
                    EDIS;
                    
					pllStatus = BROM_PLL_CONFIG_SUCCESS;
				}
                else
                {
				    //
                    // To avoid misra violation
					//
                }

                //
                // CPU1 Patch/Escape Point 15
                //
                entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_15;
                if((entryAddress != 0xFFFFFFFFUL) &&
                   (entryAddress != 0x00000000UL))
                {
                    //
                    // If OTP is programmed, then call OTP patch function
                    //
                    ((void (*)(void))entryAddress)();
                }
				
				//
                // Skip PBIST if configured to use PLL and PLL enable fails
                // Call Boot ROM checksum routine to check CRC on bootROM (unsecure ROM only)
                // If checksum fails - skip PBIST and continue to boot.
                //
                if((pllStatus == BROM_PLL_CONFIG_SUCCESS) &&
                   (verify_pbist_checksum_onROM() == PBIST_CHECKSUM_SUCCESS))
                {
                    //
                    // PBIST Memory Test
                    //
                    CPU1BROM_pbistStatus = PBIST_PORMemoryTest();

                    //
                    // PBIST function return here using RPC and reinitialize RPC
                    //
                    asm(" .ref ExitPBISTLoc");
                    asm(" PUSH XAR7");
                    asm(" MOVL XAR7, #ExitPBISTLoc");
                    asm(" PUSH XAR7");
                    asm(" POP RPC");
                    asm(" POP XAR7");

                    //
                    // Reinitialize Variables lost during PBIST RAMINIT
                    //
                    CPU1BROM_bootMode = 0xFFFFFFFFUL;
                    CPU1BROM_itrapAddress = 0xFFFFFFFFUL;

                    //
                    // Set boot status
                    //
                    CPU1BROM_bootStatus = (uint32_t)(CPU1_BOOTROM_BOOTSTS_SYSTEM_START_BOOT |
                                           CPU1_BOOTROM_DCSM_INIT_COMPLETE |
                                           CPU1_BOOTROM_RAM_INIT_COMPLETE |
                                           CPU1_BOOTROM_POR_MEM_TEST_COMPLETE);

                    //
                    // Re-init the single bit error address status.
                    //
                    CPU1BROM_flashSingleBitError_lowAddress =
                            Flash_getSingleBitErrorAddressLow(FLASH0ECC_BASE);
                    CPU1BROM_flashSingleBitError_highAddress =
                            Flash_getSingleBitErrorAddressHigh(FLASH0ECC_BASE);
                }
			}
		}
	}
    
    //
    // If PLL was configured to be enabled, check DCC status to 
    // know if PLL was locked successfully.
    //
    if( (BOOTPIN_CONFIG_KEY == ((OTP_BOOT_CONFIGURE_WORD & 0xFF000000UL) >> 24U))
	      && (BOOT_CONFIGURE_ENABLE_PLL == (OTP_BOOT_CONFIGURE_WORD & 0x3U)) )
    {
        if((HWREGH(DCC0_BASE + DCC_O_STATUS) &
            (DCC_STATUS_ERR | DCC_STATUS_DONE)) == DCC_STATUS_DONE)
        {           
            CPU1BROM_bootStatus |= BOOTROM_PLL_ENABLE_SUCCESS;
        }
    }

    //
    // CPU1 Patch/Escape Point 3
    //
    entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_3;
    if((entryAddress != 0xFFFFFFFFUL) &&
       (entryAddress != 0x00000000UL))
    {
        //
        // If OTP is programmed, then call OTP patch function
        //
        ((void (*)(void))entryAddress)();
    }

	//
    // Enable unbonded GPIO pullups
    //
    CPU1BROM_enableUnbondedGpioPullups();
	
	if(0UL != (HWREG(CPUSYS_BASE + SYSCTL_O_RESC) & 
              ((uint32_t)SYSCTL_RESC_POR | (uint32_t)SYSCTL_RESC_XRSN )))
	{
        EALLOW;
        
		// bypass PLL here
        if((HWREG(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) & SYSCTL_SYSPLLCTL1_PLLCLKEN) != 0UL)
        {
        	HWREG(CLKCFG_BASE + SYSCTL_O_SYSPLLCTL1) &= ~SYSCTL_SYSPLLCTL1_PLLCLKEN;
            //
            // Delay 25 cycles
            //
            asm(" MOV    @T,#25 ");
            asm(" RPT    @T \
                  || NOP ");
        }
        
        //
        // Set PLL multiplier to 0x0
        //
      	HWREG(CLKCFG_BASE + SYSCTL_O_SYSPLLMULT) = 0;

        //
        // Set the divider to /1
        //
      	HWREG(CLKCFG_BASE + SYSCTL_O_SYSCLKDIVSEL) = 0;

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

        EDIS;
    }
	
    if(HWREAD_TI_OTP_DEVCAL_KEY == TI_OTP_KEY)
    {
        //
        // Calibrate ADC(INL, Offset), GPDAC(Offset)  
        //
        (*CBROM_DEVCAL)();
    }

    //
    // On POR 
    //  - Handle POR/XRSn RESC bits so that bootROM would not go into
    //    a unwanted reset handling if it got reset for some reason    
    //  - Check if RAM init is complete, and update boot status.
    //
    if((HWREG(CPUSYS_BASE + SYSCTL_O_RESC) & ((uint32_t)SYSCTL_RESC_POR)) == 
       SYSCTL_RESC_POR)
    {        
        //
        // Clear POR and XRSn
        //
        SysCtl_clearResetCause(SYSCTL_RESC_POR | SYSCTL_RESC_XRSN);
        CPU1BROM_bootStatus |= (uint32_t)((uint32_t)CPU1_BOOTROM_HANDLED_XRSN | 
                                          (uint32_t)CPU1_BOOTROM_HANDLED_POR);
        
        //
        // Check GS RAMs to verify they have completed initialization
        //
        if(HWREG(MEMCFG_BASE + MEMCFG_O_GSXINITDONE) != RAM_GSX_NOT_DONE)
        {
            //
            // Update boot status - RAM Init complete
            //
            CPU1BROM_bootStatus |= CPU1_BOOTROM_RAM_INIT_COMPLETE; 
        }       
    }

    //
    // On XRS - Clear RESC bits so that bootROM would not go into
    // a unwanted reset handling if it got reset for some reason
    //
    if((HWREG(CPUSYS_BASE + SYSCTL_O_RESC) & ((uint32_t)SYSCTL_RESC_XRSN)) == 
       SYSCTL_RESC_XRSN)
    {
        SysCtl_clearResetCause(SYSCTL_RESC_XRSN);
        CPU1BROM_bootStatus |= (uint32_t)(CPU1_BOOTROM_HANDLED_XRSN);
    }

    //
    // Update boot status - Reset cause clearing complete
    //
    CPU1BROM_bootStatus |= CPU1_BOOTROM_RESC_HANDLED;

    //
    // CPU1 Patch/Escape Point 4
    //
    entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_4;
    if((entryAddress != 0xFFFFFFFFUL) &&
       (entryAddress != 0x00000000UL))
    {
        //
        // If OTP is programmed, then call OTP patch function
        //
        ((void (*)(void))entryAddress)();
    }

    //
    // Get boot mode selected
    //
    CPU1BROM_bootMode = CPU1BROM_selectBootMode();

    //
    // CPU1 Patch/Escape Point 4
    //
    entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_4;
    if((entryAddress != 0xFFFFFFFFUL) &&
       (entryAddress != 0x00000000UL))
    {
        //
        // If OTP is programmed, then call OTP patch function
        //
        ((void (*)(void))entryAddress)();
    }

    //
    // Update boot status - Clear booting status field before setting boot mode status
    //
    CPU1BROM_bootStatus &= ~CPU1_BOOTROM_BOOTSTS_BOOT_MASK;

    //
    // Run selected boot mode
    //
    switch(CPU1BROM_bootMode)
    {
        case FLASH_BOOT:
            entryAddress = FLASH_ENTRY_POINT;
            CPU1BROM_bootStatus |= CPU1_BOOTROM_BOOTSTS_IN_FLASH_BOOT;
            break;

        case FLASH_BOOT_ALT1:
            entryAddress = FLASH_ENTRY_POINT_ALT1;
            CPU1BROM_bootStatus |= CPU1_BOOTROM_BOOTSTS_IN_FLASH_BOOT;
            break;

        case FLASH_BOOT_ALT2:
            entryAddress = FLASH_ENTRY_POINT_ALT2;
            CPU1BROM_bootStatus |= CPU1_BOOTROM_BOOTSTS_IN_FLASH_BOOT;
            break;

        case FLASH_BOOT_ALT3:
            entryAddress = FLASH_ENTRY_POINT_ALT3;
            CPU1BROM_bootStatus |= CPU1_BOOTROM_BOOTSTS_IN_FLASH_BOOT;
            break;

        case RAM_BOOT:
            entryAddress = RAM_ENTRY_POINT;
            CPU1BROM_bootStatus |= CPU1_BOOTROM_BOOTSTS_IN_RAM_BOOT;
            break;

        case WAIT_BOOT:
        case WAIT_BOOT_ALT1:
            CPU1BROM_bootStatus |= CPU1_BOOTROM_BOOTSTS_IN_WAIT_BOOT;
            break;

        case PARALLEL_BOOT:
        case PARALLEL_BOOT_ALT1:
            CPU1BROM_bootStatus |= CPU1_BOOTROM_BOOTSTS_IN_PARALLEL_BOOT;
            entryAddress = Parallel_Boot(CPU1BROM_bootMode);
            break;

        case SCI_BOOT:
        case SCI_BOOT_ALT1:
        case SCI_BOOT_ALT2:
        case SCI_BOOT_ALT3:
        case SCI_BOOT_ALT4:
            CPU1BROM_bootStatus |= CPU1_BOOTROM_BOOTSTS_IN_SCI_BOOT;
            entryAddress = SCI_Boot(CPU1BROM_bootMode);
            break;

        case CAN_BOOT:
        case CAN_BOOT_ALT1:
        case CAN_BOOT_ALT2:
        case CAN_BOOT_SENDTEST:
        case CAN_BOOT_ALT1_SENDTEST:
        case CAN_BOOT_ALT2_SENDTEST:
            CPU1BROM_bootStatus |= CPU1_BOOTROM_BOOTSTS_IN_CAN_BOOT;
            entryAddress = DCAN_Boot(CPU1BROM_bootMode,
                                     CAN_BOOT_DEFAULT_BIT_TIMING,
                                     CAN_BOOT_USE_XTAL);
            break;

        case SPI_MASTER_BOOT:
        case SPI_MASTER_BOOT_ALT1:
        case SPI_MASTER_BOOT_ALT2:
        case SPI_MASTER_BOOT_ALT3:
            CPU1BROM_bootStatus |= CPU1_BOOTROM_BOOTSTS_IN_SPI_BOOT;
            entryAddress = SPI_Boot(CPU1BROM_bootMode);
            break;

        case I2C_MASTER_BOOT:
        case I2C_MASTER_BOOT_ALT1:
        case I2C_MASTER_BOOT_ALT2:
            CPU1BROM_bootStatus |= CPU1_BOOTROM_BOOTSTS_IN_I2C_BOOT;
            entryAddress = I2C_Boot(CPU1BROM_bootMode);
            break;


        default:
            //
            // Check if debugger is connected
            //
            if(((uint32_t)HWREG(CPUSYS_BASE + SYSCTL_O_RESC) &
                (uint32_t)SYSCTL_RESC_DCON) == SYSCTL_RESC_DCON)
            {
                //
                // CPU1 Patch/Escape Point 5
                //
                entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_5;
                if((entryAddress != 0xFFFFFFFFUL) &&
                   (entryAddress != 0x00000000UL))
                {
                    //
                    // If OTP is programmed, then call OTP patch function
                    //
                    ((void (*)(void))entryAddress)();
                }
                CPU1BROM_bootMode = WAIT_BOOT;
                CPU1BROM_bootStatus |= CPU1_BOOTROM_BOOTSTS_IN_WAIT_BOOT;
            }
            else
            {
                //
                // CPU5 Patch/Escape Point 5
                //
                entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_5;
                if((entryAddress != 0xFFFFFFFFUL) &&
                   (entryAddress != 0x00000000UL))
                {
                    //
                    // If OTP is programmed, then call OTP patch function
                    //
                    ((void (*)(void))entryAddress)();
                }

                CPU1BROM_bootMode = FLASH_BOOT;
                entryAddress = FLASH_ENTRY_POINT;
                CPU1BROM_bootStatus |= CPU1_BOOTROM_BOOTSTS_IN_FLASH_BOOT;
            }
            break;
    }
    //
    // Update boot status - Boot Complete
    //
    CPU1BROM_bootStatus |= CPU1_BOOTROM_BOOT_COMPLETE;

    //
    // Enter specified wait boot or enable watchdog, then branch to address
    //
    if(CPU1BROM_bootMode == WAIT_BOOT)
    {
        SysCtl_enableWatchdog();
        asm("   ESTOP0");
        for(;;)
        {
        }
    }
    else if(CPU1BROM_bootMode == WAIT_BOOT_ALT1)
    {
        asm("   ESTOP0");
        for(;;)
        {
        }
    }
    else
    {
        SysCtl_enableWatchdog();
        return(entryAddress);
    }
}

