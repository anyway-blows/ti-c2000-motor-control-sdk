//###########################################################################
//
// FILE:    cpubrom_dcsm.c
//
// TITLE:   main boot loader file for C-Core
//
// Functions:
//
//###########################################################################
// $TI Release: $
// $Release Date:  $
//###########################################################################

//
// Included Files
//
#include "cpu1bootrom.h"

uint32_t Z1_ZSB;
uint32_t Z2_ZSB;
uint32_t z1_key1;
uint32_t z2_key1;
uint32_t Gather_Z1_ZSB (void);
uint32_t Gather_Z2_ZSB (void);

const uint32_t dcsm_default_keys_z1[] = {
		0x47ffffffUL,			//Z1-ZSB0-CSMPSWD1
		0xdb7fffffUL,			//Z1-ZSB1-CSMPSWD1
		0x4bffffffUL,			//Z1-ZSB2-CSMPSWD1
		0x3f7fffffUL,			//Z1-ZSB3-CSMPSWD1
		0xcfbfffffUL,			//Z1-ZSB4-CSMPSWD1
		0x8bffffffUL,			//Z1-ZSB5-CSMPSWD1
		0x53ffffffUL,			//Z1-ZSB6-CSMPSWD1
		0xcf7fffffUL,			//Z1-ZSB7-CSMPSWD1
		0xe77fffffUL,			//Z1-ZSB8-CSMPSWD1
		0x93ffffffUL,			//Z1-ZSB9-CSMPSWD1
		0xeb7fffffUL,			//Z1-ZSB10-CSMPSWD1
		0x69ffffffUL,			//Z1-ZSB11-CSMPSWD1
		0xa9ffffffUL,			//Z1-ZSB12-CSMPSWD1
		0xdd7fffffUL,			//Z1-ZSB13-CSMPSWD1
		0x8bffffffUL,			//Z1-ZSB14-CSMPSWD1
		0xcfbfffffUL,			//Z1-ZSB15-CSMPSWD1
		0x3f7fffffUL,			//Z1-ZSB16-CSMPSWD1
		0x4bffffffUL,			//Z1-ZSB17-CSMPSWD1
		0xdb7fffffUL,			//Z1-ZSB18-CSMPSWD1
		0x47ffffffUL,			//Z1-ZSB19-CSMPSWD1
		0x87ffffffUL,			//Z1-ZSB20-CSMPSWD1
		0xf37fffffUL,			//Z1-ZSB21-CSMPSWD1
		0xdd7fffffUL,			//Z1-ZSB22-CSMPSWD1
		0xa9ffffffUL,			//Z1-ZSB23-CSMPSWD1
		0x69ffffffUL,			//Z1-ZSB24-CSMPSWD1
		0xeb7fffffUL,			//Z1-ZSB25-CSMPSWD1
		0x93ffffffUL,			//Z1-ZSB26-CSMPSWD1
		0xe77fffffUL,			//Z1-ZSB27-CSMPSWD1
		0xcf7fffffUL,			//Z1-ZSB28-CSMPSWD1
		0x53ffffffUL			//Z1-ZSB29-CSMPSWD1
};

const uint32_t dcsm_default_keys_z2[] = {
		0xe3ffffffUL, 		//Z2-ZSB0-CSMPSWD1
		0x977fffffUL, 		//Z2-ZSB1-CSMPSWD1
		0xf1ffffffUL, 		//Z2-ZSB2-CSMPSWD1
		0x9b7fffffUL, 		//Z2-ZSB3-CSMPSWD1
		0x5b7fffffUL, 		//Z2-ZSB4-CSMPSWD1
		0x2fffffffUL, 		//Z2-ZSB5-CSMPSWD1
		0x1fffffffUL, 		//Z2-ZSB6-CSMPSWD1
		0x6b7fffffUL, 		//Z2-ZSB7-CSMPSWD1
		0xab7fffffUL, 		//Z2-ZSB8-CSMPSWD1
		0x37ffffffUL, 		//Z2-ZSB9-CSMPSWD1
		0x4f7fffffUL, 		//Z2-ZSB10-CSMPSWD1
		0x3bffffffUL, 		//Z2-ZSB11-CSMPSWD1
		0xe5ffffffUL, 		//Z2-ZSB12-CSMPSWD1
		0x8f7fffffUL, 		//Z2-ZSB13-CSMPSWD1
		0x2fffffffUL, 		//Z2-ZSB14-CSMPSWD1
		0x5b7fffffUL, 		//Z2-ZSB15-CSMPSWD1
		0x9b7fffffUL, 		//Z2-ZSB16-CSMPSWD1
		0xf1ffffffUL, 		//Z2-ZSB17-CSMPSWD1
		0x977fffffUL, 		//Z2-ZSB18-CSMPSWD1
		0xe3ffffffUL, 		//Z2-ZSB19-CSMPSWD1
		0xcbffffffUL, 		//Z2-ZSB20-CSMPSWD1
		0x577fffffUL, 		//Z2-ZSB21-CSMPSWD1
		0x8f7fffffUL, 		//Z2-ZSB22-CSMPSWD1
		0xe5ffffffUL, 		//Z2-ZSB23-CSMPSWD1
		0x3bffffffUL, 		//Z2-ZSB24-CSMPSWD1
		0x4f7fffffUL, 		//Z2-ZSB25-CSMPSWD1
		0x37ffffffUL, 		//Z2-ZSB26-CSMPSWD1
		0xab7fffffUL, 		//Z2-ZSB27-CSMPSWD1
		0x6b7fffffUL, 		//Z2-ZSB28-CSMPSWD1
		0x1fffffffUL	    //Z2-ZSB29-CSMPSWD1
};



//#################################################
// void CPU1BROM_initDCSM(void)
//--------------------------------------------
// This function initializes code security module, until this function is executed
// all access to RAM and JTAG is blocked.
//--------------------------------------------
void CPU1BROM_initDCSM(void)
{


    Z1_ZSB = 0x0UL;
	Z2_ZSB = 0x0UL;
	z1_key1 = 0x0UL;
	z2_key1 = 0x0UL;

	EALLOW;

	HWREG(DCSMBANK0_Z1OTP_BASE + DCSM_O_B0_Z1OTP_LINKPOINTER1);	//Bank 1 Zone 1 Contents
	HWREG(DCSMBANK0_Z1OTP_BASE + DCSM_O_B0_Z1OTP_LINKPOINTER2);
	HWREG(DCSMBANK0_Z1OTP_BASE + DCSM_O_B0_Z1OTP_LINKPOINTER3);

	HWREG(DCSMBANK0_Z2OTP_BASE + DCSM_O_B0_Z2OTP_LINKPOINTER1);	//Bank 1 Zone 2 Contents
	HWREG(DCSMBANK0_Z2OTP_BASE + DCSM_O_B0_Z2OTP_LINKPOINTER2);
	HWREG(DCSMBANK0_Z2OTP_BASE + DCSM_O_B0_Z2OTP_LINKPOINTER3);

	HWREG(TI_OTP_SECDC);	// TI OTP SECDC register read

	// OTPSECLOCK and other boot related register reads from Zone 1 and Zone 2 of USER OTP1
	HWREG(DCSMBANK0_Z1OTP_BASE + DCSM_O_Z1OTP_PSWDLOCK);
	HWREG(DCSMBANK0_Z1OTP_BASE + DCSM_O_Z1OTP_JTAGLOCK);
	HWREG(DCSMBANK0_Z1OTP_BASE + DCSM_O_Z1OTP_CRCLOCK);
	HWREG(DCSMBANK0_Z1OTP_BASE + DCSM_O_Z1OTP_BOOTPIN_CONFIG);
	HWREG(DCSMBANK0_Z1OTP_BASE + DCSM_O_Z1OTP_GPREG2);
	HWREG(DCSMBANK0_Z1OTP_BASE + DCSM_O_Z1OTP_BOOTDEF_LOW);
	HWREG(DCSMBANK0_Z1OTP_BASE + DCSM_O_Z1OTP_BOOTDEF_HIGH);

	HWREG(DCSMBANK0_Z2OTP_BASE + DCSM_O_Z2OTP_PSWDLOCK);
	HWREG(DCSMBANK0_Z2OTP_BASE + DCSM_O_Z2OTP_JTAGLOCK);
	HWREG(DCSMBANK0_Z2OTP_BASE + DCSM_O_Z2OTP_CRCLOCK);
	HWREG(DCSMBANK0_Z2OTP_BASE + DCSM_O_Z1OTP_BOOTPIN_CONFIG);
	HWREG(DCSMBANK0_Z2OTP_BASE + DCSM_O_Z1OTP_GPREG2);
	HWREG(DCSMBANK0_Z2OTP_BASE + DCSM_O_Z1OTP_BOOTDEF_LOW);
	HWREG(DCSMBANK0_Z2OTP_BASE + DCSM_O_Z1OTP_BOOTDEF_HIGH);

	Z1_ZSB = Gather_Z1_ZSB();	//Gather ZSB of ZONE1 from BANK0
	Z2_ZSB = Gather_Z2_ZSB();	//Gather ZSB of ZONE2 from BANK0

	//Bank 0 Zone 1
	HWREG(Z1_ZSB + DCSM_O_Zx_EXEONLYRAM);	// Zone Select Block contents
	HWREG(Z1_ZSB + DCSM_O_Zx_EXEONLYSECT);
	HWREG(Z1_ZSB + DCSM_O_Zx_GRABRAM);
	HWREG(Z1_ZSB + DCSM_O_Zx_GRABSECT);

	//Bank 0 Zone 2
	HWREG(Z2_ZSB + DCSM_O_Zx_EXEONLYRAM);	// Zone Select Block contents
	HWREG(Z2_ZSB + DCSM_O_Zx_EXEONLYSECT);
	HWREG(Z2_ZSB + DCSM_O_Zx_GRABRAM);
	HWREG(Z2_ZSB + DCSM_O_Zx_GRABSECT);

    //BLOCKED State --> LOCKED State done

    HWREG(Z1_ZSB + DCSM_O_Z1_CSMPSWD0);		// Zone 1 CSMPSWD reads
	HWREG(Z1_ZSB + DCSM_O_Z1_CSMPSWD1);
	HWREG(Z1_ZSB + DCSM_O_Z1_CSMPSWD2);
	HWREG(Z1_ZSB + DCSM_O_Z1_CSMPSWD3);

	HWREG(Z2_ZSB + DCSM_O_Z2_CSMPSWD0);		// Zone 2 CSMPSWD reads
	HWREG(Z2_ZSB + DCSM_O_Z2_CSMPSWD1);
	HWREG(Z2_ZSB + DCSM_O_Z2_CSMPSWD2);
	HWREG(Z2_ZSB + DCSM_O_Z2_CSMPSWD3);

	//LOCKED State --> ARMED State done

	if((HWREGH(DCSMBANK0_Z1_BASE + DCSM_O_Z1_CR) & DCSM_Z1_CR_UNSECURE) == 0UL)
    {
		//LOCKED State --> ARMED State
		HWREG(DCSMBANK0_Z1_BASE + DCSM_O_Z1_CSMKEY0) = 0xFFFFFFFFUL;	//Zone 1 CSMKEY Loads
		HWREG(DCSMBANK0_Z1_BASE + DCSM_O_Z1_CSMKEY1) = (uint32_t) z1_key1;
		HWREG(DCSMBANK0_Z1_BASE + DCSM_O_Z1_CSMKEY2) = 0xFFFFFFFFUL;
		HWREG(DCSMBANK0_Z1_BASE + DCSM_O_Z1_CSMKEY3) = 0xFFFFFFFFUL;
    }

	if((HWREGH(DCSMBANK0_Z2_BASE + DCSM_O_Z2_CR) & DCSM_Z2_CR_UNSECURE) == 0UL)
    {
		HWREG(DCSMBANK0_Z2_BASE + DCSM_O_Z2_CSMKEY0) = 0xFFFFFFFFUL;	//Zone 2 CSMKEY Loads
		HWREG(DCSMBANK0_Z2_BASE + DCSM_O_Z2_CSMKEY1) = (uint32_t) z2_key1;
		HWREG(DCSMBANK0_Z2_BASE + DCSM_O_Z2_CSMKEY2) = 0xFFFFFFFFUL;
		HWREG(DCSMBANK0_Z2_BASE + DCSM_O_Z2_CSMKEY3) = 0xFFFFFFFFUL;
    }
    
    //
    // Disable writing to the EALLOW protected registers
    //
    EDIS;

	//ARMED State --> Zones UNLOCKED State done
}


uint32_t Gather_Z1_ZSB (void)
{
    uint32_t linkPointer;
    uint32_t ZSBBase = 0; // base address of the ZSB
    int32_t bitPos = 28;
    int32_t zeroFound = 0;

	linkPointer = HWREG(DCSMBANK0_Z1_BASE + DCSM_O_B0_Z1_LINKPOINTER);
	linkPointer = linkPointer << 3; // Bits 31 and 30 as most-sigificant 0 are
                        			//invalid LinkPointer options
	while((zeroFound == 0) && (bitPos > -1))
	{
		if( (linkPointer & 0x80000000U) == 0U)
		{
			zeroFound = 1;
			ZSBBase = (DCSMBANK0_Z1OTP_BASE + (((uint32_t)bitPos + 3U) * 0x10U));
		}
		else
		{
			bitPos--;
			linkPointer = linkPointer << 1;
		}
	}
	if(zeroFound == 0)
	{
		ZSBBase = (DCSMBANK0_Z1OTP_BASE + 0x20U);
	}

	z1_key1 = dcsm_default_keys_z1[bitPos+1];

    return ZSBBase;
}


uint32_t Gather_Z2_ZSB(void)
{
    uint32_t linkPointer;
    uint32_t ZSBBase = 0; // base address of the ZSB
    int32_t bitPos = 28;
    int32_t zeroFound = 0;

	linkPointer = HWREG(DCSMBANK0_Z2_BASE + DCSM_O_B0_Z2_LINKPOINTER);
	linkPointer = linkPointer << 3; // Bits 31 and 30 as most-sigificant 0 are
									//invalid LinkPointer options
	while((zeroFound == 0) && (bitPos > -1))
	{
		if( (linkPointer & 0x80000000U) == 0U)
		{
			zeroFound = 1;
			ZSBBase = (DCSMBANK0_Z2OTP_BASE + (((uint32_t)bitPos + 3U) * 0x10U));
		}
		else
		{
			bitPos--;
			linkPointer = linkPointer << 1;
		}
	}
	if(zeroFound == 0)
	{
		ZSBBase = (DCSMBANK0_Z2OTP_BASE + 0X20U);
	}

	z2_key1 = dcsm_default_keys_z2[bitPos+1];

    return ZSBBase;
}

//===========================================================================
// End of file.
//===========================================================================
