//###########################################################################
//
// FILE:    Shared_Boot.c
//
// TITLE:   Boot loader shared functions
//
// Functions:
//
//     void   CopyData(void)
//     Uint32 GetLongData(void)
//     void ReadReservedFn(void)
//
//###########################################################################
// $TI Release: F2837xS Support Library v3.12.00.00 $
// $Release Date: Fri Feb 12 19:06:50 IST 2021 $
// $Copyright:
// Copyright (C) 2014-2021 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

#include "c1_bootrom.h"
#include "F2837xS_Examples.h"
#include "F021_F2837xS_C28x.h"
//Include Flash API example header file
#include "flash_programming_c28.h"

// GetWordData is a pointer to the function that interfaces to the peripheral.
// Each loader assigns this pointer to it's particular GetWordData function.
uint16fptr GetWordData;

typedef struct
{
   Uint16 status;
   Uint32 address;
}  StatusCode;
extern StatusCode statusCode;

#define NO_ERROR					0x1000
#define BLANK_ERROR					0x2000
#define VERIFY_ERROR				0x3000
#define PROGRAM_ERROR				0x4000
#define COMMAND_ERROR				0x5000
#define UNLOCK_ERROR				0x6000

#define BUFFER_SIZE					0x80 //because of ECC, must be multiple of 128 bits, or 8 words, BUFFER_SIZE % 8 == 0

#define FLASH_BANK0                 0
#define FLASH_BANK1                 1

// Function prototypes
extern void SCI_SendChecksum(void);
Uint32 GetLongData(void);
void CopyData(void);
void ReadReservedFn(void);
Uint32 FindSector(Uint32 address);
Uint16 FindSize(Uint32 address);
Uint16 FindBank(Uint32 address);
void Example_Error(Fapi_StatusType status);
void Init_Flash_Bank0_Sectors(void);
void Init_Flash_Bank1_Sectors(void);

unsigned char erasedAlready[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

Uint16 bank0Initialized = 0;
Uint16 bank1Initialized = 0;

//#################################################
// void CopyData(void)
//-----------------------------------------------------
// This routine copies multiple blocks of data from the host
// to the specified RAM locations.  There is no error
// checking on any of the destination addresses.
// That is it is assumed all addresses and block size
// values are correct.
//
// Multiple blocks of data are copied until a block
// size of 00 00 is encountered.
//-----------------------------------------------------
void CopyData()
{
	struct HEADER
	{
		Uint16 BlockSize;
		Uint32 DestAddr;
	} BlockHeader;

	Uint16 Buffer[BUFFER_SIZE];
    Uint16 miniBuffer[8]; //useful for 8-word access to flash with
	Uint16 i = 0;
	Uint16 j = 0;
	Uint16 k = 0;
	Uint32 sectorAddress;
	Uint16 sectorSize;
	Uint16 wordData;
	Uint16 flashBank;
	int fail = 0;

    assert(BUFFER_SIZE % 8 == 0);

	if(statusCode.status != NO_ERROR)
	{
		fail++;
	}

	//Reset erase status of all flash sectors
	for(i = 0; i < 28; i++)
	{
		erasedAlready[i] = 0;
	}

	bank0Initialized = 0;
	bank1Initialized = 0;

	// Send checksum to satisfy before we begin
#if checksum_enable
	SCI_SendChecksum();
#endif

	// Get the size in words of the first block
	BlockHeader.BlockSize = (*GetWordData)();

	// While the block size is > 0 copy the data
	// to the DestAddr.  There is no error checking
	// as it is assumed the DestAddr is a valid
	// memory location

	EALLOW;
	while(BlockHeader.BlockSize != (Uint16)0x0000)
	{
	   Fapi_StatusType oReturnCheck;
	   Fapi_FlashStatusWordType oFlashStatusWord;
	   Fapi_FlashStatusType oFlashStatus;
	   BlockHeader.DestAddr = GetLongData();
	   for(i = 0; i < BlockHeader.BlockSize; i += 0)
	   {
		   if(BlockHeader.BlockSize < BUFFER_SIZE)
		   {
			   for(j = 0; j < BlockHeader.BlockSize; j++)
			   {
				   wordData = (*GetWordData)();
				   Buffer[j] = wordData;
				   i++;
			   }
			   for(j = BlockHeader.BlockSize; j < BUFFER_SIZE; j++)
			   {
				   Buffer[j] = 0xFFFF;
			   }
		   }
		   else //BlockHeader.BlockSize >= BUFFER_SIZE
		   {
			   if((BlockHeader.BlockSize - i) < BUFFER_SIZE) //less than one BUFFER_SIZE left
			   {
				   for(j = 0; j < BlockHeader.BlockSize - i; j++) //fill Buffer with rest of data
				   {
					   wordData = (*GetWordData)();
					   Buffer[j] = wordData;
				   }
				   i += j; //increment i outside here so it doesn't affect loop above
				   for(; j < BUFFER_SIZE; j++)//fill the rest with 0xFFFF
				   {
					   Buffer[j] = 0xFFFF;
				   }
			   }
			   else
			   {
				   for(j = 0; j < BUFFER_SIZE; j++) //fill up like normal, up to BUFFER_SIZE
				   {
					   wordData = (*GetWordData)();
					   Buffer[j] = wordData;
					   i++;
				   }
			   }
		   }
           for(k = 0; k < (BUFFER_SIZE / 8); k++)
           {
               miniBuffer[0] = Buffer[k * 8 + 0];
               miniBuffer[1] = Buffer[k * 8 + 1];
               miniBuffer[2] = Buffer[k * 8 + 2];
               miniBuffer[3] = Buffer[k * 8 + 3];
               miniBuffer[4] = Buffer[k * 8 + 4];
               miniBuffer[5] = Buffer[k * 8 + 5];
               miniBuffer[6] = Buffer[k * 8 + 6];
               miniBuffer[7] = Buffer[k * 8 + 7];
			   //check that miniBuffer is not already all erased data
               if(!((miniBuffer[0] == 0xFFFF) && (miniBuffer[1] == 0xFFFF) &&
                    (miniBuffer[2] == 0xFFFF) && (miniBuffer[3] == 0xFFFF) &&
					(miniBuffer[4] == 0xFFFF) && (miniBuffer[5] == 0xFFFF) &&
                    (miniBuffer[6] == 0xFFFF) && (miniBuffer[7] == 0xFFFF) ))
			   {
				    //Initialize flash banks if there is a change in the bank
				    flashBank = FindBank(BlockHeader.DestAddr);
				    if(flashBank != 0xbeef)
				    {
					    if(flashBank == FLASH_BANK0)
						    Init_Flash_Bank0_Sectors();
					    else // flashBank == FLASH_BANK1
						    Init_Flash_Bank1_Sectors();
				    }

				    //clean out flash banks if needed
					sectorAddress = FindSector(BlockHeader.DestAddr);
					if(sectorAddress != 0xdeadbeef)
					{
						sectorSize = FindSize(sectorAddress);
						oReturnCheck = Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector,
								(uint32 *)sectorAddress);
					    while(Fapi_checkFsmForReady() == Fapi_Status_FsmBusy);
						oReturnCheck = Fapi_doBlankCheck((uint32 *)sectorAddress,
								sectorSize,
								&oFlashStatusWord);
						if(oReturnCheck != Fapi_Status_Success)
						{
							if(fail == 0) //first fail
							{
								statusCode.status = BLANK_ERROR;
								statusCode.address = oFlashStatusWord.au32StatusWord[0];
							}
							fail++;
						}
					}
					//program 8 words at once, 128-bits
					oReturnCheck = Fapi_issueProgrammingCommand((uint32 *)BlockHeader.DestAddr,
							miniBuffer,
							sizeof(miniBuffer),
							0,
							0,
							Fapi_AutoEccGeneration);
					while(Fapi_checkFsmForReady() == Fapi_Status_FsmBusy);
					oFlashStatus = Fapi_getFsmStatus();
					if((oReturnCheck != Fapi_Status_Success) || (oFlashStatus != 0))
					{
						if(fail == 0) //first fail
						{
							statusCode.status = PROGRAM_ERROR;
							statusCode.address = oFlashStatusWord.au32StatusWord[0];
						}
					   fail++;
					}

                    for(j = 0; j < 8; j += 2)
					{
					   Uint32 toVerify = miniBuffer[j+1];
					   toVerify = toVerify << 16;
					   toVerify |= miniBuffer[j];
					   oReturnCheck = Fapi_doVerify((uint32 *)(BlockHeader.DestAddr+j),
							  1,
							  (uint32 *)(&toVerify),
							  &oFlashStatusWord);
					   if(oReturnCheck != Fapi_Status_Success)
					   {
						   if(fail == 0) //first fail
						   {
								statusCode.status = VERIFY_ERROR;
								statusCode.address = oFlashStatusWord.au32StatusWord[0];
						   }
						   fail++;
					   }
					} //for j; for Fapi_doVerify
			   } //check if miniBuffer does not contain all already erased data
               BlockHeader.DestAddr += 0x8;
		   } //for(int k); loads miniBuffer with Buffer elements
#if checksum_enable
		   SCI_SendChecksum();
#endif
	   }
	   //get the size of the next block
	   BlockHeader.BlockSize = (*GetWordData)();
	}
	EDIS;
	return;
}

//#################################################
// Uint32 FindSector(Uint32 address)
//-----------------------------------------------------
// This routine finds what sector the mentioned address
// is a part of.
//-----------------------------------------------------
Uint32 FindSector(Uint32 address){

    if((address >= Bzero_SectorA_start) && (address <= Bzero_SectorN_End))
    {
        if((address >= Bzero_SectorA_start) && (address <= Bzero_SectorA_End)
                && (erasedAlready[0] == 0)){
            erasedAlready[0] = 1;
            return (Uint32)Bzero_SectorA_start;
        }
        else if((address >= Bzero_SectorB_start) && (address <= Bzero_SectorB_End)
                && (erasedAlready[1] == 0)){
            erasedAlready[1] = 1;
            return (Uint32)Bzero_SectorB_start;
        }
        else if((address >= Bzero_SectorC_start) && (address <= Bzero_SectorC_End)
                && (erasedAlready[2] == 0)){
            erasedAlready[2] = 1;
            return (Uint32)Bzero_SectorC_start;
        }
        else if((address >= Bzero_SectorD_start) && (address <= Bzero_SectorD_End)
                && (erasedAlready[3] == 0)){
            erasedAlready[3] = 1;
            return (Uint32)Bzero_SectorD_start;
        }
        else if((address >= Bzero_SectorE_start) && (address <= Bzero_SectorE_End)
                && (erasedAlready[4] == 0)){
            erasedAlready[4] = 1;
            return (Uint32)Bzero_SectorE_start;
        }
        else if((address >= Bzero_SectorF_start) && (address <= Bzero_SectorF_End)
                && (erasedAlready[5] == 0)){
            erasedAlready[5] = 1;
            return (Uint32)Bzero_SectorF_start;
        }
        else if((address >= Bzero_SectorG_start) && (address <= Bzero_SectorG_End)
                && (erasedAlready[6] == 0)){
            erasedAlready[6] = 1;
            return (Uint32)Bzero_SectorG_start;
        }
        else if((address >= Bzero_SectorH_start) && (address <= Bzero_SectorH_End)
                && (erasedAlready[7] == 0)){
            erasedAlready[7] = 1;
            return (Uint32)Bzero_SectorH_start;
        }
        else if((address >= Bzero_SectorI_start) && (address <= Bzero_SectorI_End)
                && (erasedAlready[8] == 0)){
            erasedAlready[8] = 1;
            return (Uint32)Bzero_SectorI_start;
        }
        else if((address >= Bzero_SectorJ_start) && (address <= Bzero_SectorJ_End)
                && (erasedAlready[9] == 0)){
            erasedAlready[9] = 1;
            return (Uint32)Bzero_SectorJ_start;
        }
        else if((address >= Bzero_SectorK_start) && (address <= Bzero_SectorK_End)
                && (erasedAlready[10] == 0)){
            erasedAlready[10] = 1;
            return (Uint32)Bzero_SectorK_start;
        }
        else if((address >= Bzero_SectorL_start) && (address <= Bzero_SectorL_End)
                && (erasedAlready[11] == 0)){
            erasedAlready[11] = 1;
            return (Uint32)Bzero_SectorL_start;
        }
        else if((address >= Bzero_SectorM_start) && (address <= Bzero_SectorM_End)
                && (erasedAlready[12] == 0)){
            erasedAlready[12] = 1;
            return (Uint32)Bzero_SectorM_start;
        }
        else if((address >= Bzero_SectorN_start) && (address <= Bzero_SectorN_End)
                && (erasedAlready[13] == 0)){
            erasedAlready[13] = 1;
            return (Uint32)Bzero_SectorN_start;
        }
        else{
            return 0xdeadbeef; //a proxy address to signify that it is not a flash sector
        }
    }
    else if((address >= BOne_SectorO_start) && (address <= BOne_SectorAB_End))
    {
        if((address >= BOne_SectorO_start) && (address <= BOne_SectorO_End)
                && (erasedAlready[14] == 0)){
            erasedAlready[14] = 1;
            return (Uint32)BOne_SectorO_start;
        }
        else if((address >= BOne_SectorP_start) && (address <= BOne_SectorP_End)
                && (erasedAlready[15] == 0)){
            erasedAlready[15] = 1;
            return (Uint32)BOne_SectorP_start;
        }
        else if((address >= BOne_SectorQ_start) && (address <= BOne_SectorQ_End)
                && (erasedAlready[16] == 0)){
            erasedAlready[16] = 1;
            return (Uint32)BOne_SectorQ_start;
        }
        else if((address >= BOne_SectorR_start) && (address <= BOne_SectorR_End)
                && (erasedAlready[17] == 0)){
            erasedAlready[17] = 1;
            return (Uint32)BOne_SectorR_start;
        }
        else if((address >= BOne_SectorS_start) && (address <= BOne_SectorS_End)
                && (erasedAlready[18] == 0)){
            erasedAlready[18] = 1;
            return (Uint32)BOne_SectorS_start;
        }
        else if((address >= BOne_SectorT_start) && (address <= BOne_SectorT_End)
                && (erasedAlready[19] == 0)){
            erasedAlready[19] = 1;
            return (Uint32)BOne_SectorT_start;
        }
        else if((address >= BOne_SectorU_start) && (address <= BOne_SectorU_End)
                && (erasedAlready[20] == 0)){
            erasedAlready[20] = 1;
            return (Uint32)BOne_SectorU_start;
        }
        else if((address >= BOne_SectorV_start) && (address <= BOne_SectorV_End)
                && (erasedAlready[21] == 0)){
            erasedAlready[21] = 1;
            return (Uint32)BOne_SectorV_start;
        }
        else if((address >= BOne_SectorW_start) && (address <= BOne_SectorW_End)
                && (erasedAlready[22] == 0)){
            erasedAlready[22] = 1;
            return (Uint32)BOne_SectorW_start;
        }
        else if((address >= BOne_SectorX_start) && (address <= BOne_SectorX_End)
                && (erasedAlready[23] == 0)){
            erasedAlready[23] = 1;
            return (Uint32)BOne_SectorX_start;
        }
        else if((address >= BOne_SectorY_start) && (address <= BOne_SectorY_End)
                && (erasedAlready[24] == 0)){
            erasedAlready[24] = 1;
            return (Uint32)BOne_SectorY_start;
        }
        else if((address >= BOne_SectorZ_start) && (address <= BOne_SectorZ_End)
                && (erasedAlready[25] == 0)){
            erasedAlready[25] = 1;
            return (Uint32)BOne_SectorZ_start;
        }
        else if((address >= BOne_SectorAA_start) && (address <= BOne_SectorAA_End)
                && (erasedAlready[26] == 0)){
            erasedAlready[26] = 1;
            return (Uint32)BOne_SectorAA_start;
        }
        else if((address >= BOne_SectorAB_start) && (address <= BOne_SectorAB_End)
                && (erasedAlready[27] == 0)){
            erasedAlready[27] = 1;
            return (Uint32)BOne_SectorAB_start;
        }
        else{
            return 0xdeadbeef; //a proxy address to signify that it is not a flash sector
        }
    }
    else{
        return 0xdeadbeef; //a proxy address to signify that it is not a flash sector
    }
}

//#################################################
// Uint16 FindBank(Uint32 address)
//-----------------------------------------------------
// This routine finds what bank the mentioned address
// is a part of.
//-----------------------------------------------------
Uint16 FindBank(Uint32 address){

    if((address >= Bzero_SectorA_start) && (address <= Bzero_SectorN_End)
            && (bank0Initialized == 0))
    {
        bank0Initialized = 1;
        bank1Initialized = 0;
        return (Uint16)FLASH_BANK0;
    }
    else if((address >= BOne_SectorO_start) && (address <= BOne_SectorAB_End)
            && (bank1Initialized == 0))
    {
        bank1Initialized = 1;
        bank0Initialized = 0;
        return (Uint16)FLASH_BANK1;
    }
    else
        return 0xbeef;
}

//#################################################
// Uint16 FindSize(Uint32 address)
//-----------------------------------------------------
// This routine finds the size of the sector under use.
//-----------------------------------------------------
Uint16 FindSize(Uint32 address){ //set erasedAlready
	if(((address >= Bzero_SectorA_start) && (address <= Bzero_SectorD_start)) ||
       ((address >= Bzero_SectorK_start) && (address <= BOne_SectorR_start)) ||
       ((address >= BOne_SectorY_start)  && (address <= BOne_SectorAB_start)))
        return Bzero_16KSector_u32length;
	else if(((address >= Bzero_SectorE_start) && (address <= Bzero_SectorJ_start)) ||
            ((address >= BOne_SectorS_start)  && (address == BOne_SectorX_start)))
        return Bzero_64KSector_u32length;
	//no other possible case
	return 0xbeef;
}

//#################################################
// Uint32 GetLongData(void)
//-----------------------------------------------------
// This routine fetches a 32-bit value from the peripheral
// input stream.
//-----------------------------------------------------
Uint32 GetLongData()
{
    Uint32 longData;

    // Fetch the upper 1/2 of the 32-bit value
    longData = ( (Uint32)(*GetWordData)() << 16);

    // Fetch the lower 1/2 of the 32-bit value
    longData |= (Uint32)(*GetWordData)();

    return longData;
}

//#################################################
// void Read_ReservedFn(void)
//-------------------------------------------------
// This function reads 8 reserved words in the header.
// None of these reserved words are used by the
// this boot loader at this time, they may be used in
// future devices for enhancements.  Loaders that use
// these words use their own read function.
//-------------------------------------------------
void ReadReservedFn()
{
    Uint16 i;
    // Read and discard the 8 reserved words.
    for(i = 1; i <= 8; i++)
    {
       GetWordData();
    }
    return;
}

//
// Init_Flash_Bank0_Sectors - Initialize flash bank 0 sectors
//
void Init_Flash_Bank0_Sectors(void)
{
    Fapi_StatusType oReturnCheck;

    //
    // Gain pump semaphore for Bank0.
    //
    SeizeFlashPump_Bank0();

    EALLOW;

    #if CPU_FRQ_200MHZ
    oReturnCheck = Fapi_initializeAPI(F021_CPU0_W0_BASE_ADDRESS, 200);
    #endif

    #if CPU_FRQ_150MHZ
    oReturnCheck = Fapi_initializeAPI(F021_CPU0_W0_BASE_ADDRESS, 150);
    #endif

    #if CPU_FRQ_120MHZ
    oReturnCheck = Fapi_initializeAPI(F021_CPU0_W0_BASE_ADDRESS, 120);
    #endif

    if(oReturnCheck != Fapi_Status_Success)
    {
        Example_Error(oReturnCheck);
    }

    oReturnCheck = Fapi_setActiveFlashBank(Fapi_FlashBank0);
    if(oReturnCheck != Fapi_Status_Success)
    {
        Example_Error(oReturnCheck);
    }

    EDIS;
}

//
// Init_Flash_Bank1_Sectors - Initialize flash bank 1 sectors
//
void Init_Flash_Bank1_Sectors(void)
{
    Fapi_StatusType oReturnCheck;

    //
    // Gain pump semaphore for Bank1.
    //
    SeizeFlashPump_Bank1();

    EALLOW;

    #if CPU_FRQ_200MHZ
    oReturnCheck = Fapi_initializeAPI(F021_CPU0_W1_BASE_ADDRESS, 200);
    #endif

    #if CPU_FRQ_150MHZ
    oReturnCheck = Fapi_initializeAPI(F021_CPU0_W1_BASE_ADDRESS, 150);
    #endif

    #if CPU_FRQ_120MHZ
    oReturnCheck = Fapi_initializeAPI(F021_CPU0_W1_BASE_ADDRESS, 120);
    #endif

    if(oReturnCheck != Fapi_Status_Success)
    {
        Example_Error(oReturnCheck);
    }

    oReturnCheck = Fapi_setActiveFlashBank(Fapi_FlashBank1);
    if(oReturnCheck != Fapi_Status_Success)
    {
        Example_Error(oReturnCheck);
    }

    EDIS;
}
