//###########################################################################
//
// FILE:   lin.c
//
// TITLE:  C28x LIN driver.
//
//###########################################################################
// $TI Release:  $
// $Release Date:  $
// $Copyright:  $
//###########################################################################
#if __TI_COMPILER_VERSION__ >= 15006000
#include "lin.h"

//*****************************************************************************
//
// LIN_initModule
//
//*****************************************************************************
void
LIN_initModule(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(LIN_isBaseValid(base));

    EALLOW;

    //
    // Reset LIN module
    // Release from hard reset
    //
    LIN_disableModule(base);
    LIN_enableModule(base);

    //
    // Enter Software Reset State
    //
    LIN_enterSoftwareReset(base);

    //
    // Enable LIN Mode
    //
    LIN_disableSCIMode(base);

    //
    // Set LIN mode to Master
    //
    LIN_setLINMode(base, LIN_MODE_LIN_MASTER);

    //
    // Enable Fixed baud rate mode
    //
    LIN_disableAutomaticBaudrate(base);

    //
    // Use the set frame length and not ID4/ID5 bits for length control
    //
    LIN_setCommMode(base, LIN_COMM_LIN_USELENGTHVAL);

    //
    // Setup to continue operating on emulation suspend
    //
    LIN_setDebugSuspendMode(base, LIN_DEBUG_COMPLETE);

    //
    // Use Enhanced Checksum
    //
    LIN_setChecksumType(base, LIN_CHECKSUM_ENHANCED);

    //
    // Message filtering uses slave task ID byte
    //
    LIN_setMessageFiltering(base, LIN_MSG_FILTER_IDSLAVE);

    //
    // Disable Internal loopback for external communication
    //
    LIN_disableIntLoopback(base);

    //
    // Enable multi-buffer mode
    //
    LIN_enableMultibufferMode(base);

    //
    // Enable parity check on received ID
    //
    LIN_enableParity(base);

    //
    // Enable transfer of data to and from the shift registers
    //
    LIN_enableDataTransmitter(base);
    LIN_enableDataReceiver(base);

    //
    // Enable the triggering of checksum compare on extended frames
    //
    LIN_triggerChecksumCompare(base);

    //
    // Set LIN interrupts to disabled
    //
    LIN_disableInterrupt(base, LIN_INT_ALL);

    //
    // Set Baud Rate Settings - 100MHz Device
    //
    LIN_setBaudRatePrescaler(base, 96U, 11U);
    LIN_setMaximumBaudRate(base, 100000000U);

    //
    // Set response field to 1 byte
    //
    LIN_setFrameLength(base, 1U);

    //
    // Configure sync field
    // Sync break (13 + 5 = 18 Tbits)
    // Sync delimiter (1 + 3 = 4 Tbits)
    //
    LIN_setSyncFields(base, 5U, 3U);

    //
    // Set Mask ID so TX/RX match will always happen
    //
    LIN_setTxMask(base, 0xFFU);
    LIN_setRxMask(base, 0xFFU);

    //
    // Disable IODFT testing and external loopback mode
    //
    LIN_disableExtLoopback(base);

    //
    // Finally exit SW reset and enter LIN ready state
    //
    LIN_exitSoftwareReset(base);

    EDIS;
}

//*****************************************************************************
//
// LIN_sendData
//
//*****************************************************************************
void
LIN_sendData(uint32_t base, uint16_t *data)
{
    int16_t i;
    uint16_t length;
    uint16_t *pData;

    //
    // Check the arguments.
    //
    ASSERT(LIN_isBaseValid(base));

    //
    // Get the length from the SCIFORMAT register.
    //
    length = (uint16_t)((HWREG_BP(base + LIN_O_SCIFORMAT) &
                          LIN_SCIFORMAT_LENGTH_M) >> LIN_SCIFORMAT_LENGTH_S);

    pData = data + length;

    //
    // Shift each 8-bit piece of data into the correct register.
    //
    for(i = (int32_t)length; i >= 0; i--)
    {
        if(pData != 0U)
        {
            HWREGB(base + LIN_O_TD0 + ((uint16_t)i ^ 3U)) = *pData;
            pData--;
        }
    }
}

//*****************************************************************************
//
// LIN_getData
//
//*****************************************************************************
void
LIN_getData(uint32_t base, uint16_t * const data)
{
    uint16_t i;
    uint16_t length;
    uint16_t *pData = data;

    //
    // Check the arguments.
    //
    ASSERT(LIN_isBaseValid(base));

    //
    // Get the length from the SCIFORMAT register.
    //
    length = (uint16_t)((HWREG_BP(base + LIN_O_SCIFORMAT) &
                        LIN_SCIFORMAT_LENGTH_M) >> LIN_SCIFORMAT_LENGTH_S);

    //
    // Read each 8-bit piece of data.
    //
    for(i = 0U; i <= length; i++)
    {
        if(pData != 0U)
        {
            *pData = HWREGB(base + LIN_O_RD0 + ((uint32_t)i ^ 3U));
            pData++;
        }
    }
}
#endif
