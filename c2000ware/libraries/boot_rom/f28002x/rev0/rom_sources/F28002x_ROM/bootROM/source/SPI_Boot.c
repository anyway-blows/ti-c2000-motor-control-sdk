//###########################################################################
//
// FILE:    bootloader_spi.c
//
// TITLE:   SPI Bootloader routines
//
// Functions involved in running SPI bootloader
//
// -----------------------------------------------------------------------------
// |Opt No.|  BOOTDEF      |  SPISIMOA  |  SPISOMIA  |  SPICLKA   |  SPISTEA   |
// -----------------------------------------------------------------------------
// |  1    |  0x06(default)|  2         |  1         |  3         |  5         |
// |  2    |  0x26         |  16        |  1         |  3         |  0         |
// |  3    |  0x46         |  8         |  10        |  9         |  11        |
// |  3    |  0x66         |  8         |  17        |  9         |  11        |
// -----------------------------------------------------------------------------
//
//###########################################################################
// $TI Release: $
// $Release Date: $
//###########################################################################

//
// Included Files
//
#include "cpu1bootrom.h"

//
// Defines
//
#define SPI_CHARACTER_LENGTH 8

//
// SPI Bitrate
// Based on 2.5MHz LSPCLK and SPIBRR = 127. Baud Rate = LSPCLK/(SPIBRR + 1)
//
#define SPI_BIT_RATE         19531U 

//
// Function Prototypes
//
static inline uint16_t SPIA_Transmit_Receive(uint16_t cmdData);
static inline void SPIA_ReservedFn(void);
uint16_t SPIA_GetWordData(void);
uint16_t SPIA_SetAddress_KeyChk(void);
uint32_t SPI_start_Boot(uint16_t gpioSTEA);
void SPIA_Init(void);
uint16_t SPIBOOT_configure_gpio(uint32_t bootMode);

//
// SPI_Boot - This module is the main SPI boot routine.
//            It will load code via the SPI-A port.
//
//            It will return a entry point address back
//            to the system initialization routine which calls ExitBoot routine.
//
uint32_t SPI_Boot(uint32_t bootMode)
{
    uint32_t entryAddress;
    uint16_t gpioSTEA;

    //
    // CPU1 Patch/Escape Point 14
    //
    entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_14;
    if((entryAddress != 0xFFFFFFFFUL) &&
       (entryAddress != 0x00000000UL))
    {
        //
        // If OTP is programmed, then call OTP patch function
        //
        ((void (*)(void))entryAddress)();
    }

    //
    // Check to verify SPI-A is present on device
    //
   if(SysCtl_isPeripheralPresent(SYSCTL_PERIPH_PRESENT_SPIA) == false)
    {
       return(FLASH_ENTRY_POINT);
    }

    //
    // Assign GetWordData to the SPI-A version of the
    // function. GetWordData is a pointer to a function.
    //
    GetWordData = SPIA_GetWordData;

    //
    // Initialize SPI-A
    //
    SPIA_Init();

    //
    // Setup SPI GPIOs
    //
    gpioSTEA = SPIBOOT_configure_gpio(bootMode);

    //
    // CPU1 Patch/Escape Point 14
    //
    entryAddress = CPU1BROM_TI_OTP_ESCAPE_POINT_14;
    if((entryAddress != 0xFFFFFFFFUL) &&
       (entryAddress != 0x00000000UL))
    {
        //
        // If OTP is programmed, then call OTP patch function
        //
        ((void (*)(void))entryAddress)();
    }

    return SPI_start_Boot(gpioSTEA);
}

//
// SPI_start_Boot - Function to begin transmission and copying of data into
//                  device memory
//
uint32_t SPI_start_Boot(uint16_t gpioSTEA)
{
    uint32_t entryAddress;

    //
    // Enable EEPROM and send EEPROM Read Command
    //
    (void)SPIA_Transmit_Receive(0x0300U);

    //
    // Send Starting Address for Serial EEPROM (16-bit - 0x0000,0000)
    // or Serial Flash (24-bit - 0x0000,0000,0000)
    // Then check for 0x08AA data header, else go to flash
    //
    if(SPIA_SetAddress_KeyChk() != BROM_EIGHT_BIT_HEADER)
    {
        return FLASH_ENTRY_POINT;
    }

    //
    // Check for Clock speed change and reserved words
    //
    SPIA_ReservedFn();

    //
    // Get point of entry address after load
    //
    entryAddress = GetLongData();

    //
    // Receive and copy one or more code sections to destination addresses
    //
    CopyData();

    //
    // Disable EEPROM chip enable - high
    // Chip enable - high
    //
    GPIO_writePin(gpioSTEA, 1U);

    return(entryAddress);
}

//
// SPIA_SetAddress_KeyChk - This routine sends either a 16-bit address to
//                          Serial EEPROM or a 24-bit address to Serial
//                          FLASH. It then fetches the 2 bytes that make
//                          up the key value  from the SPI-A port and
//                          puts them together to form a single
//                          16-bit value.  It is assumed that the host is
//                          sending the data in the form MSB:LSB.
//
uint16_t SPIA_SetAddress_KeyChk(void)
{
    uint16_t keyValue;

    //
    // Transmit first byte of Serial Memory address
    //
    (void)SPIA_Transmit_Receive(0x0000U);

    //
    // Transmit second byte of Serial Memory address
    //
    (void)SPIA_Transmit_Receive(0x0000U);

    //
    // Transmit third byte of  Serial Memory address (0x00) if using Serial
    // Flash or receive first byte of key value if using Serial EEPROM.
    //
    keyValue = SPIA_Transmit_Receive(0x0000U);

    //
    // If previously received LSB of key value (Serial EEPROM), then fetch
    // MSB of key value
    //
    if(keyValue == 0x00AAU)
    {
        keyValue |= (SPIA_Transmit_Receive(0x0000U) << 8U);
    }
    else
    {
        //
        // Serial Flash is being used - keyValue will be received after 24-bit
        // address in the next 2 bytes
        // Fetch Key Value LSB (after 24-bit addressing)
        //
        keyValue = SPIA_Transmit_Receive(0x0000U);

        //
        // Fetch Key Value MSB (after 24-bit addressing)
        //
        keyValue |= (SPIA_Transmit_Receive(0x0000U) << 8U);
    }

    return(keyValue);
}

//
// SPIA_Transmit_Receive - Send a byte/words through SPI transmit channel
//
static inline uint16_t SPIA_Transmit_Receive(uint16_t cmdData)
{
    uint16_t recvData;

    SPI_writeDataNonBlocking(SPIA_BASE,cmdData);

    while(SPI_getInterruptStatus(SPIA_BASE) != SPI_INT_RX_DATA_TX_EMPTY)
    {
    }

    recvData = SPI_readDataNonBlocking(SPIA_BASE);

    return(recvData);
}

//
// SPIA_ReservedFn - This function reads 8 reserved words in the header.
//                   The first word has parameters for LOSPCP
//                   and SPIBRR register 0xMSB:LSB, LSB = is a three
//                   bit field for LOSPCP change MSB = is a 6bit field
//                   for SPIBRR register update
//
//                   If either byte is the default value of the register
//                   then no speed change occurs.  The default values
//                   are LOSPCP = 0x02 and SPIBRR = 0x7F
//                   The remaining reserved words are read and discarded
//                   and then returns to the main routine.
//
static inline void SPIA_ReservedFn(void)
{
    uint16_t speedData;
    uint16_t i;

    //
    // Update LOSPCP register
    //
    speedData = SPIA_Transmit_Receive((uint16_t)0x0000U);

    EALLOW;
    HWREG(CLKCFG_BASE + SYSCTL_O_LOSPCP) = speedData;
    EDIS;

    asm("   RPT #0x0F ||NOP");

    //
    // Update SPIBRR register
    //
    speedData = SPIA_Transmit_Receive((uint16_t)0x0000U);

    HWREGH(SPIA_BASE + SPI_O_BRR) = speedData;

    asm("   RPT #0x0F ||NOP");

    //
    // Read and discard the next 7 reserved words.
    //
    for(i = 1U; i <= 7U; i++)
    {
       (void)SPIA_GetWordData();
    }

    return;
}

//
// SPIA_GetWordData - This routine fetches two bytes from the SPI-A
//                    port and puts them together to form a single
//                    16-bit value.  It is assumed that the host is
//                    sending the data in the form MSB:LSB.
//
uint16_t SPIA_GetWordData(void)
{
    uint16_t wordData;

    //
    // Fetch the LSB
    //
    wordData = SPIA_Transmit_Receive(0x0000U);

    //
    // Fetch the MSB
    //
    wordData |= (SPIA_Transmit_Receive(0x0000U) << 8U);

    return(wordData);
}

//
// SPIBOOT_configure_gpio - Configure the GPIOs for the specified SPI boot mode
//
uint16_t SPIBOOT_configure_gpio(uint32_t bootMode)
{
    uint16_t gpioSIMOA;
    uint16_t gpioSOMIA;
    uint16_t gpioCLKA;    
    uint16_t gpioSTEA;      
    uint32_t gpioSIMOAPinConfig;
    uint32_t gpioSOMIAPinConfig;    
    uint32_t gpioCLKAPinConfig;     
 
    //
    // Unlock the GPIO configuration registers
    //
    GPIO_unlockPortConfig(GPIO_PORT_A,0xFFFFFFFFUL);

    switch (bootMode)
    {
        case SPI_MASTER_BOOT:
        default:
            //
            // GPIO2 = SPISIMOA
            // GPIO1 = SPISOMIA
            // GPIO3 = SPICLKA
            // GPIO5 = SPISTEA 
            //
            gpioSIMOA = 2;
            gpioSOMIA = 1;
            gpioCLKA  = 3;    
            gpioSTEA  = 5;      
            gpioSIMOAPinConfig = GPIO_2_SPISIMOA;
            gpioSOMIAPinConfig = GPIO_1_SPISOMIA;    
            gpioCLKAPinConfig  = GPIO_3_SPICLKA;             
            break;

        case SPI_MASTER_BOOT_ALT1:
            //
            // GPIO16 = SPISIMOA
            // GPIO1  = SPISOMIA
            // GPIO3  = SPICLKA
            // GPIO0  = SPISTEA
            //
            gpioSIMOA = 16;
            gpioSOMIA = 1;
            gpioCLKA  = 3;    
            gpioSTEA  = 0;      
            gpioSIMOAPinConfig = GPIO_16_SPISIMOA;
            gpioSOMIAPinConfig = GPIO_1_SPISOMIA;    
            gpioCLKAPinConfig  = GPIO_3_SPICLKA;
            break;

        case SPI_MASTER_BOOT_ALT2:
            //
            // GPIO8  = SPISIMOA
            // GPIO10 = SPISOMIA
            // GPIO9  = SPICLKA
            // GPIO11 = SPISTEA
            //
            gpioSIMOA = 8;
            gpioSOMIA = 10;
            gpioCLKA  = 9;    
            gpioSTEA  = 11;      
            gpioSIMOAPinConfig = GPIO_8_SPISIMOA;
            gpioSOMIAPinConfig = GPIO_10_SPISOMIA;    
            gpioCLKAPinConfig = GPIO_9_SPICLKA;
            break;

        case SPI_MASTER_BOOT_ALT3:
            //
            // GPIO8  = SPISIMOA
            // GPIO17 = SPISOMIA
            // GPIO9  = SPICLKA
            // GPIO11 = SPISTEA
            //
            gpioSIMOA = 8;
            gpioSOMIA = 17;
            gpioCLKA  = 9;    
            gpioSTEA  = 11;      
            gpioSIMOAPinConfig = GPIO_8_SPISIMOA;
            gpioSOMIAPinConfig = GPIO_17_SPISOMIA;    
            gpioCLKAPinConfig = GPIO_9_SPICLKA;
            break;
    }
    
    //
    // Enable pull up on GPIOs pins
    //
    GPIO_setPadConfig(gpioSIMOA,GPIO_PIN_TYPE_PULLUP);
    GPIO_setPadConfig(gpioSOMIA,GPIO_PIN_TYPE_PULLUP);
    GPIO_setPadConfig(gpioCLKA,GPIO_PIN_TYPE_PULLUP);
    GPIO_setPadConfig(gpioSTEA,GPIO_PIN_TYPE_PULLUP);

    //
    // Set PIN config for SPI
    //
    GPIO_setPinConfig(gpioSIMOAPinConfig);
    GPIO_setPinConfig(gpioSOMIAPinConfig);
    GPIO_setPinConfig(gpioCLKAPinConfig);

    //
    // Configure SPISTEA as GPIO output
    //
    GPIO_setDirectionMode(gpioSTEA,GPIO_DIR_MODE_OUT);

    //
    // Configure GPIOs pins as async pins
    //
    GPIO_setQualificationMode(gpioSIMOA,GPIO_QUAL_ASYNC);
    GPIO_setQualificationMode(gpioSOMIA,GPIO_QUAL_ASYNC);
    GPIO_setQualificationMode(gpioCLKA,GPIO_QUAL_ASYNC);

    //
    // Pull SPISTEA low to enable Chip Select
    //
    GPIO_writePin(gpioSTEA,0);    

    return(gpioSTEA);
}

//
// SPIA_Init - SPI-A initialization routine for communication with host
//
void SPIA_Init(void)
{
    //
    // Enable SPI-A clocks
    //
    EALLOW;

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SPIA);
    SysCtl_setLowSpeedClock(SYSCTL_LSPCLK_PRESCALE_4);

    //
    // Enable FIFO reset bit only (FIFO can resume transmit/receive operation)
    //
    HWREGH(SPIA_BASE + SPI_O_FFTX) = SPI_FFTX_SPIRST;

    //
    // Setup SPI configurations
    // Note - The sysclk passed in is divided by 4 due to the API requiring the 
    //        low speed clock value (by default /4 is the divider for LSPCLK)
    //
    SPI_disableModule(SPIA_BASE);
    SPI_setConfig(SPIA_BASE,(BOOTROM_SYSCLK/4U),SPI_PROT_POL0PHA1,
                  SPI_MODE_MASTER,SPI_BIT_RATE,SPI_CHARACTER_LENGTH);
    SPI_enableModule(SPIA_BASE);

    EDIS;
}

//
// End of File
//
