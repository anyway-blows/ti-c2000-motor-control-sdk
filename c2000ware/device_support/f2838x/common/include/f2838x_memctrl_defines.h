// TI File $Revision: /main/3 $
// Checkin $Date: November 10, 2009   14:05:03 $
//###########################################################################
//
// FILE:   mem_wrapper_defines.h
//
//
//###########################################################################
// $TI Release: F2838x Support Library v3.04.00.00 $
// $Release Date: Fri Feb 12 19:08:49 IST 2021 $
//###########################################################################

#define FPGA

//#define PASS      0xABCDABCD
//#define FAIL      0xDEADDEAD
#define TEST_DONE 0x13579BDF

#define M0_START_ADDR 0x00000
#define M1_START_ADDR 0x00400

#define D0_START_ADDR 0x0C000
#define D1_START_ADDR 0x0C800

#define LS0_START_ADDR 0x08000
#define LS1_START_ADDR 0x08800
#define LS2_START_ADDR 0x09000
#define LS3_START_ADDR 0x09800
#define LS4_START_ADDR 0x0A000
#define LS5_START_ADDR 0x0A800
#define LS6_START_ADDR 0x0B000
#define LS7_START_ADDR 0x0B800

#define GS0_START_ADDR  0x0D000
#define GS1_START_ADDR  0x0E000
#define GS2_START_ADDR  0x0F000
#define GS3_START_ADDR  0x10000
#define GS4_START_ADDR  0x11000
#define GS5_START_ADDR  0x12000
#define GS6_START_ADDR  0x13000
#define GS7_START_ADDR  0x14000
#define GS8_START_ADDR  0x15000
#define GS9_START_ADDR  0x16000
#define GS10_START_ADDR 0x17000
#define GS11_START_ADDR 0x18000
#define GS12_START_ADDR 0x19000
#define GS13_START_ADDR 0x1A000
#define GS14_START_ADDR 0x1B000
#define GS15_START_ADDR 0x1C000

#define CPU1TOCPU2MSGRAM_START_ADDR 0x3A000
#define CPU1TOCPU2MSGRAM_END_ADDR 0x3A7FF
#define CPU2TOCPU1MSGRAM_START_ADDR 0x3B000
#define CPU2TOCPU1MSGRAM_END_ADDR 0x3B7FF

#define CMTOCPU1MSGRAM_START_ADDR 0x38000
#define CMTOCPU1MSGRAM_END_ADDR 0x383FF
#define CPU1TOCMMSGRAM_START_ADDR 0x38400
#define CPU1TOCMMSGRAM_END_ADDR 0x387FF

#define CPUTOCLA1MSGRAM_START_ADDR 0x1500
#define CPUTOCLA1MSGRAM_END_ADDR 0x157F
#define CLA1TOCPUMSGRAM_START_ADDR 0x1480
#define CLA1TOCPUMSGRAM_END_ADDR 0x14FF


//===========================================================================
// End of file.
//===========================================================================
