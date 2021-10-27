//Library of tformat functions
#include <stdint.h>
#include "driverlib.h"
#include "clb.h"

#define SPI_FIFO_WIDTH 16
#define CRC_BITS 5
#define MODE_6BITS 6
#define START_BIT 1
#define SINK_BITS 3
#define CF_BITS 5
#define DF_BITS 8
#define PACKET_BITS 10
#define IDLE_BITS 7
#define ABIT 1
#define START_2T 2
#define STOP_2T 2
#define STOP_1T 1
#define ERROR_1BIT 1
#define ERROR_2BIT 1
#define ERROR_2BITS 2
#define CIRCLE_5BITS 5
#define ADDITIONAL_DATA_30BITS 30
#define MRS_8BITS 8
#define DATA_16BITS 16
#define ADDRESS_8BITS 8
#define TESTVAL_8BITS 8
#define TESTVAL_40BITS 40
#define THIRD_TRANS_8BITS   8
#define CMD_SUPPLEMENT_24BITS   24


extern void tformat_configureSPILen(uint16_t);
//extern void tformat_configureCLBLen(uint16_t, uint16_t, uint16_t);
static inline void
tformat_configureCLBLen(uint16_t CLB4_C1_M1, uint16_t CLB4_C1_M2,
            uint16_t CLB4_R0)
{
    CLB_writeInterface(CLB4_BASE, CLB_ADDR_COUNTER_1_MATCH1, CLB4_C1_M1);
    CLB_writeInterface(CLB4_BASE, CLB_ADDR_COUNTER_1_MATCH2, CLB4_C1_M2);
    CLB_writeInterface(CLB4_BASE, CLB_ADDR_HLC_R0, CLB4_R0);
}

extern void tformat_initSPIFIFO(uint32_t devLSPCLKFreq);
extern void tformat_initCLBXBAR(void);
extern void tformat_resetCLB(void);
extern void tformat_initCLB4(void);


extern uint32_t tformat_getBits (uint16_t len, uint16_t ByteNo,
            uint16_t bitIndex);
extern void tformat_putBits (uint32_t data, uint16_t byteNo, uint16_t bitIndex);
extern uint16_t tformat_getCRC(uint16_t inputCRCaccum, uint16_t nBitsData,
            uint16_t nBitsPoly, uint16_t * msg, uint16_t *crcTable,
            uint16_t rxLen);

