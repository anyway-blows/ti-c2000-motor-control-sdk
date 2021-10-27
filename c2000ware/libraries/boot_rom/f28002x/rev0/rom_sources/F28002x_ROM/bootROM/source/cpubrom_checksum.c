//###########################################################################
//
// FILE:    cpu1brom_checksum.c
//
// TITLE:   CPU1 boot ROM checksum calculation
//
// Calculate and verify the boot ROM checksum against the calculated checksum
// as calculated during development
//
//###########################################################################
// $TI Release: $
// $Release Date:  $
//###########################################################################

//
// Included Files
//
#include "cpu1bootrom.h"

//
// Globals
//

//
// Struct to configure memory blocks to be included in the checksum
//
typedef struct {
       unsigned long firstAddr;
       unsigned long lastAddr;
       unsigned short mem_attribute;
} memblock;

//
// Use 4 32-bit values to represent the 64-bit checksum.
// This is done such that overflow can be monitored at
// each step in the summation. If overflow from the lower
// 16-bits occurs, then the overflow is added to the next
// significant word.
//
typedef struct {
       unsigned long highHalfMSW;
       unsigned long highHalfLSW;
       unsigned long lowHalfMSW;
       unsigned long lowHalfLSW;
} ui64;

//
// Struct to store checksum calculation data
//
typedef struct checksum_data
{
    unsigned long total_ones;
    unsigned long bit_column_cksum[8][3];
    ui64 row_weight_sum;
    ui64 fact_weight_sum;
}composite_checksum_t;

//
// Boot ROM checksum calculation
//
#pragma DATA_SECTION(pbist_csum_rom, ".PBIST_CKSUMLOC");
const composite_checksum_t pbist_csum_rom = {
        0x0003568BUL,    //total_ones
        {{0x0000D557UL, 0x000015ACUL, 0x00000000UL},      //bit_column_cksum[0][3];
         {0x0000F05DUL, 0x00001CB5UL, 0x00000000UL},      //bit_column_cksum[1][3];
         {0x00005632UL, 0x00001AB9UL, 0x00000000UL},      //bit_column_cksum[2][3];
         {0x000057ADUL, 0x0000189AUL, 0x00000000UL},      //bit_column_cksum[3][3];
         {0x00004C9AUL, 0x00001485UL, 0x00000000UL},      //bit_column_cksum[4][3];
         {0x00007221UL, 0x000014FEUL, 0x00000000UL},      //bit_column_cksum[5][3];
         {0x00001562UL, 0x000014FDUL, 0x00000000UL},      //bit_column_cksum[6][3];
         {0x0000D3AFUL, 0x00001424UL, 0x00000000UL}},     //bit_column_cksum[7][3];
        {0x00000000UL, 0x00000005UL, 0x0000C2D1UL, 0x00005181UL}, //row_weight_sum,
        {0x00000000UL, 0x00001FBCUL, 0x000060C6UL, 0x00000F7EUL}  //fact_weight_sum
};

//
// Complete ROM checksum calculation
//
#pragma DATA_SECTION(csum_rom, ".CKSUMLOC");
const composite_checksum_t csum_rom = {
        0x0011FAB8UL,    //total_ones
        {{0x0000C1E5UL, 0x0000CD71UL, 0x00000001UL},	//bit_column_cksum[0][3];
         {0x0000BAB1UL, 0x0000E5BFUL, 0x00000001UL},	//bit_column_cksum[1][3];
         {0x0000763AUL, 0x0000DE0AUL, 0x00000001UL},	//bit_column_cksum[2][3];
         {0x000088C7UL, 0x0000D4F6UL, 0x00000001UL},	//bit_column_cksum[3][3];
         {0x0000C916UL, 0x0000C965UL, 0x00000001UL},	//bit_column_cksum[4][3];
         {0x0000CD86UL, 0x0000CCBAUL, 0x00000001UL},	//bit_column_cksum[5][3];
         {0x0000A98EUL, 0x0000CA36UL, 0x00000001UL},	//bit_column_cksum[6][3];
         {0x0000C607UL, 0x0000CDDAUL, 0x00000001UL}},   //bit_column_cksum[7][3];,
        {0x00000000UL, 0x00000074UL, 0x0000A2D2UL, 0x0000AABFUL}, //row_weight_sum
        {0x00000000UL, 0x00004B92UL, 0x0000C1A2UL, 0x0000F12EUL}  //fact_weight_sum
};

//
// Function Prototypes
//

#pragma CODE_SECTION(verify_pbist_checksum_onROM, ".CKSUMFUNCS");
unsigned short verify_pbist_checksum_onROM(void);

#pragma CODE_SECTION(calculate_composite_checksum_on_rom, ".CKSUMFUNCS");
unsigned short calculate_composite_checksum_on_rom(composite_checksum_t *cksum, memblock *mblock, unsigned int total_mblocks);

#pragma CODE_SECTION(add_to_factorial_sum, ".CKSUMFUNCS");
void add_to_factorial_sum(unsigned long fact_weight, composite_checksum_t *cksum);

#pragma CODE_SECTION(add_to_row_weight_sum, ".CKSUMFUNCS");
void add_to_row_weight_sum(unsigned long ones_weight, composite_checksum_t *cksum);

#pragma CODE_SECTION(add_to_bitcolumn_checksum, ".CKSUMFUNCS");
unsigned long add_to_bitcolumn_checksum(unsigned long bitcolumn, unsigned long bit_weight, composite_checksum_t *cksum);

#pragma CODE_SECTION(compare_two_composite_checksums, ".CKSUMFUNCS");
unsigned short compare_two_composite_checksums(const composite_checksum_t *goldsum, composite_checksum_t *compsum);

#pragma CODE_SECTION(calculate_checksum, ".CKSUMFUNCS");
unsigned long calculate_checksum(unsigned long start, unsigned long end, unsigned long offset, composite_checksum_t *cksum);

#pragma CODE_SECTION(test_reserved_memory, ".CKSUMFUNCS");
unsigned short test_reserved_memory(unsigned long start, unsigned long end);

//
// test_reserved_memory - Verify that reserved memory is all 1's, else
//                        return error.
//
unsigned short
test_reserved_memory(unsigned long start, unsigned long end)
{
    if((start & ~(0x3FFFFFU)) != 0U)
    {
        //
        // Address beyond 22 bits
        //
        while(start <= end)
        {
            if(__addr32_read_uint16(start) != (unsigned long)0xFFFFU)
            {
                return 0xFFFFU;    // return error
            }
            start+=1U;
        }
    }
    else
    {
        //
        // Address within 22 bits
        //
        while(start <= end)
        {
            if(* (volatile unsigned short *)start != (unsigned long)0xFFFFU)
            {
                return 0xFFFFU;    // return error
            }
            start+=1U;
        }
    }

    return 0U;
}

//
// calculate_composite_checksum_on_rom
//
// This main function performs a very simple 64 bit checksum routine.
// 4 32-bit values are used to calculate the checksum.  This is done so that
// no overflow is lost.  After each addition, the value is checked to see if
// it has overflowed the lower 16-bits.  If it has, then the overflow portion
// in the upper 16-bits is added to the next most significant portion of the
// checksum.
//
// When the routine completes, the upper 16-bits of each portion of the checksum
// will be 0x000 - this portion should be discarded as it was only used to check
// for overflow and the checksum is then formed from the lower 16-bits.
//
// For example:
//     lowHalfLSW  = 0x00001111
//     lowHalfMSW  = 0x00002222
//     highHalfLSW = 0x00003333
//     highHalfMSW = 0x00004444
//
//     would result in a 64-bit checksum of 0x44443333 22221111
//
unsigned short
calculate_composite_checksum_on_rom(composite_checksum_t *cksum,
                                    memblock *mblock,
                                    unsigned int total_mblocks)
{
    unsigned int i;
    unsigned long offset = 0U;
    memblock *lmblock = (void *)0U;
    unsigned short result = 0U;

    cksum->row_weight_sum.highHalfMSW = 0U;
    cksum->row_weight_sum.highHalfLSW = 0U;
    cksum->row_weight_sum.lowHalfLSW = 0U;
    cksum->row_weight_sum.lowHalfMSW = 0U;

    cksum->fact_weight_sum.highHalfMSW = 0U;
    cksum->fact_weight_sum.highHalfLSW = 0U;
    cksum->fact_weight_sum.lowHalfLSW = 0U;
    cksum->fact_weight_sum.lowHalfMSW = 0U;

    cksum->total_ones = 0U;

    cksum->bit_column_cksum[0][0] = 0U;
    cksum->bit_column_cksum[1][0] = 0U;
    cksum->bit_column_cksum[2][0] = 0U;
    cksum->bit_column_cksum[3][0] = 0U;        
    cksum->bit_column_cksum[4][0] = 0U;
    cksum->bit_column_cksum[5][0] = 0U;
    cksum->bit_column_cksum[6][0] = 0U;
    cksum->bit_column_cksum[7][0] = 0U;

    cksum->bit_column_cksum[0][1] = 0U;
    cksum->bit_column_cksum[1][1] = 0U;
    cksum->bit_column_cksum[2][1] = 0U;
    cksum->bit_column_cksum[3][1] = 0U;
    cksum->bit_column_cksum[4][1] = 0U;
    cksum->bit_column_cksum[5][1] = 0U;
    cksum->bit_column_cksum[6][1] = 0U;
    cksum->bit_column_cksum[7][1] = 0U;

    cksum->bit_column_cksum[0][2] = 0U;
    cksum->bit_column_cksum[1][2] = 0U;
    cksum->bit_column_cksum[2][2] = 0U;
    cksum->bit_column_cksum[3][2] = 0U;
    cksum->bit_column_cksum[4][2] = 0U;
    cksum->bit_column_cksum[5][2] = 0U;
    cksum->bit_column_cksum[6][2] = 0U;
    cksum->bit_column_cksum[7][2] = 0U;

    for(i = 0U; i < total_mblocks; i++)
    {
        lmblock = (memblock *)(&mblock[i]);

        if(lmblock->mem_attribute == 0U)
        {
            result = test_reserved_memory((unsigned long)(lmblock->firstAddr),
                                          (unsigned long)(lmblock->lastAddr));

            if(result == 0xFFFFU)
            {
                return 0xFFFFU;
            }
        }
        else if(lmblock->mem_attribute == 1U)
        {
           offset = calculate_checksum((unsigned long)(lmblock->firstAddr),
                                       (unsigned long)(lmblock->lastAddr),
                                       offset, cksum);
        }
        else
        {
            // Do nothing
        }
    }

    return 0U;
}

//
// add_to_factorial_sum - Perform a factorial summation on the checksum and
//                        handle any overflow
//
void
add_to_factorial_sum(unsigned long fact_weight, composite_checksum_t *cksum)
{
    //
    // Add in LSW
    //
    cksum->fact_weight_sum.lowHalfLSW += (fact_weight & (unsigned long)0x0000FFFFUL);

    //
    // Check for overflow
    //
    if(cksum->fact_weight_sum.lowHalfLSW >= (unsigned long)0x0000FFFFUL)
    {
       cksum->fact_weight_sum.lowHalfMSW += cksum->fact_weight_sum.lowHalfLSW >> 16;
       cksum->fact_weight_sum.lowHalfLSW &= (unsigned long)0x0000FFFFUL;
    }

    if(cksum->fact_weight_sum.lowHalfMSW >= (unsigned long)0x0000FFFFUL)
    {
       cksum->fact_weight_sum.highHalfLSW += cksum->fact_weight_sum.lowHalfMSW >> 16;
       cksum->fact_weight_sum.lowHalfMSW &= (unsigned long)0x0000FFFFUL;
    }

    if(cksum->fact_weight_sum.highHalfLSW >= (unsigned long)0x0000FFFFUL)
    {
       cksum->fact_weight_sum.highHalfMSW += cksum->fact_weight_sum.lowHalfLSW >> 16;
       cksum->fact_weight_sum.highHalfLSW &= (unsigned long)0x0000FFFFUL;
    }

    //
    // Add in MSW
    //
    cksum->fact_weight_sum.lowHalfMSW += (fact_weight >> 16);

    //
    // Check for overflow
    //
    if(cksum->fact_weight_sum.lowHalfMSW >= (unsigned long)0x0000FFFFUL)
    {
       cksum->fact_weight_sum.highHalfLSW += cksum->fact_weight_sum.lowHalfMSW >> 16;
       cksum->fact_weight_sum.lowHalfMSW &= (unsigned long)0x0000FFFFUL;
    }

    if(cksum->fact_weight_sum.highHalfLSW >= (unsigned long)0x0000FFFFUL)
    {
       cksum->fact_weight_sum.highHalfMSW += cksum->fact_weight_sum.lowHalfLSW >> 16;
       cksum->fact_weight_sum.highHalfLSW &= (unsigned long)0x0000FFFFUL;
    }
}

//
// add_to_row_weight_sum - Perform sum on checksum row weight and handle any
//                         overflow
//
void
add_to_row_weight_sum(unsigned long ones_weight, composite_checksum_t *cksum)
{
    //
    // Add in LSW
    //
    cksum->row_weight_sum.lowHalfLSW += (ones_weight & (unsigned long)0x0000FFFFUL);

    //
    // Check for overflow
    //
    if(cksum->row_weight_sum.lowHalfLSW >= (unsigned long)0x0000FFFFUL)
    {
       cksum->row_weight_sum.lowHalfMSW += cksum->row_weight_sum.lowHalfLSW >> 16;
       cksum->row_weight_sum.lowHalfLSW &= (unsigned long)0x0000FFFFUL;
    }

    if(cksum->row_weight_sum.lowHalfMSW >= (unsigned long)0x0000FFFFUL)
    {
       cksum->row_weight_sum.highHalfLSW += cksum->row_weight_sum.lowHalfMSW >> 16;
       cksum->row_weight_sum.lowHalfMSW &= (unsigned long)0x0000FFFFUL;
    }

    if(cksum->row_weight_sum.highHalfLSW >= (unsigned long)0x0000FFFFUL)
    {
       cksum->row_weight_sum.highHalfMSW += cksum->row_weight_sum.lowHalfLSW >> 16;
       cksum->row_weight_sum.highHalfLSW &= (unsigned long)0x0000FFFFUL;
    }

    //
    // Add in MSW
    //
    cksum->row_weight_sum.lowHalfMSW += (ones_weight >> 16);

    //
    // Check for overflow
    //
    if(cksum->row_weight_sum.lowHalfMSW >= (unsigned long)0x0000FFFFUL)
    {
       cksum->row_weight_sum.highHalfLSW += cksum->row_weight_sum.lowHalfMSW >> 16;
       cksum->row_weight_sum.lowHalfMSW &= (unsigned long)0x0000FFFFUL;
    }

    if(cksum->row_weight_sum.highHalfLSW >= (unsigned long)0x0000FFFFUL)
    {
       cksum->row_weight_sum.highHalfMSW += cksum->row_weight_sum.lowHalfLSW >> 16;
       cksum->row_weight_sum.highHalfLSW &= (unsigned long)0x0000FFFFUL;
    }
}

//
// add_to_bitcolumn_checksum - Perform a bit column sum. Returns ones weight of
//                             the bit just added.
//                             ones weight = 8*(bitweight-1)+bitcolumn.
//
unsigned long
add_to_bitcolumn_checksum(unsigned long bitcolumn, unsigned long bit_weight,
                          composite_checksum_t *cksum)
{
    //
    // Add in LSW
    //
    cksum->bit_column_cksum[bitcolumn][0] += (bit_weight & (unsigned long)0x0000FFFFUL);

    //
    // Check for overflow
    //
    if(cksum->bit_column_cksum[bitcolumn][0] >= (unsigned long)0x0000FFFFUL)
    {
       cksum->bit_column_cksum[bitcolumn][1] += cksum->bit_column_cksum[bitcolumn][0] >> 16;
       cksum->bit_column_cksum[bitcolumn][0] &= (unsigned long)0x0000FFFFUL;
    }

    if(cksum->bit_column_cksum[bitcolumn][1] >= (unsigned long)0x0000FFFFUL)
    {
       cksum->bit_column_cksum[bitcolumn][2] += cksum->bit_column_cksum[bitcolumn][1] >> 16;
       cksum->bit_column_cksum[bitcolumn][1] &= (unsigned long)0x0000FFFFUL;
    }

    //
    // Add in MSW
    //
    cksum->bit_column_cksum[bitcolumn][1] += (bit_weight >> 16);

    //
    // Check for overflow
    //
    if(cksum->bit_column_cksum[bitcolumn][1] >= (unsigned long)0x0000FFFFUL)
    {
       cksum->bit_column_cksum[bitcolumn][2] += cksum->bit_column_cksum[bitcolumn][1] >> 16;
       cksum->bit_column_cksum[bitcolumn][1] &= (unsigned long)0x0000FFFFUL;
    }

    return((unsigned long)(((unsigned long)8U*(bit_weight-1U)) + bitcolumn));
}

//
// calculate_checksum - Main checksum algorithm
//
unsigned long
calculate_checksum(unsigned long start, unsigned long end, unsigned long offset,
                   composite_checksum_t *cksum)
{
    unsigned short data = 0x0000U;
    unsigned short byte_value = 0U;
    unsigned long byte_count = 0U;
    unsigned long byte_offset = 0U;
    unsigned long ones_weight = 0U;
    unsigned long ones_fact_weight = 0U;
    unsigned long nibble_count = 0U;    
    unsigned int nibble_offset = 0U;

    while(start <= end)
    {
        if((start & ~(0x3FFFFFU)) != 0U)
        {
            //
            // Address beyond 22 bits
            //
            data = __addr32_read_uint16(start);
            start+=1U;
        }
        else
        {
            //
            // Address within 22 bits
            // (Start is pointer to a 16-bit value)
            //
            data = *(volatile unsigned short *)start;
            start+=1U;
        }

        //
        // Each offset has 2 bytes of data on C28x CPUs
        //
        for(byte_count = 1U; byte_count < 3U; byte_count++)
        {
            byte_offset = byte_count+(offset);
            ones_fact_weight = 1U;

            //
            // Get the byte from word
            //
            byte_value = (data & (0xFFU));
            data = (data >> 8);
            ones_weight = 0U;

            //
            // Separate the byte into two nibbles
            //
            for(nibble_offset = 0U; nibble_offset <= 4U; nibble_offset+=4U)
            {
                nibble_count+=1U;;

                if((byte_value & 0xFU) == 0U)
                {
                    ; // Do nothing
                }
                else if((byte_value & 0xFU) == 1U)
                {
                    cksum->total_ones += 1U;
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)0U + nibble_offset,
                                                             byte_offset, cksum);
                    ones_fact_weight = (nibble_count - 1U)*(nibble_count + 1U);
                }
                else if((byte_value & 0xFU) == 2U)
                {
                    cksum->total_ones += 1U;
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)1U + nibble_offset,
                                                             byte_offset, cksum);
                    ones_fact_weight = (nibble_count - 1U)*(nibble_count + 2U) ;
                }
                else if((byte_value & 0xFU) == 3U)
                {
                    cksum->total_ones += 2U;
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)0U + nibble_offset,
                                                             byte_offset, cksum);
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)1U + nibble_offset,
                                                             byte_offset, cksum);

                    ones_fact_weight = ones_fact_weight *(nibble_count + 1U);
                    ones_fact_weight = ones_fact_weight *((nibble_count%2U) + 2U);
                }
                else if((byte_value & 0xFU) == 4U)
                {
                    cksum->total_ones += 1U;
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)2U + nibble_offset,
                                                             byte_offset, cksum);

                    ones_fact_weight = (nibble_count - 1U)*(nibble_count + 3U);

                }
                else if((byte_value & 0xFU) == 5U)
                {
                    cksum->total_ones += 2U;
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)0U + nibble_offset,
                                                             byte_offset, cksum);
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)2U + nibble_offset,
                                                             byte_offset, cksum);

                    ones_fact_weight = ones_fact_weight *(nibble_count + 1U);
                    ones_fact_weight = ones_fact_weight *((nibble_count%2U) + 3U);
                }
                else if((byte_value & 0xFU) == 6U)
                {
                    cksum->total_ones += 2U;
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)1U + nibble_offset,
                                                             byte_offset, cksum);
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)2U + nibble_offset,
                                                             byte_offset, cksum);

                    ones_fact_weight = ones_fact_weight *(nibble_count + 2U);
                    ones_fact_weight = ones_fact_weight *((nibble_count%2U) + 3U);
                }
                else if((byte_value & 0xFU) == 7U)
                {
                    cksum->total_ones += 3U;
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)0U + nibble_offset,
                                                             byte_offset, cksum);
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)1U + nibble_offset,
                                                             byte_offset, cksum);
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)2U + nibble_offset,
                                                             byte_offset, cksum);

                    ones_fact_weight = ones_fact_weight * (nibble_count + 1U);
                    ones_fact_weight = ones_fact_weight * ((nibble_count%2U) + 2U);
                    ones_fact_weight = ones_fact_weight * ((nibble_count%2U) + 3U);
                }
                else if((byte_value & 0xFU) == 8U)
                {
                    cksum->total_ones += 1U;
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)3U + nibble_offset,
                                                             byte_offset, cksum);

                    ones_fact_weight = (nibble_count - 1U) * (nibble_count + 4U);
                }
                else if((byte_value & 0xFU) == 9U)
                {
                    cksum->total_ones += 2U;
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)0U + nibble_offset,
                                                             byte_offset, cksum);
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)3U + nibble_offset,
                                                             byte_offset, cksum);

                    ones_fact_weight = ones_fact_weight * (nibble_count + 1U);
                    ones_fact_weight = ones_fact_weight * ((nibble_count%2U) + 4U);
                }
                else if((byte_value & 0xFU) == 10U)
                {
                    cksum->total_ones += 2U;
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)1U + nibble_offset,
                                                             byte_offset, cksum);
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)3U + nibble_offset,
                                                             byte_offset, cksum);

                    ones_fact_weight = ones_fact_weight * (nibble_count + 2U);
                    ones_fact_weight = ones_fact_weight * ((nibble_count%2U) + 4U);
                }
                else if((byte_value & 0xFU) == 11U)
                {
                    cksum->total_ones += 3U;
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)0U + nibble_offset,
                                                             byte_offset, cksum);
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)1U + nibble_offset,
                                                             byte_offset, cksum);
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)3U + nibble_offset,
                                                             byte_offset, cksum);

                    ones_fact_weight = ones_fact_weight * (nibble_count + 1U);
                    ones_fact_weight = ones_fact_weight * ((nibble_count%2U) + 2U);
                    ones_fact_weight = ones_fact_weight * ((nibble_count%2U) + 4U);
                }
                else if((byte_value & 0xFU) == 12U)
                {
                    cksum->total_ones += 2U;
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)2U + nibble_offset,
                                                             byte_offset, cksum);
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)3U + nibble_offset,
                                                             byte_offset, cksum);

                    ones_fact_weight = ones_fact_weight * (nibble_count + 3U);
                    ones_fact_weight = ones_fact_weight * ((nibble_count%2U) + 4U);
                }
                else if((byte_value & 0xFU) == 13U)
                {
                    cksum->total_ones += 3U;
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)0U + nibble_offset,
                                                             byte_offset, cksum);
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)2U + nibble_offset,
                                                             byte_offset, cksum);
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)3U + nibble_offset,
                                                             byte_offset, cksum);

                    ones_fact_weight = ones_fact_weight * (nibble_count + 1U);
                    ones_fact_weight = ones_fact_weight * ((nibble_count%2U) + 3U);
                    ones_fact_weight = ones_fact_weight * ((nibble_count%2U) + 4U);
                }
                else if((byte_value & 0xFU) == 14U)
                {
                    cksum->total_ones += 3U;
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)1U + nibble_offset,
                                                             byte_offset, cksum);
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)2U + nibble_offset,
                                                             byte_offset, cksum);
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)3U + nibble_offset,
                                                             byte_offset, cksum);

                    ones_fact_weight = ones_fact_weight * (nibble_count + 2U);
                    ones_fact_weight = ones_fact_weight * ((nibble_count%2U) + 3U);
                    ones_fact_weight = ones_fact_weight * ((nibble_count%2U) + 4U);

                }
                else if((byte_value & 0xFU) == 15U)
                {
                    cksum->total_ones += 4U;
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)0U + nibble_offset,
                                                             byte_offset, cksum);
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)1U + nibble_offset,
                                                             byte_offset, cksum);
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)2U + nibble_offset,
                                                             byte_offset, cksum);
                    ones_weight += add_to_bitcolumn_checksum((unsigned int)3U + nibble_offset,
                                                             byte_offset, cksum);

                    ones_fact_weight = ones_fact_weight * (nibble_count + 1U);
                    ones_fact_weight = ones_fact_weight * ((nibble_count%2U) + 2U);
                    ones_fact_weight = ones_fact_weight * ((nibble_count%2U) + 3U);
                    ones_fact_weight = ones_fact_weight * ((nibble_count%2U) + 4U);

                }
                else
                {
                    // Do nothing
                }

                add_to_factorial_sum(ones_fact_weight, cksum);
                ones_fact_weight = 1U;

                //
                // Get the next nibble from byte
                //
                byte_value = byte_value >> 4U;
            }//for loop on nibble.

            add_to_row_weight_sum(ones_weight, cksum);
        } // for loop on bytes in word.

        offset++;
    }// start-end while loop

    return offset;
}

//
// compare_two_composite_checksums - Compare two checksums and return 0 if they
//                                   match
//
unsigned short
compare_two_composite_checksums(const composite_checksum_t *goldsum,
                                composite_checksum_t *compsum)
{

    if((goldsum == (void *)0) || (compsum == (void *)0))
    {
        return 0xFFFFU;
    }

    if(goldsum->total_ones != compsum->total_ones)
    {
        return 0xFFFEU;
    }

    if((goldsum->fact_weight_sum.highHalfMSW != compsum->fact_weight_sum.highHalfMSW) ||
       (goldsum->fact_weight_sum.highHalfLSW != compsum->fact_weight_sum.highHalfLSW) ||
       (goldsum->fact_weight_sum.lowHalfMSW != compsum->fact_weight_sum.lowHalfMSW) ||
       (goldsum->fact_weight_sum.lowHalfLSW != compsum->fact_weight_sum.lowHalfLSW))
    {
        return 0xFFFDU;
    }

    if((goldsum->row_weight_sum.highHalfMSW != compsum->row_weight_sum.highHalfMSW) ||
       (goldsum->row_weight_sum.highHalfLSW != compsum->row_weight_sum.highHalfLSW) ||
       (goldsum->row_weight_sum.lowHalfMSW != compsum->row_weight_sum.lowHalfMSW) ||
       (goldsum->row_weight_sum.lowHalfLSW != compsum->row_weight_sum.lowHalfLSW))
    {
        return 0xFFFCU;
    }

    if((goldsum->bit_column_cksum[7][2] != compsum->bit_column_cksum[7][2]) ||
       (goldsum->bit_column_cksum[7][1] != compsum->bit_column_cksum[7][1]) ||
       (goldsum->bit_column_cksum[7][0] != compsum->bit_column_cksum[7][0]))
    {
        return 0xFFFBU;
    }

    if((goldsum->bit_column_cksum[6][2] != compsum->bit_column_cksum[6][2]) ||
       (goldsum->bit_column_cksum[6][1] != compsum->bit_column_cksum[6][1]) ||
       (goldsum->bit_column_cksum[6][0] != compsum->bit_column_cksum[6][0]))
    {
        return 0xFFFAU;
    }

    if((goldsum->bit_column_cksum[5][2] != compsum->bit_column_cksum[5][2]) ||
       (goldsum->bit_column_cksum[5][1] != compsum->bit_column_cksum[5][1]) ||
       (goldsum->bit_column_cksum[5][0] != compsum->bit_column_cksum[5][0]))
    {
        return 0xFFF9U;
    }

    if((goldsum->bit_column_cksum[4][2] != compsum->bit_column_cksum[4][2]) ||
       (goldsum->bit_column_cksum[4][1] != compsum->bit_column_cksum[4][1]) ||
       (goldsum->bit_column_cksum[4][0] != compsum->bit_column_cksum[4][0]))
    {
        return 0xFFF8U;
    }

    if((goldsum->bit_column_cksum[3][2] != compsum->bit_column_cksum[3][2]) ||
       (goldsum->bit_column_cksum[3][1] != compsum->bit_column_cksum[3][1]) ||
       (goldsum->bit_column_cksum[3][0] != compsum->bit_column_cksum[3][0]))
    {
        return 0xFFF7U;
    }

    if((goldsum->bit_column_cksum[2][2] != compsum->bit_column_cksum[2][2]) ||
       (goldsum->bit_column_cksum[2][1] != compsum->bit_column_cksum[2][1]) ||
       (goldsum->bit_column_cksum[2][0] != compsum->bit_column_cksum[2][0]))
    {
        return 0xFFF6U;
    }

    if((goldsum->bit_column_cksum[1][2] != compsum->bit_column_cksum[1][2]) ||
       (goldsum->bit_column_cksum[1][1] != compsum->bit_column_cksum[1][1]) ||
       (goldsum->bit_column_cksum[1][0] != compsum->bit_column_cksum[1][0]))
    {
        return 0xFFF5U;
    }

    if((goldsum->bit_column_cksum[0][2] != compsum->bit_column_cksum[0][2]) ||
       (goldsum->bit_column_cksum[0][1] != compsum->bit_column_cksum[0][1]) ||
       (goldsum->bit_column_cksum[0][0] != compsum->bit_column_cksum[0][0]))
    {
        return 0xFFF4U;
    }

    return 0U;
}

//
// verify_pbist_checksum_onROM - Perform checksum on boot ROM on the specified
//                               memory blocks and return whether the calculated
//                               checksum matches the golden value in ROM
//
unsigned short
verify_pbist_checksum_onROM(void)
{
    composite_checksum_t rom_cksum;
	memblock mblock_rom[5];


	SysCtl_disableWatchdog();

	mblock_rom[0].firstAddr = 0x3F0000;
	mblock_rom[0].lastAddr =  0x3F62FF;
	mblock_rom[0].mem_attribute = 1;	//data is present

	mblock_rom[1].firstAddr = 0x3F6300;
	mblock_rom[1].lastAddr =  0X3F8051;
	mblock_rom[1].mem_attribute = 0;	//data is NOT present

	mblock_rom[2].firstAddr = 0x3F8052;
	mblock_rom[2].lastAddr =  0x3F82FF;
	mblock_rom[2].mem_attribute = 1;	//data is present
    
    //skipping :- 3F8300 to 3F83C1 (not used or (reserved))

	mblock_rom[3].firstAddr = 0x3F83C2;
	mblock_rom[3].lastAddr =  0x3F8EF3;
	mblock_rom[3].mem_attribute = 1;	//data is present
    
    //skipping :- 3F8EF4 to 3FFFBD (not used or (reserved))

	mblock_rom[4].firstAddr = 0x3FFFBE;
	mblock_rom[4].lastAddr =  0x3FFFFF;
	mblock_rom[4].mem_attribute = 1;	//data is present

    asm(" RPT #9 || NOP"); // Preserve symbols

	if ( 0xFFFF == calculate_composite_checksum_on_rom(&rom_cksum, &mblock_rom[0], 5))
		return 0xFFF2U;

	return compare_two_composite_checksums(&pbist_csum_rom, &rom_cksum);
}

//
// End of File
//
