//! @file crc.c

// Global configuration
#include "config.h"

// Interface for this file
#include "crc.h"

// Static table used for the table_driven implementation.
static const uint16_t Crc_Table[16] = {0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
                                       0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef};

uint16_t Crc_Calculate16Bit(uint16_t crc, const uint8_t *data, uint32_t length)
{
    uint16_t i;
    uint16_t shiftedData;
    uint32_t lenIterator;
    uint16_t tempCrc = crc;
    uint32_t tempCrc32;

    for (lenIterator = length; lenIterator > 0U; lenIterator--)
    {
        shiftedData = ((uint16_t)(*data) >> 4);
        i           = (tempCrc >> 12) ^ shiftedData;
        tempCrc32   = (uint32_t) tempCrc << 4;
        tempCrc     = Crc_Table[i & 0x0FU] ^ (uint16_t) tempCrc32;
        shiftedData = (uint16_t)(*data);
        i           = (tempCrc >> 12) ^ shiftedData;
        tempCrc32   = (uint32_t) tempCrc << 4;
        tempCrc     = Crc_Table[i & 0x0FU] ^ (uint16_t) tempCrc32;
        data++;
    }

    return tempCrc & 0xFFFFU;
}
