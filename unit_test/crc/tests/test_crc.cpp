//! @file test_crc.cpp
//! @brief Test the CRC calculations
#include "CppUTest/TestHarness.h"

extern "C"
{
#include "crc.h"
}

TEST_GROUP(crc){};

TEST(crc, Crc_Calculate16Bit)
{
    uint32_t index;
    struct
    {
        uint16_t crc;
        uint8_t  data[32];
        uint32_t length;
        uint16_t result;
    } testValues[] = {{0, "test", 4, 39686},
                      {0, "this is another test string", 27, 31365},
                      {39686, "this is another test string", 27, 25393},
                      {0, "testthis is another test string", 31, 25393}};

    for (index = 0; index < sizeof(testValues) / sizeof(testValues[0]); index++)
    {
        char iterationText[64] = "";

        snprintf(iterationText, sizeof(iterationText), "index: %lu", index);
        LONGS_EQUAL_TEXT(testValues[index].result,
                         Crc_Calculate16Bit(testValues[index].crc, testValues[index].data, testValues[index].length),
                         iterationText);
    }
}
