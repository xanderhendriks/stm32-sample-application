//! @file crc.h
//! @brief CRC calculations for comms
#ifndef CRC_H
#define CRC_H

// Standard library includes
#include <stdio.h>

//! @brief Calculate 16 bit CRC for SDLC comms
//! @param [in] crc: CRC value returned by previous block (use 0 for first block)
//! @param [in] data: pointer to the data
//! @param [in] length: length of data to process
//! @return resulting CRC
uint16_t Crc_Calculate16Bit(uint16_t crc, const uint8_t *data, uint32_t length);
#endif
