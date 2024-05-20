/**
|===================================== binary_operations.hpp ======================================|
|                                                                                                  |
|   Property of Daniel Sturdivant. Unauthorized copying of this file via any medium is would be    |
|   super sad and unfortunate for me. Proprietary and confidential.                                |
|                                                                                                  |
|--------------------------------------------------------------------------------------------------|
|                                                                                                  |
|   @file     include/common/binary_operations.hpp                                                 |
|   @brief    Binary register operations.                                                          |
|   @author   Daniel Sturdivant <sturdivant20@gmail.com>                                           |
|   @date     May 2024                                                                             |
|                                                                                                  |
|==================================================================================================|
*/

#ifndef STURDIVANT_SDR_BINARY_OPERATIONS_HPP
#define STURDIVANT_SDR_BINARY_OPERATIONS_HPP

#include <cstdint>

namespace sturdivant_sdr {

uint32_t rotl(uint32_t& in, uint8_t len);

bool get_bit(uint32_t& in, uint8_t idx);

uint32_t get_bits(uint32_t& in, uint8_t beg, uint8_t end);

double twos_comp(uint32_t in, uint8_t len);

}  // namespace sturdivant_sdr

#endif