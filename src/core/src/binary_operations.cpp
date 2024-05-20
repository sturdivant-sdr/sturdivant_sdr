/**
|===================================== binary_operations.cpp ======================================|
|                                                                                                  |
|   Property of Daniel Sturdivant. Unauthorized copying of this file via any medium is would be    |
|   super sad and unfortunate for me. Proprietary and confidential.                                |
|                                                                                                  |
|--------------------------------------------------------------------------------------------------|
|                                                                                                  |
|   @file     src/common/binary_operations.cpp                                                     |
|   @brief    Binary register operations.                                                          |
|   @author   Daniel Sturdivant <sturdivant20@gmail.com>                                           |
|   @date     May 2024                                                                             |
|                                                                                                  |
|==================================================================================================|
*/

#include "core/algorithms/binary_operations.hpp"

namespace sturdivant_sdr {

uint32_t rotl(uint32_t& in, uint8_t len) {
    return (in << len) ^ (in >> (32 - len));
}

bool get_bit(uint32_t& in, uint8_t idx) {
    return (in >> (31 - idx)) & 1;
}

uint32_t get_bits(uint32_t& in, uint8_t beg, uint8_t end) {
    return (in >> (31 - end)) & ((1 << (end - beg + 1)) - 1);
}

double twos_comp(uint32_t in, uint8_t len) {
    in &= ((1 << len) - 1);  // mask to only desired bits
    if ((in >> (len - 1) & 1)) {
        uint32_t sb = 1 << (len - 1);
        return -static_cast<double>(sb - (in & ~sb));
    } else {
        return static_cast<double>(in);
    }
}
}  // namespace sturdivant_sdr

// template <typename T>
// T get_bits(uint32_t& in, uint8_t beg, uint8_t end) {
//     return (in >> (31 - end)) & ((1 << (end - beg + 1)) - 1);
// }

// template <typename T>
// double twos_comp(T in, uint8_t len) {
//     in &= ((1 << len) - 1);  // mask to only desired bits
//     if ((in >> (len - 1) & 1)) {
//         uint32_t sb = 1 << (len - 1);
//         return -static_cast<double>(sb - (in & ~sb));
//     } else {
//         return static_cast<double>(in);
//     }
// }