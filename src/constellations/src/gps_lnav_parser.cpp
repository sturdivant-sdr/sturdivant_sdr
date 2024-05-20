/**
|====================================== gps_lnav_parser.hpp =======================================|
|                                                                                                  |
|   Property of Daniel Sturdivant. Unauthorized copying of this file via any medium is would be    |
|   super sad and unfortunate for me. Proprietary and confidential.                                |
|                                                                                                  |
|--------------------------------------------------------------------------------------------------|
|                                                                                                  |
|   @file     include/adapters/gps_lnav_parser.cpp                                                 |
|   @brief    GPS LNAV binary decoder.                                                             |
|   @author   Daniel Sturdivant <sturdivant20@gmail.com>                                           |
|   @date     January 2024                                                                         |
|                                                                                                  |
|==================================================================================================|
*/

#include "constellations/gps/gps_lnav_parser.hpp"

#include "core/algorithms/binary_operations.hpp"

namespace sturdivant_sdr {

//* === GpsLNAVParser ===
GpsLNAVParser::GpsLNAVParser() : GpsEphemeris() {
}

//* === ~GpsLNAVParser ===
GpsLNAVParser::~GpsLNAVParser() {
}

//* === parity_check ===
bool GpsLNAVParser::parity_check(uint32_t& gpsword) {
    // data bits ordered [-2 -1 0 ... 30]
    if (gpsword & 0x40000000u) {
        gpsword ^= 0x3FFFFFC0u;  // invert data bits according to D30star
    }

    // XOR as many bits in parallel as possible, IS-GPS-200N
    uint32_t d1 = gpsword & 0xFBFFBF00u;
    uint32_t d2 = rotl(gpsword, 1u) & 0x07FFBF01u;
    uint32_t d3 = rotl(gpsword, 2u) & 0xFC0F8100u;
    uint32_t d4 = rotl(gpsword, 3u) & 0xF81FFE02u;
    uint32_t d5 = rotl(gpsword, 4u) & 0xFC00000Eu;
    uint32_t d6 = rotl(gpsword, 5u) & 0x07F00001u;
    uint32_t d7 = rotl(gpsword, 6u) & 0x00003000u;
    uint32_t t = d1 ^ d2 ^ d3 ^ d4 ^ d5 ^ d6 ^ d7;

    // XOR the 5 6-bit fields together to produce the 6-bit parity
    uint32_t parity = t ^ rotl(t, 6u) ^ rotl(t, 12u) ^ rotl(t, 18u) ^ rotl(t, 24u);
    parity = parity & 0x0000003Fu;
    if (parity == (gpsword & 0x0000003Fu)) {
        return true;
    }
    return false;
};

bool GpsLNAVParser::parse_subframe() {
    for (uint32_t& gpsword : subframe) {
        if (!parity_check(gpsword)) {
            __log->warn("GpsLNAVParser::parity_check FAILED.");
            return false;
        }
    }
    uint8_t subframe_id = static_cast<uint8_t>(get_bits(subframe[1], 21, 23));
    switch (subframe_id) {
        case 1:
            load_subframe_1();
            break;
        case 2:
            load_subframe_2();
            break;
        case 3:
            load_subframe_3();
            break;
        case 4:
            break;
        case 5:
            break;
        default:
            __log->warn("GpsLNAVParser::parse_subframe -> Invalid subframe ID, received {}.",
                        static_cast<int>(subframe_id));
            return false;
    }
    return true;
}

//* === load_preamble ===
void GpsLNAVParser::load_preamble() {
    // word 1
    // sts.tlm_message = static_cast<uint16_t>(get_bits(subframe[0], 10, 23));  // bits 9-22 of word
    // 1 sts.integrity_status_flag = get_bit(subframe[0], 24);                    // bit 23 or word
    // 1
    sts.tlm_message = (subframe[0] & 0x003FFE00u) >> 8u;
    sts.integrity_status_flag = static_cast<bool>((subframe[0] & 0x00000080u) >> 7u);

    // word 2
    // sts.TOW = static_cast<double>(get_bits(subframe[1], 2, 18)) * 6.0;
    // sts.alert_flag = get_bit(subframe[1], 19);       // bit 18 of word 2
    // sts.anti_spoof_flag = get_bit(subframe[1], 20);  // bit 19 of word 2
    sts.TOW = 6.0 * static_cast<double>((subframe[1] & 0x3FFFE000u) >> 13u);
    sts.alert_flag = static_cast<bool>((subframe[1] & 0x00001000u) >> 12u);
    sts.anti_spoof_flag = static_cast<bool>((subframe[1] & 0x00000800u) >> 11u);
}

//* === load_subframe_1 ===
void GpsLNAVParser::load_subframe_1() {
    uint32_t tmp1 = 0;
    uint32_t tmp2 = 0;

    // word 1-2
    load_preamble();

    // word 3
    // sts.gps_week = static_cast<uint16_t>(get_bits(subframe[2], 2, 11));  // bits 1-10 of word 3
    // sts.l2_flag = static_cast<uint8_t>(get_bits(subframe[2], 12, 13));   // bits 11-12 of word 3
    // sts.URA = static_cast<uint8_t>(get_bits(subframe[2], 14, 17));       // bits 13-16 of word 3
    // sts.health = static_cast<uint8_t>(get_bits(subframe[2], 18, 23));    // bits 17-22 of word 3
    sts.gps_week = static_cast<uint16_t>((subframe[2] & 0x3FF00000u) >> 20u);
    sts.l2_flag = static_cast<uint8_t>((subframe[2] & 0x000C0000u) >> 18u);
    sts.URA = static_cast<uint8_t>((subframe[2] & 0x0003C000u) >> 14u);
    sts.health = static_cast<uint8_t>((subframe[2] & 0x00003F00u) >> 8u);

    // word 7
    // clk.T_GD = twos_comp(get_bits(subframe[6], 18, 25), 8) * TWO_N31;
    clk.T_GD = twos_comp((subframe[6] & 0x00003FC0u) >> 6u, 8u) * TWO_N31;

    // word 8
    // tmp1 = get_bits(subframe[2], 24, 25);  // bits 13-24 or word 3
    // tmp2 = get_bits(subframe[7], 2, 9);    // bits 1-8 or word 8
    // clk.IODC = static_cast<double>((tmp1 << 8) | tmp2);
    // clk.t_oc = static_cast<double>(get_bits(subframe[7], 10, 25)) * TWO_P4;
    tmp1 = (subframe[2] & 0x000000C0u) >> 6u;
    tmp2 = (subframe[7] & 0x3FC00000u) >> 22u;
    clk.IODC = static_cast<double>((tmp1 << 8u) | tmp2);
    clk.t_oc = static_cast<double>((subframe[7] & 0x003FFFC0) >> 6u) * TWO_P4;

    // word 9
    // clk.a_f2 = twos_comp(get_bits(subframe[8], 2, 9), 8) * TWO_N55;
    // clk.a_f1 = twos_comp(get_bits(subframe[8], 10, 25), 16) * TWO_N43;
    clk.a_f2 = twos_comp((subframe[8] & 0x000000C0u) >> 22u, 8u) * TWO_N55;
    clk.a_f1 = twos_comp((subframe[8] & 0x003FFFC0u) >> 6u, 16u) * TWO_P4;

    // word 10
    // clk.a_f0 = twos_comp(get_bits(subframe[9], 2, 23), 22) * TWO_N31;
    clk.a_f0 = twos_comp((subframe[9] & 0x3FFFFF00u) >> 8u, 22u) * TWO_N31;
}

//* === load_subframe_2 ===
void GpsLNAVParser::load_subframe_2() {
    uint32_t tmp1 = 0;
    uint32_t tmp2 = 0;

    // word 1-2
    load_preamble();

    // word 3
    // eph.IODE = static_cast<double>(get_bits(subframe[2], 2, 9));
    // eph.C_rs = twos_comp(get_bits(subframe[2], 10, 25), 16) * TWO_N5;
    eph.IODE = static_cast<double>((subframe[2] & 0x000000C0u) >> 22u);
    eph.C_rs = twos_comp((subframe[2] & 0x003FFFC0u) >> 6u, 16u) * TWO_N5;

    // word 4 and 5
    // eph.delta_n = twos_comp(get_bits(subframe[3], 2, 17), 16) * TWO_N43 * GPS_PI;
    // tmp1 = get_bits(subframe[3], 18, 25);
    // tmp2 = get_bits(subframe[4], 2, 25);
    // eph.M_0 = twos_comp((tmp1 << 24) | tmp2, 32) * TWO_N31 * GPS_PI;
    eph.delta_n = twos_comp((subframe[3] & 0x3FFFC000u) >> 14u, 16u) * TWO_N43 * GPS_PI;
    tmp1 = (subframe[3] & 0x00003FC0u) >> 6u;
    tmp2 = (subframe[3] & 0x3FFFFFC0u) >> 6u;
    eph.M_0 = twos_comp((tmp1 << 24u) | tmp2, 32u) * TWO_N31 * GPS_PI;

    // word 6 and 7
    // eph.C_uc = twos_comp(get_bits(subframe[5], 2, 17), 16) * TWO_N29;
    // tmp1 = get_bits(subframe[5], 18, 25);
    // tmp2 = get_bits(subframe[6], 2, 25);
    // eph.e = static_cast<double>((tmp1 << 24) | tmp2) * TWO_N33;
    eph.C_uc = twos_comp((subframe[5] & 0x3FFFC000u) >> 14u, 16u) * TWO_N29;
    tmp1 = (subframe[5] & 0x00003FC0u) >> 6u;
    tmp2 = (subframe[6] & 0x3FFFFFC0u) >> 6u;
    eph.e = static_cast<double>((tmp1 << 24u) | tmp2) * TWO_N33;

    // word 8 and 9
    // eph.C_us = twos_comp(get_bits(subframe[7], 2, 17), 16) * TWO_N29;
    // tmp1 = get_bits(subframe[7], 18, 25);
    // tmp2 = get_bits(subframe[8], 2, 25);
    // eph.sqrt_A = static_cast<double>((tmp1 << 24) | tmp2) * TWO_N19;
    eph.C_us = twos_comp((subframe[7] & 0x3FFFC000u) >> 14u, 16u) * TWO_N29;
    tmp1 = (subframe[7] & 0x00003FC0u) >> 6u;
    tmp2 = (subframe[8] & 0x3FFFFFC0u) >> 6u;
    eph.sqrt_A = static_cast<double>((tmp1 << 24u) | tmp2) * TWO_N19;

    // word 10
    // eph.t_oe = static_cast<double>(get_bits(subframe[9], 2, 17)) * TWO_P4;
    // sts.fit_interval_alert_flag = get_bit(subframe[9], 18);
    // sts.AODO = get_bits(subframe[9], 19, 23);
    eph.t_oe = static_cast<double>((subframe[9] & 0x3FFFC000u) >> 14u) * TWO_P4;
    sts.fit_interval_alert_flag = static_cast<bool>((subframe[9] & 0x00002000u) >> 13u);
}

//* === load_subframe_3 ===
void GpsLNAVParser::load_subframe_3() {
    uint32_t tmp1 = 0;
    uint32_t tmp2 = 0;

    // word 1-2
    load_preamble();

    // word 3 and 4
    // eph.C_ic = twos_comp(get_bits(subframe[2], 2, 17), 16) * TWO_N29;
    // tmp1 = get_bits(subframe[2], 18, 25);
    // tmp2 = get_bits(subframe[3], 2, 25);
    // eph.OMEGA_0 = twos_comp((tmp1 << 24) | tmp2, 32) * TWO_N31 * GPS_PI;
    eph.C_ic = twos_comp((subframe[2] & 0x3FFFC000u) >> 14u, 16u) * TWO_N29;
    tmp1 = (subframe[2] & 0x00003FC0u) >> 6u;
    tmp2 = (subframe[3] & 0x3FFFFFC0u) >> 6u;
    eph.OMEGA_0 = twos_comp((tmp1 << 24u) | tmp2, 32u) * TWO_N31 * GPS_PI;

    // word 5 and 6
    // eph.C_is = twos_comp(get_bits(subframe[4], 2, 17), 16) * TWO_N29;
    // tmp1 = get_bits(subframe[4], 18, 25);
    // tmp2 = get_bits(subframe[4], 2, 25);
    // eph.i_0 = twos_comp((tmp1 << 24) | tmp2, 32) * TWO_N31 * GPS_PI;
    eph.C_is = twos_comp((subframe[4] & 0x3FFFC000u) >> 14u, 16u) * TWO_N29;
    tmp1 = (subframe[4] & 0x00003FC0u) >> 6u;
    tmp2 = (subframe[5] & 0x3FFFFFC0u) >> 6u;
    eph.i_0 = twos_comp((tmp1 << 24u) | tmp2, 32u) * TWO_N31 * GPS_PI;

    // word 7 and 8
    // eph.C_rc = twos_comp(get_bits(subframe[6], 2, 17), 16) * TWO_N5;
    // tmp1 = get_bits(subframe[6], 18, 25);
    // tmp2 = get_bits(subframe[7], 2, 25);
    // eph.omega = twos_comp((tmp1 << 24) | tmp2, 32) * TWO_N31 * GPS_PI;
    eph.C_rc = twos_comp((subframe[6] & 0x3FFFC000u) >> 14u, 16u) * TWO_N5;
    tmp1 = (subframe[6] & 0x00003FC0u) >> 6u;
    tmp2 = (subframe[7] & 0x3FFFFFC0u) >> 6u;
    eph.omega = twos_comp((tmp1 << 24u) | tmp2, 32u) * TWO_N31 * GPS_PI;

    // word 9
    // eph.OMEGA_DOT = twos_comp(get_bits(subframe[8], 2, 25), 24) * TWO_N43 * GPS_PI;
    eph.OMEGA_DOT = twos_comp((subframe[8] & 0x3FFFFFC0u) >> 6u, 24u) * TWO_N43 * GPS_PI;

    // word 10
    // eph.IODE = static_cast<double>(get_bits(subframe[9], 2, 9));
    // eph.IDOT = twos_comp(get_bits(subframe[9], 10, 23), 14) * TWO_N43 * GPS_PI;
    eph.IODE = twos_comp((subframe[9] & 0x000000C0u) >> 22u, 8u);
    eph.IDOT = twos_comp((subframe[8] & 0x003FFF00u) >> 8u, 14u) * TWO_N43 * GPS_PI;
}

}  // namespace sturdivant_sdr