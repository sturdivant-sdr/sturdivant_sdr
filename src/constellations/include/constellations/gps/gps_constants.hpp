/**
|======================================== gps_constants.hpp =======================================|
|                                                                                                  |
|   Property of Daniel Sturdivant. Unauthorized copying of this file via any medium is would be    |
|   super sad and unfortunate for me. Proprietary and confidential.                                |
|                                                                                                  |
|--------------------------------------------------------------------------------------------------|
|                                                                                                  |
|   @file     include/adapters/gps_constants.hpp                                                   |
|   @brief    Constants for GPS data structures.                                                   |
|   @author   Daniel Sturdivant <sturdivant20@gmail.com>                                           |
|   @date     April 2024                                                                           |
|                                                                                                  |
|==================================================================================================|
*/

#ifndef STURDIVANT_SDR_GPS_CONSTANTS_HPP
#define STURDIVANT_SDR_GPS_CONSTANTS_HPP

#include <cstdint>

namespace sturdivant_sdr {

//* === GPS LNAV Data Constants ===
constexpr uint32_t GPS_LNAV_DATA_RATE = 50;       //! NAV message bit rate [bits/s]
constexpr uint32_t GPS_LNAV_WORD_BITS = 30;       //! number of bits per word in the NAV message
constexpr uint32_t GPS_LNAV_SUBFRAME_BITS = 300;  //! Number of bits per subframe in the NAV message
constexpr uint32_t GPS_LNAV_SUBFRAME_MS = 6000;   //! subframe duration [ms]
constexpr double MAX_LNAV_INTEGRATION_PERIOD_S = 0.02;  //! maximum coherent tracking period [s]
constexpr int MAX_LNAV_INTEGRATION_PERIOD_MS = 20;      //! maximum coherent tracking period [s]

//* === GPS CNAV Data Constants ===

//* === Common Ephemeris Scaling Factors ===
constexpr double TWO_P4 = 16.0;                     //! 2^4
constexpr double TWO_N5 = 0.03125;                  //! 2^-29
constexpr double TWO_N19 = 1.907348632812500e-006;  //! 2^-19
constexpr double TWO_N29 = 1.862645149230957e-009;  //! 2^-29
constexpr double TWO_N31 = 4.656612873077393e-010;  //! 2^-31
constexpr double TWO_N33 = 1.164153218269348e-010;  //! 2^-33
constexpr double TWO_N43 = 1.136868377216160e-013;  //! 2^-43
constexpr double TWO_N55 = 2.775557561562891e-017;  //! 2^-55
constexpr double GPS_PI = 3.1415926535898;          //! PI defined by IS-GPS-200N
constexpr double TWO_GPS_PI = 2.0 * GPS_PI;         //! 2*pi
constexpr double HALF_GPS_PI = 0.5 * GPS_PI;        //! pi/2
constexpr double HALF_WEEK = 302400.0;              //! half GPS week [s]
constexpr double WEEK = 604800.0;                   //! GPS week [s]

//* === WGS84 Constants (from IS-GPS-200N) ===
// constexpr double LIGHT_SPEED = 2.99792458e8;               //! speed of light [m/s]
constexpr double WGS84_OMEGA_DOT_EARTH = 7.2921151467e-5;  //! rotation rate of the Earth [rad/s]
constexpr double WGS84_MU = 3.986005e14;                   //! Earth gravitational constant
constexpr double WGS84_RE = 6378137.0;                     //! WGS 84 Earth Equatorial Radius
constexpr double WGS84_J2 = 0.0010826262;                  //! Oblate Earth Gravity Coefficient
constexpr double WGS84_F = -4.442807633e-10;               //! [s/sqrt(m)]

//* === GPS L1 C/A Constants ===
constexpr double GPS_L1_FREQ = 1575.42e6;          //! GPS L1 frequency [Hz]
constexpr double GPS_L1CA_CHIP_FREQ = 1.023e6;     //! GPS L1CA chipping frequency [chips/s]
constexpr double GPS_L1CA_CODE_LEN = 1023.0;       //! GPS L1CA code length [chips]
constexpr double GPS_L1CA_CODE_PERIOD = 0.001;     //! GPS L1CA code period [s]
constexpr uint8_t GPS_L1CA_PREAMBLE = 0b10001011;  //! GPS L1CA preamble [1 0 0 0 1 0 1 1]
constexpr int8_t L1CA_TAPS[32][2] = {/*01 {2, 6} */ {1, 5},
                                     /*02 {3, 7} */ {2, 6},
                                     /*03 {4, 8} */ {3, 7},
                                     /*04 {5, 9} */ {4, 8},
                                     /*05 {1, 9} */ {0, 8},
                                     /*06 {2, 10}*/ {1, 9},
                                     /*07 {1, 8} */ {0, 7},
                                     /*08 {2, 9} */ {1, 8},
                                     /*09 {3, 10}*/ {2, 9},
                                     /*10 {2, 3} */ {1, 2},
                                     /*11 {3, 4} */ {2, 3},
                                     /*12 {5, 6} */ {4, 5},
                                     /*13 {6, 7} */ {5, 6},
                                     /*14 {7, 8} */ {6, 7},
                                     /*15 {8, 9} */ {7, 8},
                                     /*16 {9, 10}*/ {8, 9},
                                     /*17 {1, 4} */ {0, 3},
                                     /*18 {2, 5} */ {1, 4},
                                     /*19 {3, 6} */ {2, 5},
                                     /*20 {4, 7} */ {3, 6},
                                     /*21 {5, 8} */ {4, 7},
                                     /*22 {6, 9} */ {5, 8},
                                     /*23 {1, 3} */ {0, 2},
                                     /*24 {4, 6} */ {3, 5},
                                     /*25 {5, 7} */ {4, 6},
                                     /*26 {6, 8} */ {5, 7},
                                     /*27 {7, 9} */ {6, 8},
                                     /*28 {8, 10}*/ {7, 9},
                                     /*29 {1, 6} */ {0, 5},
                                     /*30 {2, 7} */ {1, 6},
                                     /*31 {3, 8} */ {2, 7},
                                     /*32 {4, 9} */ {3, 8}};

//* === GPS L2C Constants ===

//* === GPS L5 Constants ===

}  // namespace sturdivant_sdr

#endif