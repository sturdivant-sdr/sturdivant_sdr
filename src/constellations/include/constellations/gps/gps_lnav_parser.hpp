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

#ifndef STURDIVANT_SDR_GPS_LNAV_PARSER_HPP
#define STURDIVANT_SDR_GPS_LNAV_PARSER_HPP

#include <spdlog/spdlog.h>

#include "constellations/gps/gps_constants.hpp"
#include "constellations/gps/gps_ephemeris.hpp"

namespace sturdivant_sdr {

class GpsLNAVParser : public GpsEphemeris {
  public:
    //* === GpsLNAVParser ===
    /// @brief constructor
    GpsLNAVParser();

    //* === ~GpsLNAVParser ===
    /// @brief destructor
    ~GpsLNAVParser();

    //* === parity_check ===
    bool parity_check(uint32_t& word);

    //* === parse_subframe ===
    bool parse_subframe();

    //* === load_preamble ===
    void load_preamble();

    //* === load_subframe_1 ===
    void load_subframe_1();

    //* === load_subframe_2 ===
    void load_subframe_2();

    //* === load_subframe_3 ===
    void load_subframe_3();

  protected:
    uint32_t subframe[10];  // most recent 10 words, data bits ordered [-2 -1 0 ... 30]

  private:
    std::shared_ptr<spdlog::logger> __log;
};
}  // namespace sturdivant_sdr

#endif