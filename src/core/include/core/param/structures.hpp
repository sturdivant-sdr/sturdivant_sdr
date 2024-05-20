/**
|========================================== configs.hpp ===========================================|
|                                                                                                  |
|   Property of Daniel Sturdivant. Unauthorized copying of this file via any medium is would be    |
|   super sad and unfortunate for me. Proprietary and confidential.                                |
|                                                                                                  |
|--------------------------------------------------------------------------------------------------|
|                                                                                                  |
|   @file     src/core/param/configs.hpp                                                           |
|   @brief    Contains various SDR configs and methods to read yaml file into the structure.       |
|   @author   Daniel Sturdivant <sturdivant20@gmail.com>                                           |
|   @date     May 2024                                                                             |
|                                                                                                  |
|==================================================================================================|
*/

#ifndef STURDIVANT_SDR_CONFIGS_HPP
#define STURDIVANT_SDR_CONFIGS_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace sturdivant_sdr {

//* ##### FILE CONFIG #####
struct FileConf {
    std::vector<std::string> active_signals{"GPS L1CA"};
    std::string sample_type{"real_int8"};
    std::string filename{"input_file.bin"};
    uint16_t ms_to_skip{0u};
    uint32_t ms_to_process{10000u};
};

//* ##### SIGNAL SOURCE CONFIG #####
struct SignalConf {
    double sampling_freq{0.0};
    double mix_freq{0.0};
};

//* ##### ACQUISITION CONFIG #####
struct AcquisitionConf {
    bool do_fine_search{false};
    double doppler_max_hz{5000.0};
    double doppler_step_hz{500.0};
    double threshold{20.0};
    uint8_t coherent_integration_time_ms{1u};
    bool dump{false};
    std::string dump_filename{"acq_dump_ch_"};
};

//* ##### TRACKING CONFIG #####
struct TrackingConf {
    bool do_kalman_filter{false};
    double fll_bw_hz{2.0};
    double pll_bw_hz{18.0};
    double dll_bw_hz{1.0};
    double correlator_spacing_chips{0.5};
    uint16_t min_pull_in_time_ms{150u};
    bool dump{false};
    std::string dump_filename{"trk_dump_ch_"};
};

//* ##### OBSERVABLES CONFIG #####
struct ObservablesConf {
    bool calc_range_rate{true};
    std::string cn0_estimator{"beaulieu"};
    // std::string cn0_estimator{"moments"};
    uint16_t lock_detector_integration_time_ms{100u};
    bool dump{false};
    std::string dump_filename{"observables"};
};

//* ##### PVT CONFIG #####
struct PVTConf {
    bool do_vector_track{false};
    uint16_t update_rate_ms{100u};
    uint16_t display_rate_ms{1000u};
    std::string iono_model{"klobuchar"};
    std::string trop_model{"saastamoinen"};
    bool dump{false};
    std::string dump_filename{"pvt_solution"};
};

struct SdrConf {
    FileConf fileconf;
    SignalConf sigconf;
    AcquisitionConf acqconf;
    TrackingConf trkconf;
    ObservablesConf obsconf;
    PVTConf pvtconf;
};

struct Correlators {
    double IP{0.0};
    double IE{0.0};
    double IL{0.0};
    double QP{0.0};
    double QE{0.0};
    double QL{0.0};
    double ip1{0.0};
    double ip2{0.0};
    double qp1{0.0};
    double qp2{0.0};
};

struct TrackingState {
    double carrier_phase{0.0};    // carrier phase at beginning of sample
    double carrier_doppler{0.0};  // carrier doppler at beginning of sample
    double carrier_rate{0.0};     // carrier doppler rate ate beginning of sample
    double code_phase{0.0};       // code phase at beginning of sample
    double code_doppler{0.0};     // code doppler at beginning of sample
};

}  // namespace sturdivant_sdr

#endif