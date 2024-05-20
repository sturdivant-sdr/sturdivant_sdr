/**
|====================================== gps_l1ca_channel.hpp ======================================|
|                                                                                                  |
|   Property of Daniel Sturdivant. Unauthorized copying of this file via any medium is would be    |
|   super sad and unfortunate for me. Proprietary and confidential.                                |
|                                                                                                  |
|--------------------------------------------------------------------------------------------------|
|                                                                                                  |
|   @file     include/adapters/gps_l1ca_channel.cpp                                                |
|   @brief    GPS L1CA code generation and signal replica tools.                                   |
|   @author   Daniel Sturdivant <sturdivant20@gmail.com>                                           |
|   @date     January 2024                                                                         |
|                                                                                                  |
|==================================================================================================|
*/

// TODO: run acquisition
// TODO: run tracking (look into polymorphism to select between LF and KF)
// TODO: run data bit histogram
// TODO: run CN0 estimator
// TODO: run carrier lock detector
// TODO: place data bits into gps words / subframe
// TODO: parse data bits into ephemeris parameters
// TODO: allow satellite parameter retrieval
// TODO: allow pseudorange correction

#ifndef STURDIVANT_SDR_GPS_L1CA_CHANNEL_HPP
#define STURDIVANT_SDR_GPS_L1CA_CHANNEL_HPP

#include <spdlog/spdlog.h>

#include <complex>
#include <condition_variable>
#include <fstream>
#include <shared_mutex>
#include <thread>
#include <vector>

#include "constellations/gps/gps_constants.hpp"
#include "constellations/gps/gps_ephemeris.hpp"
#include "constellations/gps/gps_lnav_parser.hpp"
#include "core/algorithms/lock_detectors.hpp"
#include "core/blocks/pcps_acquire.hpp"
#include "core/blocks/tracking.hpp"
#include "core/param/structures.hpp"
// #include "file_operations/signal_file.hpp"

// constexpr void GpsL1caChannel::generate_l1ca_code(int8_t ca_code[1023], const int& prn_id) {
//     int8_t tap_idx = prn_id - 1;  // zero-indexing
//     int8_t s1 = L1CA_TAPS[tap_idx][0];
//     int8_t s2 = L1CA_TAPS[tap_idx][1];
//     int8_t G1[10] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};  // initialize 10 bit number to all 1's
//     int8_t G2[10] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
//     int8_t f1 = 0;
//     int8_t f2 = 0;

//     for (int i = 0; i < 1023; i++) {
//         ca_code[i] = 2 * ((((G2[s1] + G2[s2]) % 2) + G1[9]) % 2) - 1;  // +/- 1
//         f1 = (G1[9] + G1[2]) % 2;
//         f2 = (G2[9] + G2[8] + G2[7] + G2[5] + G2[2] + G2[1]) % 2;
//         for (int j = 9; j > 0; j--) {
//             G1[j] = G1[j - 1];
//             G2[j] = G2[j - 1];
//         }
//         G1[0] = f1;
//         G2[0] = f2;
//     }
// };

namespace sturdivant_sdr {

class GpsL1caChannel : public GpsLNAVParser {
  public:
    //* === GpsL1caChannel
    /// @brief constructor
    GpsL1caChannel();
    GpsL1caChannel(const SdrConf &config, const uint8_t code_id, const uint32_t ch_id);

    //* === ~GpsL1caChannel ===
    /// @brief destructor
    ~GpsL1caChannel();

    //* === init ===
    void init(const SdrConf &conf);
    void init(const SdrConf &config, const uint8_t code_id, const uint32_t ch_id);

    //* === temp file reader ===
    // void set_data_stream_callback(std::function<std::vector<std::complex<double>>(int)>
    // function); std::function<std::vector<std::complex<double>>(int)> data_stream_input;
    void set_input_data_vector(std::vector<std::complex<double>> *stream);
    void set_mutex_and_conditions(std::atomic<int> *cond, bool *rep, std::shared_mutex *mtx,
                                  std::condition_variable_any *cond_var);
    // void channel_thread();
    std::vector<std::complex<double>> *data_stream;
    int data_stream_size;
    std::atomic<int> *condition;
    bool *repeat;
    std::shared_mutex *m;
    std::condition_variable_any *cv;

    //* === run ===
    void run();

    //* === try_acquisition ===
    // bool try_acquisition(std::vector<std::complex<double>> &acq_stream);
    bool try_acquisition();

  private:
    //* variables
    std::shared_ptr<spdlog::logger> __log;
    uint8_t prn_id;
    uint32_t channel_id;
    int8_t local_code[1023];
    SdrConf conf;
    std::unique_ptr<PcpsAcquire> acq;
    std::unique_ptr<Tracking> trk;
    // PcpsAcquire *acq;
    // Tracking *trk;
    std::ofstream fid;

    //* acquisition/tracking variables
    int predicted_samples;
    int samp_per_ms;
    int current_integration_ms;
    int ms_remaining;
    double current_integration_s;
    uint8_t bit_detector_histogram[20];
    uint8_t bit_flip_idx;
    Correlators corr;
    TrackingState state;

    //* observables variables
    bool cn0_estimator;  // 0 if moments method, 1 if beaulieu
    int cn0_estimator_len;
    double TOW;
    double CN0;
    double CN0_dB;
    double PHASE_DETECTOR;
    uint16_t lock_fail_count;
    uint16_t max_lock_fail_count;
    std::vector<double> IP0;
    std::vector<double> IP1;  // in-phase prompt of current set of correlators
    std::vector<double> QP1;  // quadrature prompt of current set of correlators
    uint32_t current_ms;
    uint16_t cn0_idx;

    // double (*cn0_estimator)(std::vector<double> &, std::vector<double> &, double &);
    // std::function<double(std::vector<double> &, std::vector<double> &, double &)> cn0_estimator;

    //* === tracking_callback ===
    /// @brief function that runs after a complete tracking integration period
    void tracking_callback();

    //* === lock_detectors ===
    /// @brief maintains lock detector status and tracking mode
    bool lock_detectors();

    //* === data_bit_detector ===
    void data_bit_detectors();

    //* === generate_ca_code ===
    void generate_l1ca_code();

};  // end class GpsL1caChannel
}  // namespace sturdivant_sdr

#endif