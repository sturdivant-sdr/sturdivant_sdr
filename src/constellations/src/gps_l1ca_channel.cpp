/**
|====================================== gps_l1ca_channel.cpp ======================================|
|                                                                                                  |
|   Property of Daniel Sturdivant. Unauthorized copying of this file via any medium is would be    |
|   super sad and unfortunate for me. Proprietary and confidential.                                |
|                                                                                                  |
|--------------------------------------------------------------------------------------------------|
|                                                                                                  |
|   @file     src/adapters/gps_l1ca_channel.cpp                                                    |
|   @brief    GPS L1CA code generation and signal replica tools.                                   |
|   @author   Daniel Sturdivant <sturdivant20@gmail.com>                                           |
|   @date     April 2024                                                                           |
|                                                                                                  |
|==================================================================================================|
*/

#include "constellations/gps/gps_l1ca_channel.hpp"

#include <cmath>
#include <exception>
#include <iostream>

namespace sturdivant_sdr {

//* === GpsL1caChannel ===
GpsL1caChannel::GpsL1caChannel() : GpsLNAVParser(), __log(spdlog::default_logger()) {
}
GpsL1caChannel::GpsL1caChannel(const SdrConf& config, const uint8_t code_id, const uint32_t ch_id)
    : GpsLNAVParser(),
      __log(spdlog::default_logger()),
      prn_id(code_id),
      channel_id(ch_id),
      conf(config) {
    init(conf);
}

//* === GpsL1caChannel ===
GpsL1caChannel::~GpsL1caChannel() {
    // delete acq;
    // delete trk;
}

//* === init ===
void GpsL1caChannel::init(const SdrConf& conf) {
    // acq = new PcpsAcquire(conf.acqconf);
    acq = std::make_unique<PcpsAcquire>(PcpsAcquire(conf.acqconf));
    if (conf.trkconf.do_kalman_filter) {
        // trk = new TrackingKF(conf.trkconf);
        trk = std::make_unique<TrackingKF>(TrackingKF(conf.trkconf));
    } else {
        // trk = new TrackingLF(conf.trkconf);
        trk = std::make_unique<TrackingLF>(TrackingLF(conf.trkconf));
    }
    generate_l1ca_code();

    // acquisition/tracking variables
    samp_per_ms = static_cast<int>(conf.sigconf.sampling_freq / 1000.0);
    current_integration_ms = static_cast<int>(GPS_L1CA_CODE_PERIOD * 1000.0);
    current_integration_s = GPS_L1CA_CODE_PERIOD;
    memset(bit_detector_histogram, 0u, 20);  // set to all zero
    bit_flip_idx = 255;

    // observables variables
    TOW = 0;
    IP0.resize(conf.obsconf.lock_detector_integration_time_ms);
    IP1.resize(conf.obsconf.lock_detector_integration_time_ms);
    QP1.resize(conf.obsconf.lock_detector_integration_time_ms);
    if (!conf.obsconf.cn0_estimator.compare("moments")) {
        cn0_estimator = false;
    } else {
        cn0_estimator = true;
    }
    cn0_estimator_len = conf.obsconf.lock_detector_integration_time_ms;
    current_ms = 0u;
    cn0_idx = 0u;
    lock_fail_count = 0u;
    max_lock_fail_count = 100u;
    PHASE_DETECTOR = 0.85;

    // set up acquisition
    acq->set_channel_id(channel_id);
    acq->set_sampling_freq(conf.sigconf.sampling_freq);
    acq->set_mixing_freq(conf.sigconf.mix_freq);
    acq->create_fft_plan(samp_per_ms * conf.acqconf.coherent_integration_time_ms);
    acq->set_local_code(local_code, GPS_L1CA_CHIP_FREQ, GPS_L1CA_CODE_LEN);

    // set tracking variables available
    trk->set_channel_id(channel_id);
    trk->set_local_code(local_code, GPS_L1CA_CODE_LEN);
    trk->set_code_to_carrier_ratio(GPS_L1CA_CHIP_FREQ, GPS_L1_FREQ);
    trk->set_mixing_freq(conf.sigconf.mix_freq);
    trk->set_sampling_freq(conf.sigconf.sampling_freq);
    trk->set_chip_freq(GPS_L1CA_CHIP_FREQ);
    trk->set_integration_period(GPS_L1CA_CODE_PERIOD);
    trk->set_dump_callback(std::bind(&GpsL1caChannel::tracking_callback, this));

    // open file
    std::stringstream ss;
    ss << "tracking_ch" << channel_id << ".csv";
    fid.open(ss.str(), std::ios::out);
    fid << "ms,IP,IE,IL,QP,QE,QL,doppler" << std::endl;

    __log->info("Channel {} created!", channel_id);
}
void GpsL1caChannel::init(const SdrConf& config, const uint8_t code_id, const uint32_t ch_id) {
    conf = config;
    prn_id = code_id;
    channel_id = ch_id;
    init(conf);
}

//* === run ===
// void GpsL1caChannel::run() {
//     // try acquisition
//     predicted_samples = samp_per_ms;
//     std::vector<std::complex<double>> in_stream = data_stream_input(predicted_samples);
//     if (try_acquisition(in_stream)) {
//         // std::cout << "acquisition success" << std::endl;
//         std::cout << std::endl;

//         // skip forward acquired number of samples
//         in_stream = data_stream_input(acq->code_max_idx);
//         predicted_samples = trk->predict_num_samples();

//         while (current_ms < static_cast<uint32_t>(conf.fileconf.ms_to_process)) {
//             in_stream = data_stream_input(predicted_samples);
//             trk->accumulate_samples(in_stream);
//             // std::cout << "current ms = " << current_ms << std::endl;
//             // std::cout << "end of for loop" << std::endl;
//             // std::cout << std::endl;
//         }
//     };
// }

// //* === set_data_stream_callback ===
// void GpsL1caChannel::set_data_stream_callback(
//     std::function<std::vector<std::complex<double>>(int)> function) {
//     data_stream_input = function;
// }

// //* === try_acquisition ===
// bool GpsL1caChannel::try_acquisition(std::vector<std::complex<double>>& acq_stream) {
//     if (acq->acquire(acq_stream, samp_per_ms)) {
//         // if success initialize tracking
//         trk->set_code_phase(0.0);
//         trk->set_doppler_freq(acq->doppler);
//         trk->set_cn0(acq->CN0);
//         trk->set_lock_status(2);
//         trk->init_filter();

//         CN0 = acq->CN0;
//         CN0_dB = 10.0 * std::log10(CN0);

//         __log->info("PRN {}", prn_id);
//         __log->info("Success: {}", acq->success);
//         __log->info("Code Sample: {}", acq->code_max_idx);
//         __log->info("Carrier Doppler: {}", acq->doppler);
//         __log->info("Ratio: {}", acq->ratio);
//         __log->info("Peak Size: {}", acq->peak_size);
//         __log->info("Mean: {}", acq->bin_max_mean);
//         __log->info("Stdev: {}", acq->bin_max_stdev);
//         __log->info("CN0: {}", CN0_dB);
//         acq->clear();

//         return true;
//     }
//     return false;
// }

void GpsL1caChannel::set_input_data_vector(std::vector<std::complex<double>>* stream) {
    data_stream = stream;
    data_stream_size = static_cast<int>(data_stream->size());
    std::cout << "data_stream_size = " << data_stream_size << std::endl;
}

void GpsL1caChannel::set_mutex_and_conditions(std::atomic<int>* cond, bool* rep,
                                              std::shared_mutex* mtx,
                                              std::condition_variable_any* cond_var) {
    condition = cond;
    repeat = rep;
    m = mtx;
    cv = cond_var;
}

void GpsL1caChannel::run() {
    {
        // make sure to initially wait for reading thread to initialize
        std::shared_lock<std::shared_mutex> lk(*m);
        cv->wait(lk, [this]() { return ((*condition) == 0); });
    }
    if (try_acquisition()) {
        // std::cout << "channel " << channel_id << " acquired!" << std::endl;
        // acquisition handover, start tracking at beginning of code period
        trk->predict_num_samples();
        int stream_count = acq->code_max_idx;
        int stream_available = data_stream_size - stream_count;
        // trk->accumulate_samples(*data_stream, stream_available, stream_count);
        // stream_available = data_stream_size;
        // stream_count = 0;
        // (*condition) += 1;

        while (*repeat) {
            // std::cout << "channel " << channel_id << " at loop!" << std::endl;

            // wait for new data
            std::shared_lock<std::shared_mutex> lk(*m);
            cv->wait(lk, [this]() { return ((*condition) == 0); });

            // run tracking loop on accumulated data
            // std::cout << "stream_available = " << stream_available << std::endl;
            // std::cout << "stream_count = " << stream_count << std::endl;
            // std::cout << "tracking stream = " << std::endl;
            // for (int i = 0; i < 10; i++) {
            //     std::cout << data_stream->at(i) << ", ";
            // }
            // std::cout << std::endl << std::endl;
            trk->accumulate_samples(*data_stream, stream_available, stream_count);

            // reset condition to complete
            stream_available = data_stream_size;
            stream_count = 0;
            (*condition) += 1;
        }
    }
}

// void GpsL1caChannel::run() {
//     std::thread th(&GpsL1caChannel::channel_thread, this);
//     th.join();
// }

bool GpsL1caChannel::try_acquisition() {
    // std::cout << "here2" << std::endl;
    if (acq->acquire(*data_stream, samp_per_ms)) {
        // if success initialize tracking
        trk->set_code_phase(0.0);
        trk->set_doppler_freq(acq->doppler);
        trk->set_cn0(acq->CN0);
        trk->set_lock_status(2);
        trk->init_filter();

        CN0 = acq->CN0;
        CN0_dB = 10.0 * std::log10(CN0);

        __log->info(
            " \n PRN {} \n Success: {} \n Code Sample: {} \n Carrier Doppler: {} \n"
            " Ratio: {} \n Peak Size: {} \n Mean: {} \n Stdev: {} \n CN0: {} \n ",
            prn_id, acq->success, acq->code_max_idx, acq->code_max_idx, acq->doppler, acq->ratio,
            acq->peak_size, acq->bin_max_mean, acq->bin_max_stdev, CN0_dB);
        // __log->info("Success: {}", acq->success);
        // __log->info("Code Sample: {}", acq->code_max_idx);
        // __log->info("Carrier Doppler: {}", acq->doppler);
        // __log->info("Ratio: {}", acq->ratio);
        // __log->info("Peak Size: {}", acq->peak_size);
        // __log->info("Mean: {}", acq->bin_max_mean);
        // __log->info("Stdev: {}", acq->bin_max_stdev);
        // __log->info("CN0: {} \n", CN0_dB);
        acq->clear();

        return true;
    }
    return false;
}

//* === tracking_callback ===
void GpsL1caChannel::tracking_callback() {
    // std::cout << "tracking callback" << std::endl;

    corr = trk->get_correlators();
    state = trk->get_tracking_state();
    IP1[cn0_idx] = corr.IP;
    QP1[cn0_idx] = corr.QP;
    cn0_idx += 1;

    fid << current_ms << "," << corr.IP << "," << corr.IE << "," << corr.IL << "," << corr.QP << ","
        << corr.QE << "," << corr.QL << "," << state.carrier_doppler << std::endl;

    // check if new CN0 calculation is necessary
    if (cn0_idx == cn0_estimator_len) {
        // std::cout << "cn0 update request" << std::endl;
        lock_detectors();
        cn0_idx = 0;
    }

    // close the tracking loop
    trk->run_filter();
    predicted_samples = trk->predict_num_samples();
    if (current_integration_ms < MAX_LNAV_INTEGRATION_PERIOD_MS) {
        data_bit_detectors();
    }
    current_ms += current_integration_ms;
}

//! ================================================================================================

bool GpsL1caChannel::lock_detectors() {
    // std::cout << "lock detectors, current_ms = " << current_ms << std::endl;
    // estimate new CN0
    double new_CN0;
    if (cn0_estimator && current_ms > (2 * conf.obsconf.lock_detector_integration_time_ms)) {
        // std::cout << "beaulieu's method" << std::endl;
        new_CN0 = cn0_beaulieu_estimate(IP1, IP0, current_integration_s);
    } else {
        // std::cout << "moments method" << std::endl;
        new_CN0 = cn0_m2m4_estimate(IP1, QP1, current_integration_s);
    }
    IP0 = IP1;
    // std::cout << "new CN0 estimate = " << 10 * std::log10(new_CN0) << std::endl;

    // CN0 moving average filter
    CN0 = (0.95 * CN0) + (0.05 * new_CN0);
    CN0_dB = 10.0 * std::log10(CN0);
    trk->set_cn0(CN0);

    // estimate new carrier lock
    double new_phase_lock = carrier_lock_detector(IP1, QP1);
    PHASE_DETECTOR = (0.95 * PHASE_DETECTOR) + (0.05 * new_phase_lock);

    // check for lock
    if (current_ms > conf.trkconf.min_pull_in_time_ms) {
        if (CN0 > 30 && PHASE_DETECTOR > 0.85) {
            if (lock_fail_count > 0) {
                lock_fail_count -= 1;
                if (lock_fail_count < static_cast<uint16_t>(0.5 * max_lock_fail_count)) {
                    trk->set_lock_status(1);
                } else {
                    trk->set_lock_status(2);
                }
            } else {
                trk->set_lock_status(0);
            }
        } else {
            lock_fail_count += 1;
        }
    }

    if (lock_fail_count > max_lock_fail_count) {
        __log->warn("Loss of lock on Channel {}, GPS{}", channel_id, prn_id);
        return false;
    }

    // __log->info("PRN {}", prn_id);
    // __log->info("Channel {}", channel_id);
    // __log->info("CN0: {}", CN0_dB);
    // __log->info("Carrier Lock: {}, {}", PHASE_DETECTOR > 0.85, PHASE_DETECTOR);
    // __log->info("Lock Fail Count: {}", lock_fail_count);
    // std::cout << std::endl;
    return true;
}

void GpsL1caChannel::data_bit_detectors() {
    // check for bit polarity changes
    if (current_ms > conf.trkconf.min_pull_in_time_ms) {
        if (bit_flip_idx == 255) {
            // check for bit flip
            uint16_t idx1, idx2;
            if (cn0_idx == 0) {
                idx1 = conf.obsconf.lock_detector_integration_time_ms - 1u;
                idx2 = conf.obsconf.lock_detector_integration_time_ms - 2u;
            } else if (cn0_idx == 1) {
                idx1 = 0u;
                idx2 = conf.obsconf.lock_detector_integration_time_ms - 1u;
            } else {
                idx1 = cn0_idx - 1u;
                idx2 = cn0_idx - 2u;
            }
            int a = static_cast<int>(std::copysign(1.0, IP1[idx1]));
            int b = static_cast<int>(std::copysign(1.0, IP1[idx2]));
            // std::cout << "current_ms = " << current_ms << ", cn0_idx = " << cn0_idx
            //           << ", idx1 = " << idx1 << ", idx2 = " << idx2 << ", IP1 = " << IP1[idx1]
            //           << ", IP2 = " << IP1[idx2] << std::endl;
            if (a != b) {
                bit_detector_histogram[current_ms % 20] += 1;
            }

            // check for bit syncronization
            if (std::any_of(bit_detector_histogram, bit_detector_histogram + 20,
                            [](uint8_t x) { return x >= 10; })) {
                bit_flip_idx = current_ms % 20;
                __log->info("Channel {}, GPS{} - BIT FLIP! -- current_ms = {}, bit_flip_idx = {}",
                            channel_id, prn_id, current_ms, bit_flip_idx);
                // for (int i = 0; i < 20; i++) {
                //     std::cout << static_cast<int>(bit_detector_histogram[i]) << ", ";
                // }
                // std::cout << std::endl;

                ms_remaining = MAX_LNAV_INTEGRATION_PERIOD_MS - 1u;
            }
        } else {
            // bit flip has been found
            ms_remaining -= 1;
            if (ms_remaining < 1) {
                current_integration_ms = MAX_LNAV_INTEGRATION_PERIOD_MS;
                trk->set_integration_period(MAX_LNAV_INTEGRATION_PERIOD_S);
                // std::cout << "here, current_ms = " << current_ms << std::endl;
                cn0_idx = 0;
                cn0_estimator_len =
                    conf.obsconf.lock_detector_integration_time_ms / current_integration_ms;
                // std::cout << "new_size = " << cn0_estimator_len << std::endl;
                IP0.resize(cn0_estimator_len);
                IP1.resize(cn0_estimator_len);
                QP1.resize(cn0_estimator_len);
            }
            // std::cout << "ms_remaining = " << ms_remaining << std::endl;
        }
    }
}

//! ================================================================================================

//* === generate_l1ca_code ===
void GpsL1caChannel::generate_l1ca_code() {
    int8_t tap_idx = prn_id - 1;  // zero-indexing
    int8_t s1 = L1CA_TAPS[tap_idx][0];
    int8_t s2 = L1CA_TAPS[tap_idx][1];
    int8_t G1[10] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};  // initialize 10 bit number to all 1's
    int8_t G2[10] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    int8_t f1 = 0;
    int8_t f2 = 0;

    for (int i = 0; i < 1023; i++) {
        local_code[i] = 2 * ((((G2[s1] + G2[s2]) % 2) + G1[9]) % 2) - 1;  // +/- 1
        f1 = (G1[9] + G1[2]) % 2;
        f2 = (G2[9] + G2[8] + G2[7] + G2[5] + G2[2] + G2[1]) % 2;
        for (int j = 9; j > 0; j--) {
            G1[j] = G1[j - 1];
            G2[j] = G2[j - 1];
        }
        G1[0] = f1;
        G2[0] = f2;
    }
};

}  // namespace sturdivant_sdr