/**
|======================================== pcps_acquire.cpp ========================================|
|                                                                                                  |
|   Property of Daniel Sturdivant. Unauthorized copying of this file via any medium is would be    |
|   super sad and unfortunate for me. Proprietary and confidential.                                |
|                                                                                                  |
|--------------------------------------------------------------------------------------------------|
|                                                                                                  |
|   @file     src/sdr/pcps_acquire.cpp                                                             |
|   @brief    Abstract (default) class for the parallel code phase search acquisition method.      |
|   @author   Daniel Sturdivant <sturdivant20@gmail.com>                                           |
|   @date     April 2024                                                                           |
|                                                                                                  |
|==================================================================================================|
*/

#include "core/blocks/pcps_acquire.hpp"

#include <cmath>
#include <exception>
#include <iostream>

#include "core/algorithms/statistics.hpp"
#include "core/param/math_constants.hpp"

namespace sturdivant_sdr {

//* === PcpsAcquire ===
PcpsAcquire::PcpsAcquire(const AcquisitionConf& conf)
    // : __fine_freq_search{conf.do_fine_search},
    : doppler_bin_width{conf.doppler_step_hz},
      doppler_bandwidth{conf.doppler_max_hz},
      threshold{conf.threshold},
      __log{spdlog::default_logger()} {
    coherent_integration_time_s = static_cast<double>(conf.coherent_integration_time_ms) / 1000.0;
    init();
};

//* === ~PcpsAcquire ===
PcpsAcquire::~PcpsAcquire() {};

//* === init ===
bool PcpsAcquire::init() {
    try {
        size_t num_doppler_bins = 2 * (doppler_bandwidth / doppler_bin_width) + 1;
        doppler_bins.resize(num_doppler_bins);

        double doppler = -doppler_bandwidth;
        for (size_t ii = 0; ii < num_doppler_bins; ii++) {
            doppler_bins[ii] = doppler;
            doppler += doppler_bin_width;
        }

        return true;
    } catch (std::exception& e) {
        __log->error("PcpsAcquire::init -> Unable to initialize acquisition! ERROR {}!", e.what());
        return false;
    }
}

//* === clear ===
void PcpsAcquire::clear() {
    // empty vectors to clear memory
    std::vector<std::complex<double>>().swap(local_code);
    // std::vector<std::complex<double>>().swap(local_carrier);
    std::vector<std::complex<double>>().swap(fft_input);
    std::vector<std::complex<double>>().swap(fft_result);
    std::vector<std::complex<double>>().swap(ifft_input);
    std::vector<std::complex<double>>().swap(ifft_result);
    std::vector<double>().swap(doppler_bins);
}

//* === set_fft_plan ===
void PcpsAcquire::create_fft_plan(int len) {
    // create acquisition fft plan
    fft_input.resize(len);
    fft_result.resize(len);
    ifft_input.resize(len);
    ifft_result.resize(len);
    fft.create_fft_plan(fft_input, fft_result, len);
    fft.create_ifft_plan(ifft_input, ifft_result, len);

    // reserve appropriate size for carrier replica as well
    // local_carrier.resize(len);
    local_code.resize(len);
}

//* === set_channel_id ===
void PcpsAcquire::set_channel_id(const uint32_t& channel) {
    channel_id = channel;
}

//* === set_local_code ===
void PcpsAcquire::set_local_code(int8_t* prn, const double& code_freq, int len) {
    // upsample code
    double size = static_cast<double>(len);
    double code_phase = 0.0;
    double phase_step = code_freq / sampling_freq;
    // code_replica(prn, fft_input, rem_phase, code_freq, sampling_freq);
    for (int ii = 0; ii < static_cast<int>(fft_input.size()); ii++) {
        fft_input[ii] = prn[static_cast<int>(std::fmod(code_phase, size))];
        code_phase += phase_step;
    }

    // take fft and move result into appropriate vector
    fft.fft();
    local_code = fft_result;

    // conjugate code
    std::complex<double> N_complex(len, 0);
    for (std::complex<double>& x : local_code) {
        x = std::conj(x) / N_complex;
    }
}

//* === set_carrier_freq ===
void PcpsAcquire::set_mixing_freq(const double& freq) {
    mix_freq = freq;
}

//* === set_carrier_freq ===
void PcpsAcquire::set_sampling_freq(const double& freq) {
    sampling_freq = freq;
}

//* === acquire ===
//! i don't know what i'm doing :(
bool PcpsAcquire::acquire(const std::vector<std::complex<double>>& in_stream, int samp_per_ms) {
    // std::cout << "here3" << std::endl;
    // initialize acquisition plane
    acq_plane.resize(doppler_bins.size(), std::vector<double>(samp_per_ms));

    // std::cout << "acquisition stream = " << std::endl;
    // for (int i = 0; i < 10; i++) {
    //     std::cout << in_stream[i] << ", ";
    // }
    // std::cout << std::endl << std::endl;

    // acquisition loop
    // double tmp;
    int N = static_cast<int>(fft_input.size());

    // std::complex<double> carr_rep;
    double carrier_phase;
    double phase_step;

    for (int dop_i = 0; dop_i < static_cast<int>(doppler_bins.size()); dop_i++) {
        // std::cout << doppler_bins[dop_i] << std::endl;

        // generate carrier replica
        carrier_phase = 0.0;
        phase_step = TWO_PI * (mix_freq + doppler_bins[dop_i]) / sampling_freq;
        // carrier_replica(local_carrier, rem_carrier_phase, carrier_freq, sampling_freq,
        //                 doppler_bins[dop_i]);

        // modulate replica and received signal stream
        for (int ii = 0; ii < N; ii++) {
            fft_input[ii] = std::exp(-J * carrier_phase) * in_stream[ii];
            carrier_phase += phase_step;
            // fft_input[ii] = local_carrier[ii] * in_stream[ii];
        }
        fft.fft();

        // modulate signal, carrier, and code in the frequency domain
        for (int ii = 0; ii < N; ii++) {
            ifft_input[ii] = fft_result[ii] * local_code[ii];
        }
        fft.ifft();

        // save the power to the acquisition plane
        for (int ii = 0; ii < samp_per_ms; ii++) {
            // tmp = std::abs(ifft_result[ii]);
            // acq_plane[dop_i][ii] += tmp * tmp;
            acq_plane[dop_i][ii] = ifft_result[ii].real() * ifft_result[ii].real() +
                                   ifft_result[ii].imag() * ifft_result[ii].imag();
        }
    }
    // std::cout << "end of acquisition loop" << std::endl;

    // grab statistics
    doppler_max_idx = 0;
    code_max_idx = 0;
    peak_size = 0.0;
    for (int ii = 0; ii < static_cast<int>(doppler_bins.size()); ii++) {
        for (int jj = 0; jj < samp_per_ms; jj++) {
            if (acq_plane[ii][jj] > peak_size) {
                doppler_max_idx = ii;
                code_max_idx = jj;
                peak_size = acq_plane[ii][jj];
            }
        }
    }

    // code_max_idx = code_max_idx % samp_per_ms;
    doppler = doppler_bins[doppler_max_idx];
    bin_max_mean = mean(acq_plane[doppler_max_idx]);
    bin_max_stdev = stdev(acq_plane[doppler_max_idx], bin_max_mean);
    ratio = (peak_size - bin_max_mean) / bin_max_stdev;
    CN0 = (peak_size - 4.0 * bin_max_stdev) / (bin_max_stdev * coherent_integration_time_s);

    // std::cout << "doppler = " << doppler << ", code_phase = " << code_max_idx << std::endl
    //           << std::endl;

    success = false;
    if (ratio >= threshold) {
        success = true;
    }

    return success;
}

}  // namespace sturdivant_sdr