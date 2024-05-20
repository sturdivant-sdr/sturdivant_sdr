/**
|======================================== pcps_acquire.hpp ========================================|
|                                                                                                  |
|   Property of Daniel Sturdivant. Unauthorized copying of this file via any medium is would be    |
|   super sad and unfortunate for me. Proprietary and confidential.                                |
|                                                                                                  |
|--------------------------------------------------------------------------------------------------|
|                                                                                                  |
|   @file     include/sdr/pcps_acquire.hpp                                                         |
|   @brief    Abstract (default) class for the parallel code phase search acquisition method.      |
|   @author   Daniel Sturdivant <sturdivant20@gmail.com>                                           |
|   @date     April 2024                                                                           |
|                                                                                                  |
|==================================================================================================|
*/

// TODO: add more safety-debug checks
// TODO: look into "vkFFT" in order to use GPU for FFTs

#ifndef STURDIVANT_SDR_PCPS_ACQUIRE_HPP
#define STURDIVANT_SDR_PCPS_ACQUIRE_HPP

#include <spdlog/spdlog.h>

#include <complex>
#include <string>
#include <vector>

#include "core/param/structures.hpp"
#include "fftw_wrapper/fft.hpp"

namespace sturdivant_sdr {

class PcpsAcquire {
  public:
    //* === PcpsAcquire ===
    /// @brief constructor
    /// @param config Acquisition configuration
    PcpsAcquire(const AcquisitionConf& conf);

    //* === ~PcpsAcquire ===
    /// @brief destructor
    ~PcpsAcquire();

    //* === init ===
    /// @brief initialize acquisition search parameters
    /// @return True|False based on initialization success
    bool init();

    //* === clear ===
    /// @brief empty acquisition vectors to clear memory
    void clear();

    //* === create_fft_plan ===
    void create_fft_plan(int len);

    //* === set_channel_id ===
    /// @brief set channel in order to make sure correct messages are synced
    void set_channel_id(const uint32_t& channel);

    //* === set_prn_code ===
    /// @brief set local code replica for correlation
    void set_local_code(int8_t* prn, const double& code_freq, int len);

    //* === set_mixing_freq ===
    /// @brief set local carrier replica for correlation
    void set_mixing_freq(const double& freq);

    //* === set_sampling_freq ===
    /// @brief set receiver sampling frequency
    void set_sampling_freq(const double& freq);

    //* === acquire ===
    /// @brief run pcps acquisition methods
    bool acquire(const std::vector<std::complex<double>>& in_stream, int samp_per_ms);

    //* === acquisition results ===
    bool success;
    int doppler_max_idx;
    int code_max_idx;
    double doppler;
    double bin_max_mean;
    double bin_max_stdev;
    double ratio;
    double peak_size;
    double CN0;
    std::vector<std::vector<double>> acq_plane;

  private:
    // bool __fine_freq_search;
    double doppler_bin_width;
    double doppler_bandwidth;
    double threshold;
    std::shared_ptr<spdlog::logger> __log;

    double mix_freq;
    double sampling_freq;
    double coherent_integration_time_s;
    std::vector<std::complex<double>> local_code;
    // std::vector<std::complex<double>> local_carrier;
    std::vector<std::complex<double>> fft_input;
    std::vector<std::complex<double>> fft_result;
    std::vector<std::complex<double>> ifft_input;
    std::vector<std::complex<double>> ifft_result;
    std::vector<double> doppler_bins;
    uint32_t channel_id;
    fftw_wrapper::FftwWrapper fft;

};  // end PcpsAcquire
}  // namespace sturdivant_sdr

#endif