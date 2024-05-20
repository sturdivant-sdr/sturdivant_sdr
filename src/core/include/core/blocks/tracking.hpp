/**
|========================================== tracking.hpp ==========================================|
|                                                                                                  |
|   Property of Daniel Sturdivant. Unauthorized copying of this file via any medium is would be    |
|   super sad and unfortunate for me. Proprietary and confidential.                                |
|                                                                                                  |
|--------------------------------------------------------------------------------------------------|
|                                                                                                  |
|   @file     include/sdr/tracking_.hpp                                                            |
|   @brief    Standard GNSS tracking using loop filters.                                           |
|   @author   Daniel Sturdivant <sturdivant20@gmail.com>                                           |
|   @date     April 2024                                                                           |
|   @ref      "Understanding GPS/GNSS Principles and Applications 3rd Ed." - Kaplan and Hegarty    |
|                                                                                                  |
|==================================================================================================|
*/

#ifndef STURDIVANT_SDR_TRACKING_HPP
#define STURDIVANT_SDR_TRACKING_HPP

#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <complex>
#include <vector>

#include "core/param/structures.hpp"

namespace sturdivant_sdr {

class Tracking {
  public:
    //* === Tracking ===
    /// @brief constructor
    /// @param conf  Tracking configuration
    Tracking(const TrackingConf& conf);

    //* === ~Tracking ===
    virtual ~Tracking() = default;
    // /// @brief destructor
    // ~Tracking();

    //* === set_channel_id ===
    /// @brief set channel in order to make sure correct messages are synced
    void set_channel_id(const uint32_t& channel);

    //* === set_code_to_carrier_ratio ===
    /// @param true_chip_rate    actual cdma code chipping rate
    /// @param true_carrier_rate actual carrier frequency of received signal
    void set_code_to_carrier_ratio(const double& true_chip_rate, const double& true_carrier_freq);

    //* === set_local_code ===
    /// @brief set local prn sequence
    void set_local_code(int8_t* code, int len);

    //* === set_sampling_freq ===
    /// @brief set receiver sampling frequency
    void set_sampling_freq(const double& freq);

    //* === set_mixing_freq ===
    /// @brief set receiver mixing/carrier frequency
    void set_mixing_freq(const double& freq);

    //* === set_doppler_freq ===
    /// @brief set acquired carrier doppler
    void set_doppler_freq(const double& freq);

    //* === set_chip_freq ===
    /// @brief set nominal code chipping freq
    void set_chip_freq(const double& freq);

    //* === set_code_phase ===
    /// @brief set acquired code phase
    void set_code_phase(const double& phase);

    //* === set_current_sample ===
    /// @brief set acquired code phase
    void set_current_sample(const int& sample);

    //* === set_dump_callback ===
    void set_dump_callback(std::function<void()> function);

    //* === accumulate_samples ===
    /// @brief accumulate received samples
    void accumulate_samples(std::vector<std::complex<double>> samples, int& samp_available,
                            int& samp_idx);
    // void accumulate_samples(std::complex<double>* samples);

    //* === reset_correlators ===
    /// @brief after accumulation, dump and set correlators to zero
    void reset_correlators();

    //* === get_correlators ===
    Correlators get_correlators();

    //* === get_tracking_state ===
    TrackingState get_tracking_state();

    //* ##### virtual functions #####

    //* === init ===
    virtual void init() = 0;

    //* === init_filter ===
    virtual void init_filter() = 0;

    //* === run_filter ===
    virtual void run_filter() = 0;

    //* === set_integration_period ===
    /// @brief set the new integration period
    virtual void set_integration_period(const double& t_int) = 0;

    //* === set_cn0 ===
    /// @brief set the new cn0
    virtual void set_cn0(const double& cn0) = 0;

    //* === set_lock_status ===
    virtual void set_lock_status(uint8_t status) = 0;

    //* === predict_num_samples ===
    /// @brief predict the number of samples necessary in the next integration period
    virtual int predict_num_samples() = 0;

    // protected:
    // config parameters
    std::shared_ptr<spdlog::logger> __log;
    uint32_t channel_id;
    uint8_t lock_status;

    // NCO variables
    int8_t* local_code;
    double code_len;
    double correlator_spacing;  // spacing between E, P, L correlators
    int total_samples;          // total number of samples in current integration period
    int half_samples;           // half the total number of samples
    int current_sample;         // sample index of current integration period
    double rem_code_phase;
    double rem_carrier_phase;
    std::function<void()> dump_callback;

    // frequency variables
    double mix_freq;       // intermediate mixing frequency
    double chip_freq;      // code chipping rate
    double sampling_freq;  // receiver sampling frequency
    double beta;           // ratio between carrier frequency and code frequency

    // tracking variables
    double T;             // length in seconds of current integration period
    double N_ms;          // number of ms in tracking period
    double CN0;           // carrier-to-noise-density ratio magnitude
    TrackingState state;  // current phases and frequencies of the tracking loops
    Correlators corr;     // early, prompt, and late tracking correlators
    double (*dll_discriminator)(Correlators&);
    double (*fll_discriminator)(Correlators&, double&);
    double (*pll_discriminator)(Correlators&);
};

//! ================================================================================================

class TrackingLF : public Tracking {
  public:
    //* === TrackingLF ===
    /// @brief constructor
    /// @param conf  Tracking configuration
    TrackingLF(const TrackingConf& conf);

    //* === ~TrackingLF ===
    /// @brief destructor
    // ~TrackingLF();

    //* ##### virtual functions #####

    //* === init ===
    void init();

    //* === init_filter ===
    void init_filter();

    //* === run_filter ===
    void run_filter();

    //* === set_integration_period ===
    /// @brief set the new integration period
    void set_integration_period(const double& t_int);

    //* === set_cn0 ===
    /// @brief set the new cn0
    void set_cn0(const double& cn0);

    //* === set_lock_status ===
    void set_lock_status(uint8_t status);

    //* === predict_num_samples ===
    /// @brief predict the number of samples necessary in the next integration period
    int predict_num_samples();

  private:
    // loop filter variables
    double pll_gain_wide[3];
    double fll_gain_wide[2];
    double dll_gain_wide[2];
    double pll_gain_narrow[3];
    double fll_gain_narrow[2];
    double dll_gain_narrow[2];
    double pll_gain[3];
    double fll_gain[2];
    double dll_gain[2];
    double* Kp;
    double* Kf;
    double* Kd;
    double phase_err;
    double freq_err;
    double chip_err;
    double pll_w;
    double pll_x;
    double dll_x;
    double pll_bw_hz;
    double fll_bw_hz;
    double dll_bw_hz;
};

//! ================================================================================================

class TrackingKF : public Tracking {
  public:
    //* === TrackingKF ===
    /// @brief constructor
    /// @param conf  Tracking configuration
    TrackingKF(const TrackingConf& conf);

    //* === ~TrackingKF ===
    /// @brief destructor
    // ~TrackingKF();

    //* ##### virtual functions #####

    //* === init ===
    void init();

    //* === init_filter ===
    void init_filter();

    //* === run_filter ===
    void run_filter();

    //* === set_integration_period ===
    /// @brief set the new integration period
    void set_integration_period(const double& t_int);

    //* === set_cn0 ===
    /// @brief set the new cn0
    void set_cn0(const double& cn0);

    //* === set_lock_status ===
    void set_lock_status(uint8_t status);

    //* === predict_num_samples ===
    /// @brief predict the number of samples necessary in the next integration period
    int predict_num_samples();

  private:
    // Kalman Filter matrices
    //! KALMAN FILTER WORKS
    Eigen::Matrix<double, 5, 5> I;  // = Eigen::Matrix<double, 5, 5>::Identity();
    Eigen::Matrix<double, 5, 5> A;  // state transition matrix
    Eigen::Vector<double, 5> B;     // constant input matrix
    Eigen::Matrix<double, 3, 5> C;  // observation matrix
    Eigen::Matrix<double, 5, 5> Q;  // process noise covariance
    Eigen::Matrix<double, 3, 3> R;  // measurement noise covariance
    Eigen::Matrix<double, 5, 5> P;  // state covariance
    Eigen::Vector<double, 5> x;     // state
    Eigen::Vector<double, 3> dy;    // innovation
    Eigen::Matrix<double, 3, 3> S;  // innovation covariance
    Eigen::Matrix<double, 5, 3> K;  // kalman gain
    Eigen::Matrix<double, 5, 3> PCt;
    Eigen::Matrix<double, 5, 5> I_KC;
    double pll_bw_hz;  // PLL process covariance (T/2*bandwidth)
    double fll_bw_hz;  // FLL process covariance
    double dll_bw_hz;  // DLL process covariance
    double q_gain;

    //* === update_state_transition_matrix ===
    void update_state_transition_matrix();

    //* === update_input_matrix ===
    void update_input_matrix();

    //* === update_process_noise_covariance ===
    void update_process_noise_covariance();

    //* === update_measurement_noise_covariance ===
    void update_measurement_noise_covariance();
};

}  // namespace sturdivant_sdr

#endif