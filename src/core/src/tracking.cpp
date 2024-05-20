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

#include "core/blocks/tracking.hpp"

#include <cmath>
#include <iostream>

#include "core/algorithms/tracking_discriminators.hpp"
#include "core/param/math_constants.hpp"

namespace sturdivant_sdr {

//* === Tracking ===
Tracking::Tracking(const TrackingConf& conf)
    // : __fine_freq_search{conf.do_fine_search},
    : __log{spdlog::default_logger()}, correlator_spacing{conf.correlator_spacing_chips} {
}

//* === ~Tracking ===
// Tracking::~Tracking() {
// }

//* === set_channel_id ===
void Tracking::set_channel_id(const uint32_t& channel) {
    channel_id = channel;
}

//* === set_code_to_carrier_ratio ===
void Tracking::set_code_to_carrier_ratio(const double& true_chip_rate,
                                         const double& true_carrier_freq) {
    beta = true_chip_rate / true_carrier_freq;
}

//* === set_local_code ===
void Tracking::set_local_code(int8_t* code, int len) {
    local_code = code;
    code_len = static_cast<double>(len);
}

//* === set_sampling_freq ===
void Tracking::set_sampling_freq(const double& freq) {
    sampling_freq = freq;
}

//* === set_mixing_freq ===
void Tracking::set_mixing_freq(const double& freq) {
    mix_freq = freq;
}

//* === set_doppler_freq ===
void Tracking::set_doppler_freq(const double& freq) {
    state.carrier_doppler = freq;
}

//* === set_chip_freq ===
void Tracking::set_chip_freq(const double& freq) {
    chip_freq = freq;
}

//* === set_code_phase ===
void Tracking::set_code_phase(const double& phase) {
    state.code_phase = phase;
}

void Tracking::set_current_sample(const int& sample) {
    current_sample = sample;
}

//* === set_dump_callback ===
void Tracking::set_dump_callback(std::function<void()> function) {
    dump_callback = function;
}

//* === accumulate_samples ===
void Tracking::accumulate_samples(std::vector<std::complex<double>> samples, int& samp_available,
                                  int& samp_idx) {
    // void Tracking::accumulate_samples(std::complex<double>* samples) {
    double dt = 1.0 / sampling_freq;
    double t = static_cast<double>(current_sample) / sampling_freq;
    // std::cout << "init code_phase = " << rem_code_phase << std::endl;

    std::complex<double> tmp;
    std::complex<double> carrier_replica;
    double code_replica;

    // std::cout << "samples.size() = " << samples.size() << std::endl;
    // for (int ii = 0; ii < static_cast<int>(samples.size()); ii++) {
    // int jj = current_sample;
    for (int ii = 0; ii < samp_available; ii++) {
        // carrier replica
        carrier_replica = std::exp(-J * state.carrier_phase) * samples[ii + samp_idx];

        // TODO: there is probably a faster way to accumulate all 3 taps
        // prompt
        code_replica = local_code[static_cast<int>(std::fmod(state.code_phase, code_len))];
        tmp = code_replica * carrier_replica;
        if (current_sample < half_samples) {
            corr.ip1 += tmp.real();
            corr.qp1 += tmp.imag();
        } else {
            corr.ip2 += tmp.real();
            corr.qp2 += tmp.imag();
        }

        // early
        code_replica = local_code[static_cast<int>(
            std::fmod(state.code_phase + correlator_spacing, code_len))];
        tmp = code_replica * carrier_replica;
        corr.IE += tmp.real();
        corr.QE += tmp.imag();

        // late
        code_replica = local_code[static_cast<int>(
            std::fmod(state.code_phase - correlator_spacing, code_len))];
        tmp = code_replica * carrier_replica;
        corr.IL += tmp.real();
        corr.QL += tmp.imag();

        // increment time
        current_sample += 1;
        // t = static_cast<double>(current_sample) / sampling_freq;
        t += dt;
        state.code_phase = rem_code_phase + (chip_freq + state.code_doppler) * t;
        state.carrier_phase = rem_carrier_phase + TWO_PI * ((mix_freq + state.carrier_doppler) * t +
                                                            state.carrier_rate * t * t / 2);

        if (current_sample == total_samples) {
            // std::cout << "\x1B[32m"
            //           << "tracking callback requested " << ii << ", t = " << t << std::endl
            //           << "callback count = " << ii + 1 << std::endl
            //           << "callback current_sample = " << current_sample << std::endl
            //           << "\x1B[37m";
            // std::cout << "callback code_phase = " << state.code_phase << std::endl;
            corr.IP += corr.ip1 + corr.ip2;
            corr.QP += corr.qp1 + corr.qp2;
            dump_callback();

            // reset
            t = 0.0;
        }
    }

    // std::cout << "\x1B[35m" << "accumulation count = " << count << std::endl
    //           << "accumulation current_sample = " << current_sample << std::endl
    //           << "accumulation code_phase = " << state.code_phase << std::endl
    //           << "\x1B[37m";
}

//* === reset_correlators ===
void Tracking::reset_correlators() {
    corr.IP = 0.0;
    corr.IE = 0.0;
    corr.IL = 0.0;
    corr.QP = 0.0;
    corr.QE = 0.0;
    corr.QL = 0.0;
    corr.ip1 = 0.0;
    corr.ip2 = 0.0;
    corr.qp1 = 0.0;
    corr.qp2 = 0.0;
}

//* === get_correlators ===
Correlators Tracking::get_correlators() {
    return corr;
}

//* === get_correlators ===
TrackingState Tracking::get_tracking_state() {
    return state;
}

//! ================================================================================================

//* === TrackingLF ===
TrackingLF::TrackingLF(const TrackingConf& conf)
    // : __fine_freq_search{conf.do_fine_search},
    : Tracking{conf},
      pll_bw_hz{conf.pll_bw_hz},
      fll_bw_hz{conf.fll_bw_hz},
      dll_bw_hz{conf.dll_bw_hz} {
    // std::cout << "loop filter" << std::endl;
    init();
}

// //* === ~TrackingLF ===
// TrackingLF::~TrackingLF() {
// }

//* === init ===
void TrackingLF::init() {
    current_sample = 0;
    total_samples = 0;
    state.carrier_phase = 0.0;
    state.carrier_doppler = 0.0;
    state.carrier_rate = 0.0;
    state.code_phase = 0.0;
    state.code_doppler = 0.0;
    CN0 = 10000.0;  // 40 dB
    lock_status = 2;
    rem_code_phase = 0.0;
    rem_carrier_phase = 0.0;
}

//* === init_filter ===
void TrackingLF::init_filter() {
    // initailize loop filter
    pll_w = 0.0;
    pll_x = state.carrier_doppler;
    dll_x = state.code_doppler;

    double a2 = 1.414;
    double a3 = 1.1;
    double b3 = 2.4;

    // nominal gains
    double w0p = pll_bw_hz / 0.7845;
    double w0f = fll_bw_hz / 0.53;
    double w0d = dll_bw_hz / 0.53;
    pll_gain[0] = b3 * w0p;
    pll_gain[1] = a3 * w0p * w0p;
    pll_gain[2] = w0p * w0p * w0p;
    fll_gain[0] = a2 * w0f;
    fll_gain[1] = w0f * w0f;
    dll_gain[0] = a2 * w0d;
    dll_gain[1] = w0d * w0d;

    // wide gains
    w0p = 2.0 * pll_bw_hz / 0.7845;
    w0f = 2.0 * fll_bw_hz / 0.53;
    w0d = 2.0 * dll_bw_hz / 0.53;
    pll_gain_wide[0] = b3 * w0p;
    pll_gain_wide[1] = a3 * w0p * w0p;
    pll_gain_wide[2] = w0p * w0p * w0p;
    fll_gain_wide[0] = a2 * w0f;
    fll_gain_wide[1] = w0f * w0f;
    dll_gain_wide[0] = a2 * w0d;
    dll_gain_wide[1] = w0d * w0d;

    // narrow gains
    w0p = 0.6 * pll_bw_hz / 0.7845;
    w0f = 0.0;
    w0d = 0.6 * dll_bw_hz / 0.53;
    pll_gain_narrow[0] = b3 * w0p;
    pll_gain_narrow[1] = a3 * w0p * w0p;
    pll_gain_narrow[2] = w0p * w0p * w0p;
    fll_gain_narrow[0] = 0.0;
    fll_gain_narrow[1] = 0.0;
    dll_gain_narrow[0] = a2 * w0d;
    dll_gain_narrow[1] = w0d * w0d;
}

//* === run_filter ===
void TrackingLF::run_filter() {
    phase_err = pll_discriminator(corr);
    freq_err = fll_discriminator(corr, T);
    chip_err = dll_discriminator(corr);

    // 3rd order PLL with 2nd order FLL assist
    double new_pll_w = pll_w + (Kp[2] * phase_err + Kf[1] * freq_err) * T;
    double new_pll_x =
        pll_x + (0.5 * (pll_w + new_pll_w) + Kp[1] * phase_err + Kf[0] * freq_err) * T;
    state.carrier_doppler = 0.5 * (pll_x + new_pll_x) + Kp[0] * phase_err;
    state.carrier_rate = 0.5 * pll_w;  //? maybe
    // carrier_rate = 0.0;

    // 2nd order DLL with carrier aiding
    double new_dll_x = dll_x + Kd[1] * chip_err * T;
    state.code_doppler = (0.5 * (dll_x + new_dll_x) + Kd[0] * chip_err);
    state.code_doppler += beta * state.carrier_doppler;

    // save filter
    pll_w = new_pll_w;
    pll_x = new_pll_x;
    dll_x = new_dll_x;

    // make sure code and carrier phase is around 0
    state.code_phase = std::fmod(state.code_phase, code_len);
    if (state.code_phase > 0.5 * code_len) {
        state.code_phase -= code_len;
    }
    state.carrier_phase = std::fmod(state.carrier_phase, TWO_PI);
    rem_carrier_phase = state.carrier_phase;
    rem_code_phase = state.code_phase;
}

//* === set_integration_period ===
void TrackingLF::set_integration_period(const double& t_int) {
    T = t_int;
    N_ms = t_int * 1000.0;
}

//* === set_cn0 ===
void TrackingLF::set_cn0(const double& cn0) {
    CN0 = cn0;
}

//* === set_lock_status ===
void TrackingLF::set_lock_status(uint8_t status) {
    lock_status = status;
    switch (lock_status) {
        case 0:
            // std::cout << "case 0" << std::endl;
            Kp = pll_gain_narrow;
            Kf = fll_gain_narrow;
            Kd = dll_gain_narrow;
            correlator_spacing = 0.1;
            dll_discriminator = dll_cdp_normalized;
            fll_discriminator = fll_ddcp_normalized;
            pll_discriminator = pll_ddq_normalized;
            break;
        case 1:
            // std::cout << "case 1" << std::endl;
            Kp = pll_gain;
            Kf = fll_gain;
            Kd = dll_gain;
            correlator_spacing = 0.25;
            dll_discriminator = dll_nceml_normalized;
            fll_discriminator = fll_ddcp_normalized;
            pll_discriminator = pll_ddq_normalized;
            break;
        case 2:
            // std::cout << "case 2" << std::endl;
            Kp = pll_gain_wide;
            Kf = fll_gain_wide;
            Kd = dll_gain_wide;
            correlator_spacing = 0.5;
            dll_discriminator = dll_nceml_normalized;
            fll_discriminator = fll_atan2_normalized;
            pll_discriminator = pll_atan_normalized;
            break;
    }
}

//* === predict_num_samples ===
int TrackingLF::predict_num_samples() {
    double code_phase_step = (chip_freq + state.code_doppler) / sampling_freq;
    total_samples =
        static_cast<int>(std::ceil((N_ms * code_len - state.code_phase) / code_phase_step));
    half_samples = total_samples / 2;
    current_sample = 0;
    T = total_samples / sampling_freq;

    reset_correlators();

    // std::cout << "code_phase = " << code_phase << std::endl;
    // std::cout << "total_samples = " << total_samples << std::endl;
    // std::cout << "half_samples = " << half_samples << std::endl << std::endl;

    // std::cout << total_samples << " predicted" << std::endl;
    // std::cout << "\x1B[36m" << "N_ms = " << N_ms << ", code_len = " << code_len
    //           << ", code_phase = " << state.code_phase << ", code_phase_step = " <<
    //           code_phase_step
    //           << std::endl
    //           << "predicted total_samples = " << total_samples
    //           << ", doppler = " << state.carrier_doppler
    //           << ", code_doppler = " << state.code_doppler << std::endl
    //           << "\x1B[37m";
    return total_samples;
}

//! ================================================================================================

//* === TrackingKF ===
TrackingKF::TrackingKF(const TrackingConf& conf)
    // : __fine_freq_search{conf.do_fine_search},
    : Tracking{conf},
      pll_bw_hz{conf.pll_bw_hz},
      fll_bw_hz{conf.fll_bw_hz},
      dll_bw_hz{conf.dll_bw_hz} {
    // std::cout << "kalman filter" << std::endl;
    init();
}

// //* === ~TrackingKF ===
// TrackingKF::~TrackingKF() {
// }

//* === init ===
void TrackingKF::init() {
    current_sample = 0;
    total_samples = 0;
    state.carrier_phase = 0.0;
    state.carrier_doppler = 0.0;
    state.carrier_rate = 0.0;
    state.code_phase = 0.0;
    state.code_doppler = 0.0;
    CN0 = 10000.0;  // 40 dB
    lock_status = 2;
    I.setIdentity();
    rem_code_phase = 0.0;
    rem_carrier_phase = 0.0;
}

//* === init_filter ===
void TrackingKF::init_filter() {
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;
    double beta2 = beta * beta;

    // state transition matrix
    A << 1.0, T, T2 / 2.0, 0.0, 0.0,             //
        0.0, 1.0, T, 0.0, 0.0,                   //
        0.0, 0.0, 1.0, 0.0, 0.0,                 //
        0.0, beta * T, beta * T2 / 2.0, 1.0, T,  //
        0.0, 0.0, 0.0, 0.0, 1.0;
    // std::cout << "A = " << std::endl << A << std::endl;

    // input/control matrix
    B << T * mix_freq, 0.0, 0.0, T * chip_freq, 0.0;
    // std::cout << "B = " << std::endl << B << std::endl;

    // process noise covariance
    Q << fll_bw_hz * T5 / 20.0 + pll_bw_hz * T3 / 3.0,
        (pll_bw_hz * T2 + 4.0 * pll_bw_hz) * T2 / 8.0, T3 * fll_bw_hz / 6.0,
        beta * T3 * (3.0 * fll_bw_hz * T2 + 20.0 * pll_bw_hz) / 60.0, 0.0,  //
        fll_bw_hz * T4 / 8.0 + pll_bw_hz * T2 / 2.0, fll_bw_hz * T3 / 3.0 + pll_bw_hz * T,
        T2 * fll_bw_hz / 2.0, beta * T2 * (fll_bw_hz * T2 + 4.0 * pll_bw_hz) / 8.0, 0.0,  //
        T3 * fll_bw_hz / 6.0, T2 * fll_bw_hz / 2.0, T * fll_bw_hz, beta * T3 * fll_bw_hz / 6.0,
        0.0,  //
        beta * T3 * (3.0 * fll_bw_hz * T2 + 20.0 * pll_bw_hz) / 60.0,
        beta * T2 * (fll_bw_hz * T2 + 4.0 * pll_bw_hz) / 8.0, beta * T3 * fll_bw_hz / 6.0,
        T3 * (3.0 * fll_bw_hz * beta2 * T2 + 20.0 * pll_bw_hz * beta2 + 20.0 * dll_bw_hz) / 60.0,
        T2 * dll_bw_hz / 2.0,                                //
        0.0, 0.0, 0.0, T2 * dll_bw_hz / 2.0, T * dll_bw_hz;  //
    // std::cout << "Q = " << std::endl << Q << std::endl;

    // observation matrix
    C << 1.0, 0.0, 0.0, 0.0, 0.0,  //
        0.0, 1.0, 0.0, 0.0, 0.0,   //
        0.0, 0.0, 0.0, 1.0, 0.0;
    // std::cout << "C = " << std::endl << C << std::endl;

    // measurement noise covariance
    R << pll_variance(CN0, T), 0.0, 0.0,  //
        0.0, fll_variance(CN0, T), 0.0,   //
        0.0, 0.0, dll_variance(CN0, T, correlator_spacing);
    // std::cout << "R = " << std::endl << R << std::endl;

    // state covariance
    P << 0.5, 0.0, 0.0, 0.0, 0.0,  //
        0.0, 50.0, 0.0, 0.0, 0.0,  //
        0.0, 0.0, 1.0, 0.0, 0.0,   //
        0.0, 0.0, 0.0, 0.5, 0.0,   //
        0.0, 0.0, 0.0, 0.0, 1.0;
    // std::cout << "P = " << std::endl << P << std::endl;
    // std::cout << "I = " << std::endl << I << std::endl;

    // state (carrier phase, carrier doppler, carrier rate, code phase, code doppler)
    x << 0.0, state.carrier_doppler, 0.0, state.code_phase, 0.0;
}

//* === run_filter ===
void TrackingKF::run_filter() {
    P = A * P * A.transpose() + (Q * q_gain);
    x = A * x + B;

    // innovation (based on discriminators)
    dy << pll_discriminator(corr), fll_discriminator(corr, T), dll_discriminator(corr);
    // std::cout << std::endl << "dy = " << std::endl << dy << std::endl << std::endl;

    PCt = P * C.transpose();
    S = C * PCt + R;
    K = PCt * S.inverse();
    I_KC = I - K * C;

    P = (I_KC * P * I_KC.transpose()) + (K * R * K.transpose());
    x += K * dy;

    // save variables
    x[0] = std::fmod(x[0], 1.0);
    x[3] = std::fmod(x[3], code_len);
    if (x[3] > 0.5 * code_len) {
        x[3] -= code_len;
    }

    state.carrier_phase = x[0] * TWO_PI;
    state.carrier_doppler = x[1];
    state.carrier_rate = x[2];
    state.code_phase = x[3];
    state.code_doppler = x[4];
    rem_carrier_phase = state.carrier_phase;
    rem_code_phase = state.code_phase;
}

//* === set_integration_period ===
void TrackingKF::set_integration_period(const double& t_int) {
    T = t_int;
    N_ms = t_int * 1000.0;
    update_process_noise_covariance();
}

//* === set_lock_status ===
void TrackingKF::set_lock_status(uint8_t status) {
    lock_status = status;
    switch (lock_status) {
        case 0:
            // std::cout << "lock_status 0" << std::endl;
            q_gain = 50.0 / (N_ms);
            correlator_spacing = 0.1;
            dll_discriminator = dll_cdp_normalized;
            // fll_discriminator = fll_ddcp_normalized;
            // pll_discriminator = pll_ddq_normalized;
            fll_discriminator = fll_atan2_normalized;
            pll_discriminator = pll_atan_normalized;
            break;
        case 1:
            // std::cout << "lock_status 1" << std::endl;
            q_gain = 110.0 / (N_ms);
            correlator_spacing = 0.25;
            dll_discriminator = dll_nceml_normalized;
            // fll_discriminator = fll_ddcp_normalized;
            // pll_discriminator = pll_ddq_normalized;
            fll_discriminator = fll_atan2_normalized;
            pll_discriminator = pll_atan_normalized;
            break;
        case 2:
            // std::cout << "lock_status 2" << std::endl;
            q_gain = 250.0 / (N_ms);
            correlator_spacing = 0.5;
            dll_discriminator = dll_nceml_normalized;
            fll_discriminator = fll_atan2_normalized;
            pll_discriminator = pll_atan_normalized;
            break;
    }
    // std::cout << "lock_status = " << static_cast<int>(lock_status) << std::endl;
    // std::cout << "q_gain = " << q_gain << std::endl;
}

//* === set_cn0 ===
void TrackingKF::set_cn0(const double& cn0) {
    CN0 = cn0;
    update_measurement_noise_covariance();
}

//* === predict_num_samples ===
int TrackingKF::predict_num_samples() {
    double code_phase_step = (chip_freq + state.code_doppler) / sampling_freq;
    total_samples =
        static_cast<int>(std::ceil((N_ms * code_len - state.code_phase) / code_phase_step));
    half_samples = total_samples / 2;
    current_sample = 0;

    T = static_cast<double>(total_samples) / sampling_freq;
    update_input_matrix();
    update_state_transition_matrix();
    // update_process_noise_covariance();

    reset_correlators();

    // std::cout << "code_phase = " << code_phase << std::endl;
    // std::cout << "predicted total_samples = " << total_samples
    //           << ", doppler = " << state.carrier_doppler << std::endl;
    // std::cout << "half_samples = " << half_samples << std::endl << std::endl;

    // std::cout << "\x1B[36m" << "N_ms = " << N_ms << ", code_len = " << code_len
    //           << ", code_phase = " << state.code_phase << ", code_phase_step = " <<
    //           code_phase_step
    //           << std::endl
    //           << "predicted total_samples = " << total_samples
    //           << ", doppler = " << state.carrier_doppler
    //           << ", code_doppler = " << state.code_doppler << std::endl
    //           << "\x1B[37m";
    return total_samples;
}

//* === update_state_transition_matrix ===
void TrackingKF::update_state_transition_matrix() {
    double T2 = T * T;

    // state transition matrix
    A << 1.0, T, T2 / 2.0, 0.0, 0.0,             //
        0.0, 1.0, T, 0.0, 0.0,                   //
        0.0, 0.0, 1.0, 0.0, 0.0,                 //
        0.0, beta * T, beta * T2 / 2.0, 1.0, T,  //
        0.0, 0.0, 0.0, 0.0, 1.0;
}

//* === update_input_matrix ===
void TrackingKF::update_input_matrix() {
    // input/control matrix
    B << T * mix_freq, 0.0, 0.0, T * chip_freq, 0.0;
}

//* === update_process_noise_covariance ===
void TrackingKF::update_process_noise_covariance() {
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;
    double beta2 = beta * beta;

    Q << fll_bw_hz * T5 / 20.0 + pll_bw_hz * T3 / 3.0,
        (pll_bw_hz * T2 + 4.0 * pll_bw_hz) * T2 / 8.0, T3 * fll_bw_hz / 6.0,
        beta * T3 * (3.0 * fll_bw_hz * T2 + 20.0 * pll_bw_hz) / 60.0, 0.0,  //
        fll_bw_hz * T4 / 8.0 + pll_bw_hz * T2 / 2.0, fll_bw_hz * T3 / 3.0 + pll_bw_hz * T,
        T2 * fll_bw_hz / 2.0, beta * T2 * (fll_bw_hz * T2 + 4.0 * pll_bw_hz) / 8.0, 0.0,  //
        T3 * fll_bw_hz / 6.0, T2 * fll_bw_hz / 2.0, T * fll_bw_hz, beta * T3 * fll_bw_hz / 6.0,
        0.0,  //
        beta * T3 * (3.0 * fll_bw_hz * T2 + 20.0 * pll_bw_hz) / 60.0,
        beta * T2 * (fll_bw_hz * T2 + 4.0 * pll_bw_hz) / 8.0, beta * T3 * fll_bw_hz / 6.0,
        T3 * (3.0 * fll_bw_hz * beta2 * T2 + 20.0 * pll_bw_hz * beta2 + 20.0 * dll_bw_hz) / 60.0,
        T2 * dll_bw_hz / 2.0,                                //
        0.0, 0.0, 0.0, T2 * dll_bw_hz / 2.0, T * dll_bw_hz;  //
}

//* === update_measurement_noise_covariance ===
void TrackingKF::update_measurement_noise_covariance() {
    // measurement noise covariance
    R << pll_variance(CN0, T), 0.0, 0.0,  //
        0.0, fll_variance(CN0, T), 0.0,   //
        0.0, 0.0, dll_variance(CN0, T, correlator_spacing);
}

}  // namespace sturdivant_sdr