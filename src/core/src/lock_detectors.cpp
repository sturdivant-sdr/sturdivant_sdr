/**
|======================================= lock_detectors.cpp =======================================|
|                                                                                                  |
|   Property of Daniel Sturdivant. Unauthorized copying of this file via any medium is would be    |
|   super sad and unfortunate for me. Proprietary and confidential.                                |
|                                                                                                  |
|--------------------------------------------------------------------------------------------------|
|                                                                                                  |
|   @file     src/sdr/lock_detectors.cpp                                                           |
|   @brief    Abstract GNSS signal detection functions.                                            |
|   @author   Daniel Sturdivant <sturdivant20@gmail.com>                                           |
|   @date     April 2024                                                                           |
|   @ref      "Understanding GPS/GNSS Principles and Applications 3rd Ed." - Kaplan and Hegarty    |
|                                                                                                  |
|==================================================================================================|
*/

#include "core/algorithms/lock_detectors.hpp"

#include <cmath>
#include <iostream>

#include "core/param/math_constants.hpp"

namespace sturdivant_sdr {

//* === cn0_m2m4_estimate ===
double cn0_m2m4_estimate(std::vector<double> &IPs, std::vector<double> &QPs, double &T) {
    // 2nd and 4th moments
    double m2 = 0.0;
    double m4 = 0.0;
    double N = static_cast<double>(IPs.size());
    double tmp;
    for (int ii = 0; ii < static_cast<int>(IPs.size()); ii++) {
        tmp = IPs[ii] * IPs[ii] + QPs[ii] + QPs[ii];
        m2 += tmp;
        m4 += tmp * tmp;
    }
    m2 /= N;
    m4 /= N;
    double Pd = std::sqrt(2.0 * m2 * m2 - m4);
    double Pn = m2 - Pd;
    return (Pd / Pn) / T;
}

//* === cn0_beaulieu_estimate ===
double cn0_beaulieu_estimate(std::vector<double> &IPs, std::vector<double> &prev_IPs, double &T) {
    double Pd = 0.0;
    double Pn = 0.0;
    double snr = 0.0;
    double tmp;
    for (int ii = 0; ii < static_cast<int>(IPs.size()); ii++) {
        tmp = std::abs(IPs[ii]) - std::abs(prev_IPs[ii]);
        Pn = tmp * tmp;
        Pd = 0.5 * (IPs[ii] * IPs[ii] + prev_IPs[ii] * prev_IPs[ii]);
        snr += Pn / Pd;
    }
    return (1.0 / (snr / static_cast<double>(IPs.size()))) / T;
}

//* === carrier_lock_detector ===
double carrier_lock_detector(std::vector<double> &IPs, std::vector<double> &QPs) {
    double I = 0.0;
    double Q = 0.0;
    for (int ii = 0; ii < static_cast<int>(IPs.size()); ii++) {
        I += IPs[ii];
        Q += QPs[ii];
    }
    double I2 = I * I;
    double Q2 = Q * Q;
    // NBD / NBP
    return (I2 - Q2) / (I2 + Q2);
}

}  // namespace sturdivant_sdr