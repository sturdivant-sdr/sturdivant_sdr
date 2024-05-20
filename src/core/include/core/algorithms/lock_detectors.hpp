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

#ifndef STURDIVANT_SDR_LOCK_DETECTORS_HPP
#define STURDIVANT_SDR_LOCK_DETECTORS_HPP

#include <vector>

namespace sturdivant_sdr {

//* === cn0_m2m4_estimate ===
/// @brief moments method to calculate the carrier-to-noise-density ratio
/// @param IPs  vector of in-phase, prompt correlators
/// @param QPs  vector of quadrature, prompt correlators
/// @param T    integration period [s]
/// @returns carrier-to-noise-density ratio [magnitude]
double cn0_m2m4_estimate(std::vector<double> &IPs, std::vector<double> &QPs, double &T);

//* === cn0_beaulieu_estimate ===
/// @brief Beaulieu's method to calculate the carrier-to-noise-density ratio
/// @param IPs      newest vector of in-phase, prompt correlators
/// @param prev_IPs previous vector of in-phase, prompt correlators
/// @param T        integration period [s]
/// @returns carrier-to-noise-density ratio [magnitude]
double cn0_beaulieu_estimate(std::vector<double> &IPs, std::vector<double> &prev_IPs, double &T);

//* === carrier_lock_detector ===
/// @brief carrier lock determined by the cosine of twice the carrier phase, cos(2p) = NBD/NBP
/// @param IPs  vector of in-phase, prompt correlators
/// @param QPs  vector of quadrature, prompt correlators
/// @returns estimate of cos(2p)
double carrier_lock_detector(std::vector<double> &IPs, std::vector<double> &QPs);

}  // namespace sturdivant_sdr

#endif