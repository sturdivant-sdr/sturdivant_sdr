/**
|=================================== tracking_discriminators.hpp ==================================|
|                                                                                                  |
|   Property of Daniel Sturdivant. Unauthorized copying of this file via any medium is would be    |
|   super sad and unfortunate for me. Proprietary and confidential.                                |
|                                                                                                  |
|--------------------------------------------------------------------------------------------------|
|                                                                                                  |
|   @file     src/sdr/tracking_discriminators.hpp                                                  |
|   @brief    GNSS signal tracking discriminator functions.                                        |
|   @author   Daniel Sturdivant <sturdivant20@gmail.com>                                           |
|   @date     April 2024                                                                           |
|   @ref      "Understanding GPS/GNSS Principles and Applications 3rd Ed." - Kaplan and Hegarty    |
|   @ref      "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems"        |
|               2nd Ed. - Groves                                                                   |
|                                                                                                  |
|==================================================================================================|
*/

#ifndef STURDIVANT_SDR_TRACKING_DISCRIMINATORS_HPP
#define STURDIVANT_SDR_TRACKING_DISCRIMINATORS_HPP

#include "core/param/structures.hpp"

namespace sturdivant_sdr {

//* === dll_nceml_normalized ===
/// @brief DLL noncoherent early minus late envelope discriminator
/// @param IE   in-phase, early correlator
/// @param QE   quadrature, early correlator
/// @param IL   in-phase, late correlator
/// @param QL   quadrature, late correlator
/// @returns DLL discriminator
double dll_nceml_normalized(double &IE, double &QE, double &IL, double &QL);
double dll_nceml_normalized(Correlators &corr);

//* === dll_cdp_normalized ===
/// @brief DLL coherent dot product discriminator (only use when phase locked)
/// @param IE   in-phase, early correlator
/// @param IP   in-phase, prompt correlator
/// @param IL   in-phase, late correlator
/// @returns DLL discriminator
double dll_cdp_normalized(double &IE, double &IP, double &IL);
double dll_cdp_normalized(Correlators &corr);

//* === fll_atan2_normalized ===
/// @brief FLL four quadrant arctan discriminator
/// @param ip1  first half-integration period in-phase, prompt correlator
/// @param qp1  first half-integration period quadrature, prompt correlator
/// @param ip2  second half-integration period in-phase, prompt correlator
/// @param qp2  second half-integration period quadrature, prompt correlator
/// @param T    integration period [s]
/// @returns FLL discriminator
double fll_atan2_normalized(double &ip1, double &ip2, double &qp1, double &qp2, double &T);
double fll_atan2_normalized(Correlators &corr, double &T);

//* === fll_ddcp_normalized ===
/// @brief FLL decision directed cross product discriminator
/// @param ip1  first half-integration period in-phase, prompt correlator
/// @param qp1  first half-integration period quadrature, prompt correlator
/// @param ip2  second half-integration period in-phase, prompt correlator
/// @param qp2  second half-integration period quadrature, prompt correlator
/// @param T    integration period [s]
/// @returns FLL discriminator
double fll_ddcp_normalized(double &ip1, double &ip2, double &qp1, double &qp2, double &T);
double fll_ddcp_normalized(Correlators &corr, double &T);

//* === pll_atan_normalized ===
/// @brief PLL two quadrant arctan discriminator (Costas)
/// @param IP   in-phase, prompt correlator
/// @param QP   quadrature, prompt correlator
/// @returns PLL discriminator
double pll_atan_normalized(double &IP, double &QP);
double pll_atan_normalized(Correlators &corr);

//* === pll_ddq_normalized ===
/// @brief PLL decision directed Q discriminator (Costas)
/// @param IP   in-phase, prompt correlator
/// @param QP   quadrature, prompt correlator
/// @returns PLL discriminator
double pll_ddq_normalized(double &IP, double &QP);
double pll_ddq_normalized(Correlators &corr);

//* === dll_variance ===
/// @brief calculates the DLL code discriminator variance (independent of bandwidth)
/// @param CN0  carrier-to-noise-density ratio [magnitude]
/// @param T    integration period [s]
/// @returns DLL discriminator variance
double dll_variance(double &CN0, double &T, double &D);

//* === fll_variance ===
/// @brief calculates the FLL discriminator variance (independent of bandwidth)
/// @param CN0  carrier-to-noise-density ratio [magnitude]
/// @param T    integration period [s]
/// @returns FLL discriminator variance
double fll_variance(double &CN0, double &T);

//* === pll_variance ===
/// @brief calculates PLL Costas discriminator variance (independent of bandwidth)
/// @param CN0  carrier-to-noise-density ratio [magnitude]
/// @param T    integration period [s]
/// @returns PLL discriminator variance
double pll_variance(double &CN0, double &T);

}  // namespace sturdivant_sdr

#endif