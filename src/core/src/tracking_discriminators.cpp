/**
|=================================== tracking_discriminators.cpp ==================================|
|                                                                                                  |
|   Property of Daniel Sturdivant. Unauthorized copying of this file via any medium is would be    |
|   super sad and unfortunate for me. Proprietary and confidential.                                |
|                                                                                                  |
|--------------------------------------------------------------------------------------------------|
|                                                                                                  |
|   @file     src/sdr/tracking_discriminators.cpp                                                  |
|   @brief    GNSS signal tracking discriminator functions.                                        |
|   @author   Daniel Sturdivant <sturdivant20@gmail.com>                                           |
|   @date     April 2024                                                                           |
|   @ref      "Understanding GPS/GNSS Principles and Applications" 3rd Ed. - Kaplan and Hegarty    |
|   @ref      "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems"        |
|               2nd Ed. - Groves                                                                   |
|                                                                                                  |
|==================================================================================================|
*/

#include "core/algorithms/tracking_discriminators.hpp"

#include <cmath>

#include "core/param/math_constants.hpp"

namespace sturdivant_sdr {

// double sign(double &a) {
//     return ((a > 0.0) - (a < 0.0));
// }

//* === dll_nceml_normalized ===
double dll_nceml_normalized(double &IE, double &QE, double &IL, double &QL) {
    double E = IE * IE + QE * QE;
    double L = IL * IL + QL * QL;
    return 0.5 * (E - L) / (E + L);  // chips
}
double dll_nceml_normalized(Correlators &corr) {
    double E = corr.IE * corr.IE + corr.QE * corr.QE;
    double L = corr.IL * corr.IL + corr.QL * corr.QL;
    return 0.5 * (E - L) / (E + L);  // chips
}

//* === dll_cdp_normalized ===
double dll_cdp_normalized(double &IE, double &IP, double &IL) {
    return 0.25 * (IE - IL) / IP;  // chips
}
double dll_cdp_normalized(Correlators &corr) {
    return 0.25 * (corr.IE - corr.IL) / corr.IP;  // chips
}

//* === fll_atan2_normalized ===
double fll_atan2_normalized(double &ip1, double &ip2, double &qp1, double &qp2, double &T) {
    double x = ip1 * qp2 - ip2 * qp1;
    double d = ip1 * ip2 + qp1 * qp2;
    return std::atan2(x, d) / (TWO_PI * T);  // Hz
}
double fll_atan2_normalized(Correlators &corr, double &T) {
    double x = corr.ip1 * corr.qp2 - corr.ip2 * corr.qp1;
    double d = corr.ip1 * corr.ip2 + corr.qp1 * corr.qp2;
    return std::atan2(x, d) / (TWO_PI * T);  // Hz
}

//* === fll_ddcp_normalized ===
double fll_ddcp_normalized(double &ip1, double &ip2, double &qp1, double &qp2, double &T) {
    double IP = ip1 + ip2;
    double QP = qp1 + qp2;
    double x = ip1 * qp2 - ip2 * qp1;
    double d = ip1 * ip2 + qp1 * qp2;
    return x * std::copysign(1.0, d) / (HALF_PI * (IP * IP + QP * QP) * T);  // Hz
}
double fll_ddcp_normalized(Correlators &corr, double &T) {
    double x = corr.ip1 * corr.qp2 - corr.ip2 * corr.qp1;
    double d = corr.ip1 * corr.ip2 + corr.qp1 * corr.qp2;
    return x * std::copysign(1.0, d) /
           (HALF_PI * (corr.IP * corr.IP + corr.QP * corr.QP) * T);  // Hz
}

//* === pll_atan_normalized ===
double pll_atan_normalized(double &IP, double &QP) {
    return std::atan(QP / IP) / TWO_PI;  // cycles
}
double pll_atan_normalized(Correlators &corr) {
    return std::atan(corr.QP / corr.IP) / TWO_PI;  // cycles
}

//* === pll_ddq_normalized ===
double pll_ddq_normalized(double &IP, double &QP) {
    return QP * std::copysign(1.0, IP) / (TWO_PI * std::sqrt(IP * IP + QP * QP));
}
double pll_ddq_normalized(Correlators &corr) {
    return corr.QP * std::copysign(1.0, corr.IP) /
           (TWO_PI * std::sqrt(corr.IP * corr.IP + corr.QP * corr.QP));
}

//* === dll_variance ===
double dll_variance(double &CN0, double &T, double &D) {
    double tmp = 1.0 / (CN0 * T);
    return 0.25 * D * tmp * (1.0 + tmp);
    // return 0.5 * D * tmp;
}

//* === fll_variance ===
double fll_variance(double &CN0, double &T) {
    double tmp = 1.0 / (CN0 * T);
    return 0.5 * tmp / (PI_SQUARED * T * T) * (1.0 + tmp);
}

//* === pll_variance ===
double pll_variance(double &CN0, double &T) {
    double tmp = 0.5 / (CN0 * T);
    return tmp * (1.0 + tmp);
}

}  // namespace sturdivant_sdr