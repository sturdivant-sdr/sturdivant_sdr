/**
|===================================== gps_l1ca_ephemeris.cpp =====================================|
|                                                                                                  |
|   Property of Daniel Sturdivant. Unauthorized copying of this file via any medium is would be    |
|   super sad and unfortunate for me. Proprietary and confidential.                                |
|                                                                                                  |
|--------------------------------------------------------------------------------------------------|
|                                                                                                  |
|   @file     src/adapters/gps_l1ca_ephemeris.cpp                                                  |
|   @brief    NAV message decoder for the GPS L1 CA structure.                                     |
|   @author   Daniel Sturdivant <sturdivant20@gmail.com>                                           |
|   @date     April 2024                                                                           |
|                                                                                                  |
|==================================================================================================|
*/

#include "constellations/gps/gps_ephemeris.hpp"

#include <cmath>

#include "core/param/math_constants.hpp"

namespace sturdivant_sdr {

//* === GpsEphemeris ===
GpsEphemeris::GpsEphemeris() {
    sv_pos.resize(3);
    sv_vel.resize(3);
    sv_acc.resize(3);
};

//* === ~GpsEphemeris ===
GpsEphemeris::~GpsEphemeris() {};

//* === check_time ===
double GpsEphemeris::check_time(double t) {
    if (t > HALF_WEEK) {
        t -= WEEK;
    } else if (t < -HALF_WEEK) {
        t += WEEK;
    }
    return t;
};

//* === sv_clock_correction ===
void GpsEphemeris::calc_sv_states(double &gps_t, bool calc_accel) {
    // ANOMALY
    double A = eph.sqrt_A * eph.sqrt_A;              // semi-major axis
    double n_0 = std::sqrt(WGS84_MU / (A * A * A));  // computed mean motion
    double n = n_0 + eph.delta_n;                    // corrected mean motion
    double t_k = check_time(gps_t - eph.t_oe);       // time from ephemeris reference epoch
    double M_k = eph.M_0 + n * t_k;                  // mean anomaly
    M_k = std::fmod(M_k + TWO_GPS_PI, TWO_GPS_PI);

    double E_k = M_k;  // eccentric anomaly
    double dE = 1.0;
    while (dE > 1e-15) {
        dE = (M_k - E_k + eph.e * std::sin(E_k)) / (1.0 - eph.e * std::cos(E_k));
        E_k += dE;
    }
    E_k = std::fmod(E_k + TWO_GPS_PI, TWO_GPS_PI);

    double v_k = 2.0 * std::atan2(std::sqrt((1.0 + eph.e) / (1.0 - eph.e)) * std::tan(0.5 * E_k),
                                  1.0);  // true anomaly

    // CLOCK CORRECTIONS
    double dt = check_time(gps_t - clk.t_oc);
    double cos_E = std::cos(E_k);
    double sin_E = std::sin(E_k);
    double FesqrtA = WGS84_F * eph.e * eph.sqrt_A;
    double den = 1.0 - eph.e * cos_E;

    double delta_t_R = FesqrtA * sin_E;
    delta_t_sv = clk.a_f0 + (clk.a_f1 * dt) + (clk.a_f2 * dt * dt) + delta_t_R;
    delta_tdot_sv = clk.a_f1 + 2.0 * clk.a_f2 * dt + (n * FesqrtA * cos_E / den);
    delta_tddot_sv = 2.0 * clk.a_f2 - (n * n * FesqrtA * sin_E / (den * den));

    // POSITION CALCULATIONS
    double Phi_k = v_k + eph.omega;  // argument of latitude
    Phi_k = std::fmod(Phi_k, TWO_GPS_PI);
    double cos2Phi = std::cos(2.0 * Phi_k);
    double sin2Phi = std::sin(2.0 * Phi_k);

    double delta_u_k = eph.C_us * sin2Phi + eph.C_uc * cos2Phi;  // argument of latitude correction
    double delta_r_k = eph.C_rs * sin2Phi + eph.C_rc * cos2Phi;  // radius correction
    double delta_i_k = eph.C_is * sin2Phi + eph.C_ic * cos2Phi;  // inclination correction

    double u_k = Phi_k + delta_u_k;                     // corrected argument of latitude
    double r_k = A * den + delta_r_k;                   // corrected radius
    double i_k = eph.i_0 + delta_i_k + eph.IDOT * t_k;  // corrected inclination
    double OMEGA_k = eph.OMEGA_0 + (eph.OMEGA_DOT - WGS84_OMEGA_DOT_EARTH) * t_k -
                     WGS84_OMEGA_DOT_EARTH * eph.t_oe;  // corrected longitude of ascending node
    OMEGA_k = std::fmod(OMEGA_k + TWO_GPS_PI, TWO_GPS_PI);
    double cosOmega = std::cos(OMEGA_k);
    double sinOmega = std::sin(OMEGA_k);
    double cosi = std::cos(i_k);
    double sini = std::sin(i_k);
    double cosu = std::cos(u_k);
    double sinu = std::sin(u_k);

    double x_k_prime = r_k * cosu;  // x-position in orbital frame
    double y_k_prime = r_k * sinu;  // y-position in orbital frame

    sv_pos(0) = x_k_prime * cosOmega - y_k_prime * cosi * sinOmega;
    sv_pos(1) = x_k_prime * sinOmega + y_k_prime * cosi * cosOmega;
    sv_pos(2) = y_k_prime * sini;

    // VELOCITY CALCULATIONS
    double Edot_k = n / den;                                        // eccentric anomaly rate
    double vdot_k = Edot_k * std::sqrt(1.0 - eph.e * eph.e) / den;  // true anomaly rate

    // corrected inclination angle rate
    double idot_k = eph.IDOT + 2.0 * vdot_k * (eph.C_is * cos2Phi - eph.C_ic * sin2Phi);
    // corrected argument of latitude rate
    double udot_k = vdot_k * (1.0 + 2.0 * (eph.C_us * cos2Phi - eph.C_uc * sin2Phi));
    // corrected radius rate
    double rdot_k =
        eph.e * A * Edot_k * sin_E + 2.0 * vdot_k * (eph.C_rs * cos2Phi - eph.C_rc * sin2Phi);
    double OMEGAdot_k = eph.OMEGA_DOT - WGS84_OMEGA_DOT_EARTH;  // longitude of ascending node rate

    double xdot_k_prime = rdot_k * cosu - r_k * udot_k * sinu;  // x-velocity in orbital frame
    double ydot_k_prime = rdot_k * sinu + r_k * udot_k * cosu;  // y-velocity in orbital frame

    sv_vel(0) = -(x_k_prime * OMEGAdot_k * sinOmega) + (xdot_k_prime * cosOmega) -
                (ydot_k_prime * sinOmega * cosi) -
                (y_k_prime * (OMEGAdot_k * cosOmega * cosi - idot_k * sinOmega * sini));
    sv_vel(1) = (x_k_prime * OMEGAdot_k * cosOmega) + (xdot_k_prime * sinOmega) +
                (ydot_k_prime * cosOmega * cosi) -
                (y_k_prime * (OMEGAdot_k * sinOmega * cosi + idot_k * cosOmega * sini));
    sv_vel(2) = (ydot_k_prime * sini) + (y_k_prime * idot_k * cosi);

    // ACCELERATION CALCULATIONS
    if (calc_accel) {
        double tmp = WGS84_RE / r_k;
        double r2 = r_k * r_k;
        double r3 = r2 * r_k;
        double F = -1.5 * WGS84_J2 * (WGS84_MU / r2) * (tmp * tmp);
        double OMEGA_DOT_E2 = WGS84_OMEGA_DOT_EARTH * WGS84_OMEGA_DOT_EARTH;
        tmp = sv_pos(2) / r_k;
        tmp *= tmp;
        tmp *= 5.0;

        sv_acc(0) = -WGS84_MU * (sv_pos(0) / r3) + F * ((1.0 - tmp) * (sv_pos(0) / r_k)) +
                    (2.0 * sv_vel(1) * WGS84_OMEGA_DOT_EARTH) + (sv_pos(0) * OMEGA_DOT_E2);
        sv_acc(1) = -WGS84_MU * (sv_pos(1) / r3) + F * ((1.0 - tmp) * (sv_pos(1) / r_k)) -
                    (2.0 * sv_vel(0) * WGS84_OMEGA_DOT_EARTH) + (sv_pos(1) * OMEGA_DOT_E2);
        sv_acc(2) = -WGS84_MU * (sv_pos(2) / r3) + F * ((3.0 - tmp) * (sv_pos(2) / r_k));
    }
}

//* === get_sv_pos ===
void GpsEphemeris::get_sv_pos(Eigen::Vector3d &pos) {
    pos = sv_pos;
}
Eigen::Vector3d GpsEphemeris::get_sv_pos() {
    return sv_pos;
}

//* === get_sv_vel ===
void GpsEphemeris::get_sv_vel(Eigen::Vector3d &vel) {
    vel = sv_vel;
}
Eigen::Vector3d GpsEphemeris::get_sv_vel() {
    return sv_vel;
}

//* === get_sv_acc ===
void GpsEphemeris::get_sv_acc(Eigen::Vector3d &acc) {
    acc = sv_acc;
}
Eigen::Vector3d GpsEphemeris::get_sv_acc() {
    return sv_acc;
}

//! ============================================================================================ !\\

//* === ionosphere_model ===
double GpsEphemeris::ionosphere_model(double &az, double &el, double &lat, double &lon,
                                      double &gps_t) {
    // convert radians to semi-circles
    double A = az / PI;
    double E = el / PI;
    double phi_u = lat / PI;
    double lmb_u = lon / PI;

    // earth's central angle between the user position and the earth projection of ionospheric
    // intersection point [semi-circles]
    double psi = (0.0137 / (E + 0.11)) - 0.022;

    // geodetic latitude of the earth projection of the ionospheric intersection point
    // [semi-circles]
    double phi_i = phi_u + psi * std::cos(A);
    if (phi_i > 0.416) {
        phi_i = 0.416;
    } else if (phi_i < -0.416) {
        phi_i = -0.416;
    }

    // geodetic longitude of the earth projection of the ionospheric intersection point
    // [semi-circles]
    double lmb_i = lmb_u + (psi * std::sin(A) / std::cos(phi_i));

    // geomagnetic latitude of the earth projection of the ionospheric intersection point (mean
    // ionospheric height assumed 350 km) [semi-circles]
    double phi_m = phi_i + (0.064 * std::cos(lmb_i - 1.617));

    // local time [s]
    double t = 4.32e4 * phi_i + gps_t;
    if (t >= 86400.0) {
        t -= 86400.0;
    } else if (t < 0.0) {
        t += 86400.0;
    }

    // obliquity factor [dimensionless]
    double tmp = 0.53 - E;
    double F = 1.0 + (16.0 * (tmp * tmp * tmp));

    // period [s]
    double PER =
        (atm.beta_0 * phi_m) + (atm.beta_1 * phi_m) + (atm.beta_2 * phi_m) + (atm.beta_3 * phi_m);
    if (PER < 72000.0) {
        PER = 72000.0;
    }

    // amplitude [s]
    double AMP = (atm.alpha_0 * phi_m) + (atm.alpha_1 * phi_m) + (atm.alpha_2 * phi_m) +
                 (atm.alpha_3 * phi_m);
    if (AMP < 0.0) {
        AMP = 0.0;
    }

    // ionospheric correction term [s]
    double x = TWO_GPS_PI * (t - 50400.0) / PER;  // [radians]
    double T_iono;
    if (std::abs(x) >= 1.57) {
        T_iono = F * (5e9);
    } else {
        double x2 = x * x;
        T_iono = F * (5e9 + AMP * (1 + (x2 / 2.0) + (x2 * x2 / 24.0)));
    };

    return T_iono;
}

}  // namespace sturdivant_sdr