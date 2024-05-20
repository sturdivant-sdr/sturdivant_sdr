/**
|===================================== gps_l1ca_ephemeris.hpp =====================================|
|                                                                                                  |
|   Property of Daniel Sturdivant. Unauthorized copying of this file via any medium is would be    |
|   super sad and unfortunate for me. Proprietary and confidential.                                |
|                                                                                                  |
|--------------------------------------------------------------------------------------------------|
|                                                                                                  |
|   @file     include/adapters/gps_l1ca_ephemeris.hpp                                              |
|   @brief    NAV message decoder for the GPS L1 CA structure.                                     |
|   @author   Daniel Sturdivant <sturdivant20@gmail.com>                                           |
|   @date     April 2024                                                                           |
|   @ref      IS-GPS-200N
|                                                                                                  |
|==================================================================================================|
*/

#ifndef STURDIVANT_SDR_GPS_L1CA_EPHEMERIS_HPP
#define STURDIVANT_SDR_GPS_L1CA_EPHEMERIS_HPP

#include <Eigen/Dense>
#include <cstdint>

#include "constellations/gps/gps_constants.hpp"

namespace sturdivant_sdr {

//* === STRUCTURES ===
//* === Ephemerides ===
struct GpsStatus {
    bool anti_spoof_flag{false};
    bool alert_flag{false};
    bool integrity_status_flag{false};
    bool fit_interval_alert_flag{true};
    uint8_t l2_flag{2u};
    uint8_t URA{0u};
    uint8_t health{0u};
    uint8_t AODO{0u};
    uint16_t tlm_message{0u};
    uint16_t gps_week{0u};
    double TOW{0.0};
};

struct GpsClock {
    //* === subframe 1 ===
    double T_GD{0.0};  // Estimated group delay differential
    double IODC{0.0};  // Issue of data, clock
    double t_oc{0.0};  // clock data reference time
    double a_f2{0.0};  // polynomial clock coefficients
    double a_f1{0.0};
    double a_f0{0.0};
};

struct GpsEphemerides {
    //* === subframe 2 & 3 (ephemeris) ===
    double M_0{0.0};        // mean anomaly at reference time
    double delta_n{0.0};    // mean motion difference from computed value
    double e{0.0};          // eccentricity
    double sqrt_A{0.0};     // square root of the semi major axis
    double OMEGA_0{0.0};    // longitude of ascending node of orbital plane at weekly epoch
    double i_0{0.0};        // inclination angle at reference time
    double omega{0.0};      // argument of perigee
    double OMEGA_DOT{0.0};  // rate of right ascension
    double IDOT{0.0};       // rate of inclination angle
    double C_uc{0.0};  // amplitude of the cos-harmonic correction term to the argument of latitude
    double C_us{0.0};  // amplitude of the sin-harmonic correction term to the argument of latitude
    double C_rc{0.0};  // amplitude of the cos-harmonic term to the orbit radius
    double C_rs{0.0};  // amplitude of the sin-harmonic term to the orbit radius
    double C_ic{0.0};  // amplitude of the cos-harmonic term to the angle of inclination
    double C_is{0.0};  // amplitude of the sin-harmonic term to the angle of inclination
    double t_oe{0.0};  // reference time, ephemeris
    double IODE{0.0};  // issue of data, ephemeris
};

struct GpsAtmosphere {
    //* === subframe 4, page 18 (ionospheric data) ===
    double alpha_0{0.0};  // polynomial coefficients for the amplitude of the vertical delay
    double alpha_1{0.0};
    double alpha_2{0.0};
    double alpha_3{0.0};
    double beta_0{0.0};  // polynomial coefficients for the period of the model
    double beta_1{0.0};
    double beta_2{0.0};
    double beta_3{0.0};
};

//* === CLASS ===
class GpsEphemeris {
  public:
    //* === GpsL1caEphemeris ===
    /// @brief constructor
    GpsEphemeris();

    //* === ~GpsL1caEphemeris ===
    /// @brief destructor
    ~GpsEphemeris();

    //* === check_time ===
    /// @brief check for any beginning or end of week crossovers
    /// @param t time to validate
    /// @returns validated time
    double check_time(double t);

    //* === calc_sv_states ===
    /// @brief determines the GPS position, velocity, acceleration, and clock corrections from the
    ///        ephemeris data
    /// @param gps_t GPS signal transmit time [s]
    void calc_sv_states(double &gps_t, bool calc_accel = false);

    // //* === correct_pseudorange ===
    // /// @brief correct estimated pseudorange with ephemeris clock parameters
    // void correct_pseudorange(double &t_psr);

    //* === get_sv_pos ===
    /// @brief return estimated satellite position
    /// @param pos vector to place satellite position
    void get_sv_pos(Eigen::Vector3d &pos);
    Eigen::Vector3d get_sv_pos();

    //* === get_sv_vel ===
    /// @brief return estimated satellite velocity
    /// @param vel vector to place satellite velocity
    void get_sv_vel(Eigen::Vector3d &vel);
    Eigen::Vector3d get_sv_vel();

    //* === get_sv_acc ===
    /// @brief return estimated satellite acceleration
    /// @param acc vector to place satellite acceleration
    void get_sv_acc(Eigen::Vector3d &acc);
    Eigen::Vector3d get_sv_acc();

    //* === ionosphere_model ===
    /// @brief klobuchar model defined in IS-GPS-200N
    /// @param az       azimuth angle between user and satellite [rad]
    /// @param el       elevation angle between user and satellite [rad]
    /// @param lat      user geodetic latitude [rad]
    /// @param lon      user geodetic longitude [rad]
    /// @param gps_t    receiver computed system time
    /// @return         ionospheric error
    double ionosphere_model(double &az, double &el, double &lat, double &lon, double &gps_t);

    //   protected:
    GpsClock clk;  // GPS clock correction terms
    double delta_t_sv;
    double delta_tdot_sv;
    double delta_tddot_sv;

    GpsEphemerides eph;  // GPS satellite ephemeris terms
    Eigen::Vector3d sv_pos;
    Eigen::Vector3d sv_vel;
    Eigen::Vector3d sv_acc;

    GpsStatus sts;  // GPS status flags

    GpsAtmosphere atm;  // GPS atmospheric correction terms

  private:
};

}  // namespace sturdivant_sdr

#endif