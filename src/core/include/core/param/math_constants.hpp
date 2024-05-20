/**
|====================================== math_constants.hpp ========================================|
|                                                                                                  |
|   Property of Daniel Sturdivant. Unauthorized copying of this file via any medium is would be    |
|   super sad and unfortunate for me. Proprietary and confidential.                                |
|                                                                                                  |
|--------------------------------------------------------------------------------------------------|
|                                                                                                  |
|   @file     src/core/param/math_constants.cpp                                                    |
|   @brief    Defines useful math constants and scalings.                                          |
|   @author   Daniel Sturdivant <sturdivant20@gmail.com>                                           |
|   @date     May 2024                                                                             |
|                                                                                                  |
|==================================================================================================|
*/

#ifndef STURDIVANT_SDR_MATH_CONSTANTS_HPP
#define STURDIVANT_SDR_MATH_CONSTANTS_HPP

#include <cmath>
#include <complex>
#include <vector>

namespace sturdivant_sdr {

constexpr double LIGHT_SPEED = 299792458.0;    //! speed of light [m/s]
constexpr double PI = 3.14159265358979323846;  //! pi
constexpr double HALF_PI = PI / 2.0;           //! pi/2
constexpr double TWO_PI = 2.0 * PI;            //! 2*pi
constexpr double PI_SQUARED = PI * PI;         //! pi*pi
constexpr std::complex<double> J(0.0, 1.0);    //! imaginary value

// template <typename T>
// constexpr std::complex<T> J = std::complex<T>(0, 1);

}  // namespace sturdivant_sdr

#endif