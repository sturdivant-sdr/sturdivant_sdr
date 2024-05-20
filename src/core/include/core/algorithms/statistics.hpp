/**
|======================================== statistics.hpp ==========================================|
|                                                                                                  |
|   Property of Daniel Sturdivant. Unauthorized copying of this file via any medium is would be    |
|   super sad and unfortunate for me. Proprietary and confidential.                                |
|                                                                                                  |
|--------------------------------------------------------------------------------------------------|
|                                                                                                  |
|   @file     include/core/params/statistics.cpp                                                   |
|   @brief    Defines useful statistics functions.                                                 |
|   @author   Daniel Sturdivant <sturdivant20@gmail.com>                                           |
|   @date     April 2024                                                                           |
|                                                                                                  |
|==================================================================================================|
*/

#ifndef STURDIVANT_SDR_STATISTICS_HPP
#define STURDIVANT_SDR_STATISTICS_HPP

#include <cmath>
#include <vector>

// common statistics functions

//* === mean ===
template <typename T>
T mean(std::vector<T> &in) {
    T m = 0;
    for (T &ii : in) {
        m += ii;
    }
    return (m / static_cast<T>(in.size()));
};
template <typename T>
T mean(T *in, int &len) {
    T m = 0;
    for (int ii = 0; ii < len; ii++) {
        m += in[ii];
    }
    return (m / static_cast<T>(len));
};

//* === variance ===
template <typename T>
T variance(std::vector<T> &in, T &mean) {
    T v = 0, tmp;
    for (T &ii : in) {
        tmp = ii - mean;
        v += (tmp * tmp);
    }
    return (v / static_cast<T>(in.size()));
};
template <typename T>
T variance(T *in, T &mean, int &len) {
    T v = 0, tmp;
    for (int ii = 0; ii < len; ii++) {
        tmp = in[ii] - mean;
        v += (tmp * tmp);
    }
    return (v / static_cast<T>(len));
};

//* === standard deviation ===
template <typename T>
T stdev(std::vector<T> &in, T &mean) {
    return std::sqrt(variance(in, mean));
};
template <typename T>
T stdev(T *in, T &mean, int &len) {
    return std::sqrt(variance(in, mean, len));
};

//* === mean and variance ===
template <typename T>
void mean_and_var(T &mean, T &var, std::vector<T> &in) {
    // may be less stable than two pass algorithm
    T s = 0;
    T s2 = 0;
    T n = static_cast<T>(in.size());
    for (T &ii : in) {
        s += ii;
        s2 += (ii * ii);
    }
    mean = s / n;
    var = (s2 - (s / n)) / n;
}
template <typename T>
void mean_and_var(T &mean, T &var, T *in, int &len) {
    // may be less stable than two pass algorithm
    T s = 0;
    T s2 = 0;
    T n = static_cast<T>(len);
    for (int ii = 0; ii < len; ii++) {
        s += in[ii];
        s2 += (in[ii] * in[ii]);
    }
    mean = s / n;
    var = (s2 - (s / n)) / n;
}

#endif