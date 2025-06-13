#pragma once

#include "units/units.hpp"
#include "vexmaps/mcl/point.hpp"
#include "vexmath/entropy.hpp"
#include <arm_neon.h>
#include <cmath>
#include <random>

namespace vexmaps {
// definition of useful constants
constexpr Length wall_length = 1.78308_m;

inline RobotEntropy<uint32_t> robot_rng;

inline std::ranlux24_base rng(robot_rng());

inline float NormalDistributionApproximation(const float x) {
    // Coefficients for the rational approximation
    constexpr float sqrt2pi = 2.50662827463; // sqrt(2pi)
    constexpr float C0 = 1.0 * sqrt2pi;
    constexpr float C2 = 0.4258 * sqrt2pi;
    constexpr float C4 = 0.258 * sqrt2pi;

    // Compute the approximate normal PDF using a rational polynomial
    // PDF(x) = (1/sqrt(2pi)) * e^(-(x^2)/2)
    // = 1 / (e^((x^2)/2) * sqrt(2pi))
    // a = sqrt(2pi)
    // = 1 / (e^((x^2)/2) * a)
    // e^((x^2)/2)  approximately equals C0 + C2*x^2 + C4 * x^4
    //
    // PDF = 1 / ((C0 + C2*x^2 + C4 * x^4) * a)
    // PDF = 1 / (a*C0 + a*C2*x^2 + a*C4*x^4)
    // PDF = 1 / (a*C0 + x^2(a*C2 + a*C4*x^2))
    // PDF = 1 / (C0 + x^2(C2 + C4 * x^2))
    const float x2 = x * x;
    const float pdfApprox = 1/(C0 + x2 * (C2 + C4 * x2));

    return pdfApprox;
}

// allows multiplier to be built in and optimized as well
template<double multiplier = 1.0>
inline float32x4_t  VNormalDistributionApproximation(const float32x4_t x) {
    // Approximation of the standard normal PDF
    constexpr double sqrt2pi = 2.50662827463; // sqrt(2pi)
    constexpr float C0 = 1.0 * sqrt2pi / multiplier;
    constexpr float C2 = 0.4258 * sqrt2pi / multiplier;
    constexpr float C4 = 0.258 * sqrt2pi / multiplier;

    // Compute the approximate normal PDF using a rational polynomial
    // PDF(x) = (1/sqrt(2pi)) * e^(-(x^2)/2)
    // = 1 / (e^((x^2)/2) * sqrt(2pi))
    // a = sqrt(2pi)
    // = 1 / (e^((x^2)/2) * a)
    // e^((x^2)/2)  approximately equals C0 + C2*x^2 + C4 * x^4
    //
    // PDF = 1 / ((C0 + C2*x^2 + C4 * x^4) * a)
    // PDF = 1 / (a*C0 + a*C2*x^2 + a*C4*x^4)
    // PDF = 1 / (a*C0 + x^2(a*C2 + a*C4*x^2))
    // PDF = 1 / (C0 + x^2(C2 + C4*x^2))

    float32x4_t x2 = vmulq_f32(x,x);

    float32x4_t t1 = vdupq_n_f32(C2);
    float32x4_t t2 = vdupq_n_f32(C0);

    // t1 = C2 + x^2 * C4
    t1 = vmlaq_n_f32(t1, x2, C4);

    // t2 = C0 + x^2 * t1
    t2 = vmlaq_f32(t2,x2,t1);

    return vrecpeq_f32(t2);
}

} // namespace vexmaps
