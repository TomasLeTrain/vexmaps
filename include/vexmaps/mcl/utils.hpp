#pragma once

#include "units/Pose.hpp"
#include "units/units.hpp"
#include "vexmath/entropy.hpp"
#include <arm_neon.h>
#include <cmath>
#include <random>

namespace vexmaps {
// definition of useful constants
// our field
// constexpr FLength global_hor_wall_length = (142.1_in) / 2.0;
// constexpr FLength global_ver_wall_length = (141.1_in) / 2.0;

// normal fields
constexpr FLength global_hor_wall_length = (140.45_in) / 2.0;
constexpr FLength global_ver_wall_length = (140.45_in) / 2.0;

// rng used throughout the filter
inline RobotEntropy<uint32_t> robot_rng;

inline std::ranlux24_base rng(robot_rng());

// rotates pose around origin. Does not change orientation
inline units::FPose FrotatePose(const units::FPose& point,
                                const FAngle& angle) {
    // does not use units::sin/cos to avoid cast from Number (double) to float
    const float sina = std::sin(angle.internal());
    const float cosa = std::cos(angle.internal());

    return { point.x * cosa - point.y * sina,
             point.x * sina + point.y * cosa,
             point.orientation + angle };
}

inline units::Pose rotatePose(const units::Pose& point, const Angle& angle) {
    const Number sina = units::sin(angle);
    const Number cosa = units::cos(angle);

    return { point.x * cosa - point.y * sina,
             point.x * sina + point.y * cosa,
             point.orientation + angle };
}

inline float NormalDistributionApproximation(const float x,
                                             const float std_dev = 1.0,
                                             const float multiplier = 1.0) {
    const double sqrt2pi = 2.50662827463; // sqrt(2pi)
    const float C0 = (1.0 * sqrt2pi * std_dev) / multiplier;
    const float C2 = 0.4258 * sqrt2pi / (std_dev * multiplier);
    const float C4 =
      (0.258 * sqrt2pi) / (std_dev * std_dev * std_dev * multiplier);

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
    const float pdfApprox = 1 / (C0 + x2 * (C2 + C4 * x2));

    return pdfApprox;
}

template<double std_dev = 1.0, double multiplier = 1.0>
inline float32x4_t VNormalDistributionApproximation(const float32x4_t x) {
    constexpr double sqrt2pi = 2.50662827463; // sqrt(2pi)
    constexpr float C0 = (1.0 * sqrt2pi * std_dev) / multiplier;
    constexpr float C2 = 0.4258 * sqrt2pi / (std_dev * multiplier);
    constexpr float C4 =
      (0.258 * sqrt2pi) / (std_dev * std_dev * std_dev * multiplier);

    float32x4_t x2 = vmulq_f32(x, x);

    float32x4_t t1 = vdupq_n_f32(C2);
    float32x4_t t2 = vdupq_n_f32(C0);

    // t1 = C2 + x^2 * C4
    t1 = vmlaq_n_f32(t1, x2, C4);

    // t2 = C0 + x^2 * t1
    t2 = vmlaq_f32(t2, x2, t1);

    return vrecpeq_f32(t2);
}

inline float expDistribution(float x, float exp_l) {
    const float c0 = 0.877896649672 / exp_l;
    const float c1 = 0.68318558894 / exp_l;
    const float c2 = 0.396549717716 * exp_l;
    const float c3 = 0.0532744591926 * exp_l * exp_l * exp_l;

    float x2 = x + c0;
    x2 *= x2;
    return 1 / (c1 + x2 * (c2 + c3 * x2));
}

template<double exp_l>
inline float32x4_t VexpDistribution(float32x4_t x) {
    constexpr float c0 = 0.877896649672 / exp_l;
    constexpr float c1 = 0.68318558894 / exp_l;
    constexpr float c2 = 0.396549717716 * exp_l;
    constexpr float c3 = 0.0532744591926 * exp_l * exp_l * exp_l;

    // x2 = x + c0
    float32x4_t x2 = vaddq_f32(x, vdupq_n_f32(c0));
    // x2 = (x+c0)^2
    x2 = vmulq_f32(x2, x2);

    float32x4_t t1 = vdupq_n_f32(c2);
    float32x4_t t2 = vdupq_n_f32(c1);

    t1 = vmlaq_n_f32(t1, x2, c3);
    t2 = vmlaq_f32(t2, x2, t1);

    return vrecpeq_f32(t2);
}

// ensures distribution's integral is always 1
template<double exp_l>
inline float expNormalizationFactor(float v) {
    return 1.0 / (1.0 - std::exp(-v * exp_l));
}

// taylor approximation
// only accurate on an approximate range of (0,2)
template<double exp_l>
inline float expNormalizationFactorApproximation(float v) {
    constexpr float c0 = 1.0 / 12.0;
    v *= exp_l;

    return (1 / v) + (v * c0) + 0.5;
}

} // namespace vexmaps
