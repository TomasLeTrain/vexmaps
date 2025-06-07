#pragma once

#include "units/units.hpp"
#include "vexmaps/mcl/point.hpp"
#include "vexmath/entropy.hpp"
#include <arm_neon.h>
#include <cmath>
#include <random>

namespace vexmaps {
// definition of useful constants
const Length wall_length = 1.78308_m;

inline RobotEntropy<uint32_t> robot_rng;

inline std::ranlux24_base rng(robot_rng());

// credit to Alex Dickens for this implementation
inline float cheapNormalDistribution(const float x) {
    // Coefficients for the rational approximation
    const float a = 0.3989422804014337; //  1 / sqrt(2*pi)
    const float e = 0.59422804014337; // ??

    // Compute the approximate normal PDF using a rational polynomial
    // PDF(x) = (1/sqrt(2pi)) * e^(-(x^2)/2)
    // = 1 / (e^((x^2)/2) * sqrt(2pi))
    // a = 1/sqrt(2pi) --> = a / e^((x^2)/2)
    // e^((x^2)/2)  approximately equals 1 + 0.59...x^4
    //
    // PDF = a / (1 + 0.59...x^4)
    const float pdfApprox = a / (1.0 + e * x * x * x * x);

    return pdfApprox;
}

inline float newCheapNormalDistribution(const float x) {
    // Coefficients for the rational approximation
    const float sqrt2pi = 2.50662827463; // sqrt(2pi)
    const float C0 = 1.0 * sqrt2pi;
    const float C2 = 0.4258 * sqrt2pi;
    const float C4 = 0.258 * sqrt2pi;

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
    const float_t x2 = x * x;
    const float pdfApprox = 1/(C0 + x2 * (C2 + C4 * x2));

    return pdfApprox;
}

inline void VnewCheapNormalDistribution(const float32x4_t x,
                                        float32x4_t* result) {
    // Approximation of the standard normal PDF
    const float32x4_t Vsqrt2pi = vmovq_n_f32(2.50662827463); // sqrt(2pi)

    const float32x4_t VC0 = vmovq_n_f32(1.0) * Vsqrt2pi;
    const float32x4_t VC2 = vmovq_n_f32(0.4258) * Vsqrt2pi;
    const float32x4_t VC4 = vmovq_n_f32(0.258) * Vsqrt2pi;

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
    const float32x4_t x2 = x * x;
    *result = vrecpeq_f32(VC0 + x2 * (VC2 + VC4 * x2));
}

} // namespace vexmaps
