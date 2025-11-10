#pragma once

#include "arm_neon.h"
#include <bit>
#include <cstdint>
#include <cstring>
#include <iostream>

namespace vexmaps {
extern "C" void asm_pdfApproximation(float* res,
                                     float* x,
                                     uint32_t n,
                                     float VC0,
                                     float VC1,
                                     float VC2,
                                     float VC3);

/**
 * @brief Performs res += normalDistributionPDF(x).
 *
 * @param x the list of values to be run through the normal distribution.
 * @param res the list which the pdf values get added to.
 * @param n length of the list. Must be multiple of 24 or have extra unused
 * space.
 */
inline void VNormalDistributionPDF(float* res,
                                   float* x,
                                   uint32_t n,
                                   double mean = 0.0,
                                   double std_dev = 1.0,
                                   double multiplier = 1.0) {
    constexpr double sqrt2pi = 2.50662827463; // sqrt(2pi)
    const float C0 = (1.0 * sqrt2pi * std_dev) / multiplier;
    const float C2 = 0.4258 * sqrt2pi / (std_dev * multiplier);
    const float C4 =
      (0.258 * sqrt2pi) / (std_dev * std_dev * std_dev * multiplier);

    asm_pdfApproximation(res, x, n, -mean, C0, C2, C4);

    // clang-format on
}

/**
 * @brief Performs res += exponentialDistributionPDF(x).
 *
 * @param x the list of values to be run through the normal distribution.
 * @param res the list which the pdf values get added to.
 * @param n length of the list. Must be multiple of 24 or have extra unused
 * space.
 */
inline void VNexpDistributionPDF(float* res,
                                 float* x,
                                 uint32_t n,
                                 double exp_l = 1.0,
                                 double multiplier = 1.0) {
    const float C0 = 0.877896649672 / exp_l;
    const float C1 = 0.68318558894 / exp_l;
    const float C2 = 0.396549717716 * exp_l;
    const float C3 = 0.0532744591926 * exp_l * exp_l * exp_l;

    asm_pdfApproximation(res, x, n, C0, C1, C2, C3);
}

} // namespace vexmaps
