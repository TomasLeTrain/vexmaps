#pragma once

#include "arm_neon.h"
#include <cstdint>

namespace vexmaps {
/**
 * @brief Calculates the distance from a ray starting at (x,y) to a circle
 * centered at (ox,oy) with radius r. The vector (vx,vy) is meant to be a unit
 * vector representing the direction of the ray.
 *
 * @param res where the results gets stored
 * @param x list of x components to test
 * @param y list of y components to test
 * @param len length of the list
 * @param vx x component of the unit vector of the ray. For a ray with angle
 * theta, this should be cos(theta).
 * @param vy y component of the unit vector of the ray. For a ray with angle
 * theta, this should be sin(theta).
 * @param ox x component of the center of the circle.
 * @param oy y component of the center of the circle.
 * @param r radius of the circle
 */
extern "C" void circleIntersection(float* res,
                                   float* x,
                                   float* y,

                                   uint32_t len,

                                   float vx,
                                   float vy,

                                   float ox,
                                   float oy,
                                   float r);

/**
 * @brief Calculates distance to the center of the circle, and if doesn't touch,
 * it returns a big value.
 *
 * @param res [TODO:parameter]
 * @param x [TODO:parameter]
 * @param y [TODO:parameter]
 * @param len [TODO:parameter]
 * @param vx [TODO:parameter]
 * @param vy [TODO:parameter]
 * @param ox [TODO:parameter]
 * @param oy [TODO:parameter]
 * @param r [TODO:parameter]
 */
extern "C" void circleCenterIntersection(float* res,
                                         float* x,
                                         float* y,

                                         uint32_t len,

                                         float vx,
                                         float vy,

                                         float ox,
                                         float oy,
                                         float r);

/**
 * @brief Performs res += normalDistributionPDF(x).
 *
 * @param x the list of values to be run through the normal distribution.
 * @param res the list which the pdf values get added to.
 * @param n length of the list. Must be multiple of 24 or have extra unused space.
 */
template<double std_dev = 1.0, double multiplier = 1.0>
inline void VNormalDistributionPDF(float* x, float* res, uint32_t n, float mean = 0) {
    constexpr double sqrt2pi = 2.50662827463; // sqrt(2pi)
    constexpr float C0 = (1.0 * sqrt2pi * std_dev) / multiplier;
    constexpr float C2 = 0.4258 * sqrt2pi / (std_dev * multiplier);
    constexpr float C4 =
      (0.258 * sqrt2pi) / (std_dev * std_dev * std_dev * multiplier);

    // uses up 4 registers, leaving 12 to be used for processing
    float32x4_t VC0 = vdupq_n_f32(C0), VC2 = vdupq_n_f32(C2),
                VC4 = vdupq_n_f32(C4), VMEAN = vdupq_n_f32(mean);

    for (int i = 0; i < n; i += 24) {
        // main logic loop

        // clang-format off
        asm volatile(
                "vldmia %m[x]!, {q0-q5}\n\t"
                // q0-q7 are now loaded
                
                // subtract the mean
                "vsub.f32 q0, q0, %q[MEAN]\n\t"
                "vsub.f32 q1, q1, %q[MEAN]\n\t"
                "vsub.f32 q2, q2, %q[MEAN]\n\t"
                "vsub.f32 q3, q3, %q[MEAN]\n\t"
                "vsub.f32 q4, q4, %q[MEAN]\n\t"
                "vsub.f32 q5, q5, %q[MEAN]\n\t"

                // x^2
                "vmul.f32 q0, q0, q0\n\t"
                "vmul.f32 q1, q1, q1\n\t"
                "vmul.f32 q2, q2, q2\n\t"
                "vmul.f32 q3, q3, q3\n\t"
                "vmul.f32 q4, q4, q4\n\t"
                "vmul.f32 q5, q5, q5\n\t"

                // t1 = x^2 * C4
                "vmul.f32 q6,  q0, %q[C4]\n\t"
                "vmul.f32 q7,  q1, %q[C4]\n\t"
                "vmul.f32 q8,  q2, %q[C4]\n\t"
                "vmul.f32 q9,  q3, %q[C4]\n\t"
                "vmul.f32 q10, q4, %q[C4]\n\t"
                "vmul.f32 q11, q5, %q[C4]\n\t"

                // t1 = C2 + t1
                "vadd.f32 q6,  %q[C2], q6  \n\t"
                "vadd.f32 q7,  %q[C2], q7  \n\t"
                "vadd.f32 q8,  %q[C2], q8  \n\t"
                "vadd.f32 q9,  %q[C2], q9  \n\t"
                "vadd.f32 q10, %q[C2], q10 \n\t"
                "vadd.f32 q11, %q[C2], q11 \n\t"

                // t2 = x^2 * t1
                "vmul.f32 q6,  q0, q6  \n\t"
                "vmul.f32 q7,  q1, q7  \n\t"
                "vmul.f32 q8,  q2, q8  \n\t"
                "vmul.f32 q9,  q3, q9  \n\t"
                "vmul.f32 q10, q4, q10 \n\t"
                "vmul.f32 q11, q5, q11 \n\t"

                // t2 = C0 + t2
                "vadd.f32 q0, %q[C0], q6  \n\t"
                "vadd.f32 q1, %q[C0], q7  \n\t"
                "vadd.f32 q2, %q[C0], q8  \n\t"
                "vadd.f32 q3, %q[C0], q9  \n\t"
                "vadd.f32 q4, %q[C0], q10 \n\t"
                "vadd.f32 q5, %q[C0], q11 \n\t"

                // here the q6-q11 register get freed, so we can use them to store res
                "vldmia %m[res], {q6-q11}\n\t"

                // pdf(x) = 1 / t2
                "vrecpe.f32 q0, q0 \n\t"
                "vrecpe.f32 q1, q1 \n\t"
                "vrecpe.f32 q2, q2 \n\t"
                "vrecpe.f32 q3, q3 \n\t"
                "vrecpe.f32 q4, q4 \n\t"
                "vrecpe.f32 q5, q5 \n\t"
                
                // res += pdf(x)
                "vadd.f32 q0, q0, q6  \n\t"
                "vadd.f32 q1, q1, q7  \n\t"
                "vadd.f32 q2, q2, q8  \n\t"
                "vadd.f32 q3, q3, q9  \n\t"
                "vadd.f32 q4, q4, q10 \n\t"
                "vadd.f32 q5, q5, q11 \n\t"

                // store to res
                "vstmia %m[res]!, {q0-q5}\n\t"
                :
                : [x] "Um"(x), [res] "Um"(res), [C0] "w"(VC0), [C2] "w"(VC2), [C4] "w"(VC4), [MEAN] "w"(VMEAN)
                // uses 12 registers
                : "d0","d1","d2","d3","d4","d5","d6","d7","d9","d9",
                "d10","d11","d12","d13","d14","d15","d16","d17","d18","d19",
                "d20","d21","d22","d23"
                );
    }
    // clang-format on
}



/**
 * @brief Performs res += exponentialDistributionPDF(x).
 *
 * @param x the list of values to be run through the normal distribution.
 * @param res the list which the pdf values get added to.
 * @param n length of the list. Must be multiple of 24 or have extra unused space.
 */
template<double exp_l = 1.0, double multipler = 1.0>
inline void VNormalDistributionPDF(float* x, float* res, uint32_t n) {
    constexpr float C0 = 0.877896649672 / exp_l;
    constexpr float C1 = 0.68318558894 / exp_l;
    constexpr float C2 = 0.396549717716 * exp_l;
    constexpr float C3 = 0.0532744591926 * exp_l * exp_l * exp_l;

    // uses up 4 registers, leaving 12 to be used for processing
    float32x4_t VC0 = vdupq_n_f32(C0), VC1 = vdupq_n_f32(C1),
                VC2 = vdupq_n_f32(C2), VC3 = vdupq_n_f32(C3);

    for (int i = 0; i < n; i += 24) {
        // main logic loop

        // clang-format off
        asm volatile(
                "vldmia %m[x]!, {q0-q5}\n\t"
                // q0-q7 are now loaded
                
                // x2 = x + c0
                "vadd.f32 q0, q0, %q[C0]\n\t"
                "vadd.f32 q1, q1, %q[C0]\n\t"
                "vadd.f32 q2, q2, %q[C0]\n\t"
                "vadd.f32 q3, q3, %q[C0]\n\t"
                "vadd.f32 q4, q4, %q[C0]\n\t"
                "vadd.f32 q5, q5, %q[C0]\n\t"

                // x^2 = (x+c0)^2
                "vmul.f32 q0, q0, q0\n\t"
                "vmul.f32 q1, q1, q1\n\t"
                "vmul.f32 q2, q2, q2\n\t"
                "vmul.f32 q3, q3, q3\n\t"
                "vmul.f32 q4, q4, q4\n\t"
                "vmul.f32 q5, q5, q5\n\t"

                // t1 = x^2 * c3
                "vmul.f32 q6,  q0, %q[C3]\n\t"
                "vmul.f32 q7,  q1, %q[C3]\n\t"
                "vmul.f32 q8,  q2, %q[C3]\n\t"
                "vmul.f32 q9,  q3, %q[C3]\n\t"
                "vmul.f32 q10, q4, %q[C3]\n\t"
                "vmul.f32 q11, q5, %q[C3]\n\t"

                // t1 = c2 + t1
                "vadd.f32 q6,  %q[C2], q6  \n\t"
                "vadd.f32 q7,  %q[C2], q7  \n\t"
                "vadd.f32 q8,  %q[C2], q8  \n\t"
                "vadd.f32 q9,  %q[C2], q9  \n\t"
                "vadd.f32 q10, %q[C2], q10 \n\t"
                "vadd.f32 q11, %q[C2], q11 \n\t"

                // t2 = x^2 * t1
                "vmul.f32 q6,  q0, q6  \n\t"
                "vmul.f32 q7,  q1, q7  \n\t"
                "vmul.f32 q8,  q2, q8  \n\t"
                "vmul.f32 q9,  q3, q9  \n\t"
                "vmul.f32 q10, q4, q10 \n\t"
                "vmul.f32 q11, q5, q11 \n\t"

                // t2 = c1 + t2
                "vadd.f32 q0, %q[C1], q6  \n\t"
                "vadd.f32 q1, %q[C1], q7  \n\t"
                "vadd.f32 q2, %q[C1], q8  \n\t"
                "vadd.f32 q3, %q[C1], q9  \n\t"
                "vadd.f32 q4, %q[C1], q10 \n\t"
                "vadd.f32 q5, %q[C1], q11 \n\t"

                // here the q6-q11 register get freed, so we can use them to store res
                "vldmia %m[res], {q6-q11}\n\t"

                // pdf(x) = 1 / t2
                "vrecpe.f32 q0, q0 \n\t"
                "vrecpe.f32 q1, q1 \n\t"
                "vrecpe.f32 q2, q2 \n\t"
                "vrecpe.f32 q3, q3 \n\t"
                "vrecpe.f32 q4, q4 \n\t"
                "vrecpe.f32 q5, q5 \n\t"
                
                // res += pdf(x)
                "vadd.f32 q0, q0, q6  \n\t"
                "vadd.f32 q1, q1, q7  \n\t"
                "vadd.f32 q2, q2, q8  \n\t"
                "vadd.f32 q3, q3, q9  \n\t"
                "vadd.f32 q4, q4, q10 \n\t"
                "vadd.f32 q5, q5, q11 \n\t"

                // store to res
                "vstmia %m[res]!, {q0-q5}\n\t"
                :
                : [x] "Um"(x), [res] "Um"(res), [C0] "w"(VC0), [C1] "w"(VC1), [C2] "w"(VC2), [C3] "w"(VC3)
                // uses 12 registers
                : "d0","d1","d2","d3","d4","d5","d6","d7","d9","d9",
                "d10","d11","d12","d13","d14","d15","d16","d17","d18","d19",
                "d20","d21","d22","d23"
                );
    }
    // clang-format on
}

} // namespace vexmaps
