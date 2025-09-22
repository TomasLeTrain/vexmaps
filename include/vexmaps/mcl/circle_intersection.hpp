#pragma once

#include "arm_neon.h"
#include <bit>
#include <cstdint>
#include <cstring>
#include <iostream>

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
 * @brief Calculates projection of the vector from (x,y) to (ox,oy) onto the
 * vector (vx,vy). If it doesn't exist returns a big value.
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
extern "C" void circleCenterIntersection(float* res,
                                         float* x,
                                         float* y,

                                         uint32_t len,

                                         float vx,
                                         float vy,

                                         float ox,
                                         float oy,
                                         float r);

extern "C" void asm_raySegmentDistance(float* res,
                                       float* x,
                                       float* y,

                                       uint32_t len,

                                       float ra,
                                       float sa,

                                       float sb,
                                       float rb,

                                       float rc,
                                       float sc);

/**
 * @brief Calculates distance of a ray with origin at (x,y) with angle
 * represented by a unit vector (vx,vy) which intersects a line segment with
 * endpoints (x1,y1) and (x2,y2).
 *
 * @param res result array
 * @param x x component array
 * @param y y component array
 * @param len number of points to test
 * @param vx x component of v vector
 * @param vy y omponent of v vector
 * @param x1 x component of first endpoint
 * @param y1 y component of first endpoint
 * @param x2 x component of second endpoint
 * @param y2 y component of second endpoint
 */
inline void raySegmentDistance(float* res,
                               float* x,
                               float* y,
                               uint32_t len,
                               float vx,
                               float vy,
                               float x1,
                               float y1,
                               float x2,
                               float y2) {
    // precalculate some values then call the assembly implementation
    float dx = x2 - x1;
    float dy = y2 - y1;
    float dd = vx * dy - vy * dx;
    if (dd != 0) {
        // tx = x - x1
        // ty = y - y1
        //
        // r = (ty * dx - tx * dy) / dd
        // r = ((y - y1) * dx - (x - x1) * dy) / dd
        // r = (y - y1) * dx / dd - (x - x1) * dy / dd
        // r = y * dx / dd - y1 * dx / dd - x * dy / dd - x1 * dy / dd
        // r = (y * dx - x * dy) / dd - (y1 * dx - x1 * dy) / dd
        // r = y * (dx / dd) - x * (dy / dd) - c
        // r = a * y - b * x - c
        // where
        // a = dx / dd
        // b = dy / dd
        // c = (y1 * dx - x1 * dy) / dd
        //
        // using the same logic:
        // s = a * y - b * x - c
        // a = vx / dd
        // b = vy / dd
        // c = (y1 * vx - x1 * vy) / dd

        float ra = dx / dd;
        float rb = dy / dd;
        float rc = (y1 * dx - x1 * dy) / dd;

        float sa = vx / dd;
        float sb = vy / dd;
        float sc = (y1 * vx - x1 * vy) / dd;

        asm_raySegmentDistance(res, x, y, len, ra, sa, rb, sb, rc, sc);
    } else {
        // sets all res to 1.3979697 * 10^10
        memset(res, 0x50, len * sizeof(float));
    }
}

// routine that can get reused by exponential / normal distribution.
// len should either be divisible by two or be large enough to fit extra numbers
inline void asm_pdfApproximation(float* x,
                                 float* res,
                                 uint32_t n,
                                 float32x4_t VC0,
                                 float32x4_t VC1,
                                 float32x4_t VC2,
                                 float32x4_t VC3) {
    // main logic loop
    for (int i = 0; i < n; i += 24) {
		// std::cout << "bruh " << i <<  " " << *x << " " << *res << std::endl;
        // clang-format off
        asm volatile(
                "vldmia %m[x], {q0-q5}\n\t"
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
                // "vadd.f32 q0, q0, q6  \n\t"
                // "vadd.f32 q1, q1, q7  \n\t"
                // "vadd.f32 q2, q2, q8  \n\t"
                // "vadd.f32 q3, q3, q9  \n\t"
                // "vadd.f32 q4, q4, q10 \n\t"
                // "vadd.f32 q5, q5, q11 \n\t"

                // store to res
                "vstmia %m[res], {q0-q5}\n\t"
				:
                : [x] "Um"(x), [res] "Um"(res), [C0] "w"(VC0), [C1] "w"(VC1), [C2] "w"(VC2), [C3] "w"(VC3)
                // uses 12 registers
                : "d0","d1",   "d2","d3",   "d4","d5",   "d6","d7",   "d9","d9",
                  "d10","d11", "d12","d13", "d14","d15", "d16","d17", "d18","d19",
                  "d20","d21", "d22","d23"
                );

		x += 24;
		res += 24;

		// std::cout << "bruh2 " << i <<  " " << *x << " " << *res << std::endl;
    }
    // clang-format on
}

/**
 * @brief Performs res += normalDistributionPDF(x).
 *
 * @param x the list of values to be run through the normal distribution.
 * @param res the list which the pdf values get added to.
 * @param n length of the list. Must be multiple of 24 or have extra unused
 * space.
 */
inline void VNormalDistributionPDF(float* x,
                                   float* res,
                                   uint32_t n,
                                   double mean = 0.0,
                                   double std_dev = 1.0,
                                   double multiplier = 1.0) {
    double sqrt2pi = 2.50662827463; // sqrt(2pi)
    float C0 = (1.0 * sqrt2pi * std_dev) / multiplier;
    float C2 = 0.4258 * sqrt2pi / (std_dev * multiplier);
    float C4 = (0.258 * sqrt2pi) / (std_dev * std_dev * std_dev * multiplier);

    // uses up 4 registers, leaving 12 to be used for processing
    float32x4_t VC0 = vdupq_n_f32(C0), VC2 = vdupq_n_f32(C2),
                VC4 = vdupq_n_f32(C4), VMEAN = vdupq_n_f32(-mean);

    asm_pdfApproximation(x, res, n, VMEAN, VC0, VC2, VC4);

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
inline void VNexpDistributionPDF(float* x,
                                 float* res,
                                 uint32_t n,
                                 double exp_l = 1.0,
                                 double multiplier = 1.0) {
    float C0 = 0.877896649672 / exp_l;
    float C1 = 0.68318558894 / exp_l;
    float C2 = 0.396549717716 * exp_l;
    float C3 = 0.0532744591926 * exp_l * exp_l * exp_l;

    // uses up 4 registers, leaving 12 to be used for processing
    float32x4_t VC0 = vdupq_n_f32(C0), VC1 = vdupq_n_f32(C1),
                VC2 = vdupq_n_f32(C2), VC3 = vdupq_n_f32(C3);

    asm_pdfApproximation(x, res, n, VC0, VC1, VC2, VC3);
}

} // namespace vexmaps
