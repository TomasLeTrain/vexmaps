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

extern "C" void asm_pdfApproximation(float* res,
                                     float* x,
                                     uint32_t n,
                                     float VC0,
                                     float VC1,
                                     float VC2,
                                     float VC3);

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
