#pragma once

#include "units/units.hpp"
#include "units/Angle.hpp"
#include <cmath>
#include <random>
#include <numbers>
#include "vexmaps/mcl/pose.h"
#include "math/entropy.h"

namespace vexmaps {
    // definition of useful constants
    const Length wall_length = 1.78308_m;

    // unit circle angle constants
    constexpr double PI = M_PI;
    constexpr double PI_2 = M_PI_2;
    constexpr double THREE_PI_2 = PI + PI_2;
    constexpr double TWO_PI = 2*M_PI;

    inline RobotEntropy<uint32_t> robot_rng;
    
    // random number engine used is ranlux24 
    // fastest of the ones available from the C++ standard (C++20)
    inline std::ranlux24_base rng;

    inline Length distance(const Point & point){
        return units::sqrt(
                (point.x * point.x) + (point.y * point.y)
                );
    }

    // credit to Alex Dickens for this implementation
    inline float cheapNormalDistribution(const float x) {
        // Approximation of the standard normal PDF
        // Coefficients for the rational approximation
        const float a = 0.3989422804014337; //  1 / sqrt(2*pi)
        const float e = 0.59422804014337; // ??

        // Compute the approximate normal PDF using a rational polynomial
        const float pdfApprox = a / (1.0 + e * x * x * x * x);
        // PDF(x) = (1/sqrt(2pi)) * e^(-(x^2)/2)
        // = 1 / (e^((x^2)/2) * sqrt(2pi))
        // a = 1/sqrt(2pi) --> = a / e^((x^2)/2)
        // e^((x^2)/2)  approximately equals 1 + 0.59...x^4 
        //
        // PDF = a / (1 + 0.59...x^4)

        return pdfApprox;
    }
}
