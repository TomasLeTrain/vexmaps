#pragma once

#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/units.hpp"

namespace vexmaps {
/**
 * @class Point
 * @brief Simple point class that makes sure x and y data is aligned (important
 * for vectorization)
 *
 */
struct Point {
    Length x = 0.0_m;
    Length y = 0.0_m;

    constexpr Point operator+(const Point& other) const {
        return Point(x + other.x, y + other.y);
    }

    constexpr Point& operator+=(const Point& other) {
        x += other.x;
        y += other.y;
        return (*this);
    }
};

inline units::Pose rotatePose(const units::Pose& point, const Angle& angle) {
    const float sina = units::sin(angle).internal();
    const float cosa = units::cos(angle).internal();

    return { point.x * cosa - point.y * sina,
             point.y * cosa + point.x * sina,
             point.orientation };
}
} // namespace vexmaps
