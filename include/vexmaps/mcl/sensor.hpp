#pragma once

#include "units/Angle.hpp"
#include "vexmaps/mcl/point.hpp"
#include <arm_neon.h>

namespace vexmaps {
class Sensor {
  public:
    virtual void update(Angle angle) = 0;
    virtual Point getExpected() = 0;
    virtual bool hasAvailableReading() = 0;

    virtual inline float evaluate(const Point& point) = 0;
    virtual inline float32x4_t Vevaluate(float32x4x2_t point) = 0;

    virtual void disable() = 0;
    virtual void enable() = 0;
    virtual bool getEnabled() = 0;

    virtual bool getVectorized() = 0;

    virtual ~Sensor() = default;
};
} // namespace vexmaps
