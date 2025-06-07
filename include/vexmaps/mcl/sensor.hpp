#pragma once

#include "units/Angle.hpp"
#include "vexmaps/mcl/point.hpp"

namespace vexmaps {
class Sensor {
  public:
    bool exit;
    virtual void update(Angle angle) = 0;
    virtual Point getExpected() = 0;
    virtual float evaluate(const Point& point) = 0;
    virtual bool hasAvailableReading() = 0;

    virtual void disable() = 0;
    virtual void enable() = 0;
    virtual bool getEnabled() = 0;

    virtual ~Sensor() = default;
};
} // namespace vexmaps
