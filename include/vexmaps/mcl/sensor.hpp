#pragma once

// defines base class for all sensors to follow
#include "units/Angle.hpp"
#include "units/units.hpp"
#include "vexmaps/mcl/pose.hpp"
#include <optional>

namespace vexmaps {
class Sensor {
  public:
    bool exit;
    virtual void update(Angle angle) = 0;
    virtual Point getExpected() = 0;
    virtual float evaluate(const Point& point) = 0;
    virtual bool hasAvailableReading() = 0;
    virtual ~Sensor() = default;
};
} // namespace vexmaps
