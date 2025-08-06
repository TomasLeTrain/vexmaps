#pragma once

#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include <arm_neon.h>

namespace vexmaps {
class Sensor {
  public:
    virtual void update(Angle angle) = 0;
    virtual std::optional<units::V2FPosition> getExpected() = 0;
    virtual bool hasAvailableReading() = 0;

    virtual inline float evaluate(const units::V2FPosition& point) = 0;
    virtual inline float evaluate(FLength x, FLength y) = 0;
    virtual inline float32x4_t Vevaluate(float32x4_t x, float32x4_t y) = 0;

    virtual void disable() = 0;
    virtual void enable() = 0;
    virtual bool getEnabled() = 0;

    /**
     * @brief Enabled if Vevaluate is implemented
     */
    virtual bool getVectorized() = 0;

    virtual ~Sensor() = default;
};
} // namespace vexmaps
