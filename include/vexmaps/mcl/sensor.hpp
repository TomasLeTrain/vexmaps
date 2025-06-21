#pragma once

#include "units/Angle.hpp"
#include "vexmaps/mcl/point.hpp"
#include <arm_neon.h>

namespace vexmaps {
class Sensor {
  public:
    virtual void update(Angle angle) = 0;
    virtual std::optional<Point> getExpected() = 0;
    virtual bool hasAvailableReading() = 0;

    virtual inline float evaluate(const Point& point) = 0;
    virtual inline float evaluate(Length x, Length y) = 0;
    virtual inline float32x4_t Vevaluate(float32x4_t x, float32x4_t y) = 0;
    /**
     * @brief Evaluates two pairs of vectors of particles
     */
    virtual inline void Vevaluate2(float32x4_t x1,
                                          float32x4_t y1,
                                          float32x4_t x2,
                                          float32x4_t y2,
                                          float32x4_t* res1,
                                          float32x4_t* res2) = 0;

    virtual void disable() = 0;
    virtual void enable() = 0;
    virtual bool getEnabled() = 0;

    /**
     * @brief Enabled if Vevaluate is implemented
     */
    virtual bool getVectorized() = 0;
    /**
     * @brief Enabled if Vevaluate2 is implemented
     */
    virtual bool getVectorized2() = 0;

    virtual ~Sensor() = default;
};
} // namespace vexmaps
