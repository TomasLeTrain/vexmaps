#pragma once

#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include <arm_neon.h>
#include <cassert>

namespace vexmaps {
class Sensor {
  public:
    virtual void update(Angle angle) = 0;
    virtual std::optional<units::V2FPosition> getExpected() = 0;
    virtual bool hasAvailableReading() = 0;

    virtual float evaluate(FLength x, FLength y) = 0;

    virtual float evaluate(const units::V2FPosition& point) {
        return evaluate(point.x, point.y);
        return evaluate(point.x, point.y);
    }

    virtual void disable() = 0;
    virtual void enable() = 0;
    virtual bool getEnabled() = 0;

    virtual float32x4_t Vevaluate(float32x4_t x, float32x4_t y) {
        // this impl should never get called
        assert((false) &&
               "Vevaluate was called but implementation was never defined!");
        return vdupq_n_f32(1.0);
    }

    /**
     * @brief Computes the PDF for all particles
     *
     * @param curr_weights array where the results get stored
     * @param x pointer to array of x components
     * @param y pointer to array of y components
     * @param tmp_array pointer to array of temporary array
     * @param len number of particles to evaluate
     */
    virtual void evaluate_array(float* curr_weights,
                                FLength* x,
                                FLength* y,
                                float* tmp_array,
                                int len) {
        assert(
          (false) &&
          "evaluate_array was called but implementation was never defined!");
    }

    /**
     * @brief Determines if sensor supports processing entire array
     */
    virtual bool canProcessArray() {
        return false;
    }

    /**
     * @brief Enabled if Vevaluate is implemented
     */
    virtual bool getVectorized() {
        return false;
    }

    virtual ~Sensor() = default;
};
} // namespace vexmaps
