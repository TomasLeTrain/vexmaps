#pragma once

#include "units/Pose.hpp"
#include "units/units.hpp"

namespace vexmaps {
class LocalizationModel {
  public:
    LocalizationModel() {}

    /**
     * @brief Expected time between updates.
     */
    Time deltaTime;

    /**
     * @brief Should be called once to initialize the model
     */
    virtual void init() = 0;
    /**
     * @brief Updates pose estimate. Should be called frequently
     */
    virtual void update() = 0;
    virtual units::Pose getPose() = 0;
    virtual void setPose(units::Pose new_pose) = 0;

    /**
     * @brief returns a value representing the confidence of the current pose
     * estimate (if the model supports it). returns nullopt if not supported.
     *
     * @return the confidence on the estimate (if available)
     */
    virtual std::optional<float> getConfidence() = 0;

    virtual ~LocalizationModel() = default;
};
}; // namespace vexmaps
