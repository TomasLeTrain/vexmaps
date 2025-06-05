#pragma once

#include "units/Pose.hpp"
#include "units/units.hpp"

namespace vexmaps {
class LocalizationModel {
  public:
    LocalizationModel() {}

    /**
     * @brief Should be called once to initialize the model
     */
    virtual void init() = 0;
    /**
     * @brief Updates pose estimate. Should be called frequently
     */
    virtual void update() = 0;
    virtual void setPose(units::Pose new_pose) = 0;

    virtual units::Pose getPose() = 0;

    /**
     * @brief gets the previous available pose, if it exists
     */
    virtual std::optional<units::Pose> getLastPose() = 0;
    /**
     * @brief Get latest global pose delta, if it exists
     */
    virtual std::optional<units::Pose> getGlobalPoseDelta() = 0;
    /**
     * @brief Get latest local pose delta, if it exists
     */
    virtual std::optional<units::Pose> getLocalPoseDelta() = 0;

    /**
     * @brief returns a value representing the confidence of the current pose
     * estimate (if the model supports it). returns nullopt if not supported.
     *
     * @return the confidence on the estimate (if available)
     */
    virtual std::optional<float> getConfidence() = 0;

    /**
     * @brief Total distance the robot has traveled
     *
     * @return distance traveled by the robot
     */
    virtual Length getDistanceTraveled() = 0;

    virtual Time getTaskDeltaTime() = 0;

    virtual Time getLatestUpdateTimestamp() = 0;

    virtual ~LocalizationModel() = default;
};
}; // namespace vexmaps
