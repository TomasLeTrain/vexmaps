#pragma once

#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"

namespace vexmaps {

struct Confidences {
	bool know_x;
	bool know_y;
	bool know_theta;
};

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
     * @brief gets the previous available pose
     */
    virtual units::Pose getLastPose() = 0;
    /**
     * @brief Get latest global pose delta
     */
    virtual units::Pose getGlobalPoseDelta() = 0;
    /**
     * @brief Get latest local pose delta
     */
    virtual units::Pose getLocalPoseDelta() = 0;

    /**
     * @brief returns a value representing the confidence of the current pose
     * estimate (if the model supports it). returns nullopt if not supported.
     *
     * @return the confidence on the estimate (if available)
     */
    virtual std::optional<Confidences> getConfidence() = 0;

    /**
     * @brief Total distance the robot has traveled
     *
     * @return distance traveled by the robot
     */
    virtual Length getDistanceTraveled() = 0;

    virtual Time getTaskDeltaTime() = 0;

    virtual Time getLatestUpdateTimestamp() = 0;

    // returns a signed distance traveled from the start of tracking
    virtual Length getForwardTravel() = 0;

	// returns local velocity vector relative to the robot
    virtual units::V2Velocity getLocalVelocityVector() = 0;

    // returns the latest angular velocity
    virtual AngularVelocity getAngularVelocity() = 0;

    virtual ~LocalizationModel() = default;
};

// makes a task to automatically run a localizationModel
inline pros::Task createLocalizationTask(LocalizationModel* model) {
    pros::Task task { [model] {
        while (model != nullptr) {
            uint32_t current_time = pros::millis();
            model->update();
            pros::c::task_delay_until(&current_time,
                                      to_msec(model->getTaskDeltaTime()));
        }
    } };
    return task;
}

inline units::Pose localToGlobalDelta(units::Pose local_delta,
                                      Angle pose_angle) {
    // clang-format off
	return units::Pose(
			local_delta.x * units::cos(pose_angle) - local_delta.y * units::sin(pose_angle),
			local_delta.x * units::sin(pose_angle) + local_delta.y * units::cos(pose_angle),
			local_delta.orientation);
    // clang-format on
}

inline units::Pose globalToLocalDelta(units::Pose global_delta,
                                      Angle pose_angle) {
    return localToGlobalDelta(global_delta, -pose_angle);
}

}; // namespace vexmaps
