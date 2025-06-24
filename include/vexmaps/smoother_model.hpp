#pragma once

#include "units/Pose.hpp"
#include "vexmaps/localization_model.hpp"

namespace vexmaps {
class SmootherModel : public LocalizationModel {
  private:
    units::Pose last_pose = units::Pose();
    units::Pose pose;
    Length distance_traveled = 0_m;
    Time latest_delta_time;
    Time latest_timestamp;
    // relatively quick so that we can get the latest updates as fast as
    // possible
    Time task_delta_time = 5_msec;

  public:
    SmootherModel() {}

    void init() override {
        latest_timestamp = from_msec(pros::millis());
    }

    void update() override {
        Time current_timestamp = from_msec(pros::millis());
        // do processing here

        latest_delta_time = current_timestamp - latest_timestamp;
        latest_timestamp = current_timestamp;
    }

    void setPose(units::Pose new_pose) override {
        pose = new_pose;
        // should be zero instead?
        last_pose = new_pose;
    }

    units::Pose getPose() override {
        return pose;
    }

    units::Pose getLastPose() override {
        return last_pose;
    }

    units::Pose getGlobalPoseDelta() override {

    }

    units::Pose getLocalPoseDelta() override {

    }

    std::optional<float> getConfidence() override {
        return std::nullopt;
    }

    Length getDistanceTraveled() override {}

    Time getTaskDeltaTime() override {
        return task_delta_time;
    }

    Time getLatestUpdateTimestamp() override {
        return latest_timestamp;
    }

    // TODO: make this part of LocalizationModel implement
    Time getLatestDeltaTime() {
        return latest_delta_time;
    }

    ~SmootherModel() override = default;
};
} // namespace vexmaps
