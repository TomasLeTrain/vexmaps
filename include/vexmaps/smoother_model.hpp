#pragma once

#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "vexmaps/localization_model.hpp"

namespace vexmaps {
struct SmootherConfig {
    // for all parameters:
    // 0 = all model
    // 1 = all measurement

    // // determines how much a pose measurement influences the pose estimate
    double alpha_x = 0.015;
    double alpha_y = 0.015;
    double alpha_theta = 0.015;

    // used by pose_delta_measurement to estimate the pose
    double beta_x = 1;
    double beta_y = 1;
    double beta_theta = 1;

    // determines how much a pose delta measurement influences the velocity
    double beta_vx = 1;
    double beta_vy = 1;
    double beta_vtheta = 1;
};

class SmootherModel : public LocalizationModel {
  private:
    Length distance_traveled = 0_m;
    Time latest_delta_time;
    Time latest_timestamp;

    units::Pose global_pose_delta;
    units::Pose local_pose_delta;

    // used by pose delta measurement
    units::Pose last_pose_estimate = units::Pose();
    units::Pose pose_estimate = units::Pose();
    units::VelocityPose velocity_estimate = units::VelocityPose();

    units::Pose last_local_estimate = units::Pose();

    // measures the local pose deltas of the robot
    LocalizationModel* local_delta_model;
    // measures the global pose of the robot
    LocalizationModel* pose_model;

    Time last_local_delta_timestamp = 0_msec;
    Time last_pose_timestamp = 0_msec;

    // relatively quick so that we can get the latest updates as fast as
    // possible
    Time task_delta_time = 8_msec;

    SmootherConfig config;

  public:
    SmootherModel(LocalizationModel* local_delta_model,
                  LocalizationModel* global_pose_model,
                  SmootherConfig config)
        : local_delta_model(local_delta_model),
          pose_model(global_pose_model),
          config(config) {}

    void init() override {
        latest_timestamp = from_msec(pros::millis());
    }

    // TODO: switch to using doubles for this as computations are cheap
    void update() override {
        Time current_timestamp = from_msec(pros::millis());

        latest_delta_time = current_timestamp - latest_timestamp;
        latest_timestamp = current_timestamp;

        // TODO: actually only get the global updates instead of all updates
        // from pf or else its likely bad

        Time delta_time = latest_delta_time;

        Time current_local_delta_timestamp =
          local_delta_model->getLatestUpdateTimestamp();

        Time current_pose_timestamp = pose_model->getLatestUpdateTimestamp();

        units::Pose previous_pose_estimate = pose_estimate;


        // calculate the predictions first using the transition equations:
        units::Pose pose_prediction = units::Pose(
          (pose_estimate + velocity_estimate * delta_time),
          pose_estimate.orientation + velocity_estimate.orientation * delta_time
          );

        units::VelocityPose velocity_prediction = velocity_estimate;

        // allows multiple sensors to affect the final estimate
        pose_estimate = pose_prediction;
        velocity_estimate = velocity_prediction;

        bool applied_local = false;
        bool applied_global = false;

        // update steps get performed independently

        // only correct velocity if we have a new pose measurement
        if (current_local_delta_timestamp != last_local_delta_timestamp) {

            Time local_delta_measurement_delta_time =
              current_local_delta_timestamp - last_local_delta_timestamp;

            units::Pose pose_delta_measurement =
              local_delta_model->getGlobalPoseDelta();
            

            velocity_estimate.x =
              velocity_estimate.x +
              config.beta_vx *
                (pose_delta_measurement.x / local_delta_measurement_delta_time -
                 velocity_estimate.x);

            velocity_estimate.y =
              velocity_estimate.y +
              config.beta_vy *
                (pose_delta_measurement.y / local_delta_measurement_delta_time -
                 velocity_estimate.y);

            velocity_estimate.orientation =
              velocity_estimate.orientation +
              config.beta_vtheta * (pose_delta_measurement.orientation /
                                      local_delta_measurement_delta_time -
                                    velocity_estimate.orientation);

            // update pose estimate as well
            pose_estimate.x =
              pose_estimate.x + config.beta_x * ((pose_delta_measurement.x +
                                                  last_local_estimate.x) -
                                                 pose_estimate.x);

            pose_estimate.y =
              pose_estimate.y + config.beta_y * ((pose_delta_measurement.y +
                                                  last_local_estimate.y) -
                                                 pose_estimate.y);

            pose_estimate.orientation =
              pose_estimate.orientation +
              config.beta_theta * ((pose_delta_measurement.orientation +
                                    last_local_estimate.orientation) -
                                   pose_estimate.orientation);

            last_local_delta_timestamp = current_local_delta_timestamp;
            applied_local = true;
        }

        // only correct pose if we have a new pose measurement
        if (current_pose_timestamp != last_pose_timestamp) {
            units::Pose pose_measurement = pose_model->getPose();

            pose_estimate.x =
              pose_estimate.x +
              config.alpha_x * (pose_measurement.x - pose_estimate.x);

            pose_estimate.y =
              pose_estimate.y +
              config.alpha_y * (pose_measurement.y - pose_estimate.y);

            pose_estimate.orientation =
              pose_estimate.orientation +
              config.alpha_theta *
                (pose_measurement.orientation - pose_estimate.orientation);

            last_pose_timestamp = current_pose_timestamp;
            applied_global = true;
        }

        global_pose_delta = units::Pose(pose_estimate.x - last_pose_estimate.x,
                                        pose_estimate.y - last_pose_estimate.y,
                                        pose_estimate.orientation -
                                          last_pose_estimate.orientation);

        distance_traveled += global_pose_delta.magnitude();

        Angle avg_angle =
          last_pose_estimate.orientation + global_pose_delta.orientation / 2.0;

        if(applied_local){
            last_local_estimate = pose_estimate;
        }

        local_pose_delta = localToGlobalDelta(global_pose_delta, avg_angle);
    }

    void setPose(units::Pose new_pose) override {
        pose_estimate = new_pose;
        last_pose_estimate = new_pose;
        last_local_estimate = new_pose;
        velocity_estimate = units::VelocityPose();

        // since only the local delta from this is used it isn't really needed,
        // however to keep consistency its still set
        local_delta_model->setPose(new_pose);

        pose_model->setPose(new_pose);
    }

    units::Pose getPose() override {
        return pose_estimate;
    }

    units::Pose getLastPose() override {
        return last_pose_estimate;
    }

    units::Pose getGlobalPoseDelta() override {
        return global_pose_delta;
    }

    units::Pose getLocalPoseDelta() override {
        return local_pose_delta;
    }

    std::optional<float> getConfidence() override {
        return std::nullopt;
    }

    Length getDistanceTraveled() override {
        return distance_traveled;
    }

    Time getTaskDeltaTime() override {
        return task_delta_time;
    }

    Time getLatestUpdateTimestamp() override {
        return latest_timestamp;
    }

    // TODO: make this part of LocalizationModel class
    Time getLatestDeltaTime() {
        return latest_delta_time;
    }

    ~SmootherModel() override = default;
};
} // namespace vexmaps
