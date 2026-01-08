#pragma once

#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include "vexmaps/localization_model.hpp"
#include <mutex>
#include <optional>

namespace vexmaps {
struct SmootherConfig {
    // for all parameters:
    // 0 = all model
    // 1 = all measurement

    // // determines how much a pose measurement influences the pose estimate
    double alpha_x = 0.03;
    double alpha_y = 0.03;
    double alpha_theta = 0.00;

    // determines how angular velocity translates to a change in alpha values
    // alpha -= ang_vel_alpha * abs(angular_velocity)
    Divided<Number, AngularVelocity> ang_vel_alpha = 0.02 / 300_degps;

    // determines how much theta being straight changes alpha
    // alpha -= theta_alpha * theta_func(theta), where theta_func(theta)
    // peaks at 1 when robot is at 45 degree angles
    double theta_to_alpha = 0.02;

    // determines how much the linear velocity of the robot changes alpha
    // alpha -= abs_vel_alpha * velocity_vector.magnitude()
    Divided<Number, LinearVelocity> linear_vel_alpha = 0.01 / 70_inps;

    // used by pose_delta_measurement to estimate the pose
    double beta_x = 1;
    double beta_y = 1;
    double beta_theta = 1;
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

    units::Pose last_local_estimate = units::Pose();

    // measures the local pose deltas of the robot
    LocalizationModel* local_delta_model;
    // measures the global pose of the robot
    LocalizationModel* pose_model;

    Time last_local_delta_timestamp = 0_msec;
    Time last_pose_timestamp = 0_msec;

    Length forward_travel = 0_m;

    // relatively quick so that we can get the latest updates as fast as
    // possible
    Time task_delta_time = 8_msec;

    SmootherConfig config;

  protected:
    mutable pros::Mutex m_mutex;

  public:
    SmootherModel(LocalizationModel* local_delta_model,
                  LocalizationModel* global_pose_model,
                  SmootherConfig config)
        : local_delta_model(local_delta_model),
          pose_model(global_pose_model),
          config(config) {}

    void init() override {
        std::lock_guard lock(m_mutex);
        latest_timestamp = from_msec(pros::millis());
    }

    void update() override {
        std::lock_guard lock(m_mutex);
        Time current_timestamp = from_msec(pros::millis());

        latest_delta_time = current_timestamp - latest_timestamp;
        latest_timestamp = current_timestamp;

        Time current_local_delta_timestamp =
          local_delta_model->getLatestUpdateTimestamp();

        Time current_pose_timestamp = pose_model->getLatestUpdateTimestamp();

        // update last_pose
        last_pose_estimate = pose_estimate;

        bool applied_local = false;
        bool applied_global = false;

        // only use if we have a new delta measurement
        if (current_local_delta_timestamp != last_local_delta_timestamp) {

            units::Pose pose_delta_measurement =
              local_delta_model->getGlobalPoseDelta();

            units::Pose new_pose { pose_delta_measurement + last_local_estimate,
                                   pose_delta_measurement.orientation +
                                     last_local_estimate.orientation };

            units::Pose difference { new_pose - pose_estimate,
                                     new_pose.orientation -
                                       pose_estimate.orientation };

            // update pose estimate as well
            pose_estimate.x += config.beta_x * difference.x;
            pose_estimate.y += config.beta_y * difference.y;
            pose_estimate.orientation +=
              config.beta_theta * difference.orientation;

            last_local_delta_timestamp = current_local_delta_timestamp;
            applied_local = true;
        }

        // only correct pose if we have a new pose measurement
        if (current_pose_timestamp != last_pose_timestamp &&
            // only use if it measures the global directly
            pose_model->getConfidence().has_value()) {
            units::Pose pose_measurement = pose_model->getPose();
            auto confidences = pose_model->getConfidence();

            AngularVelocity abs_angular_velocity =
              units::abs(local_delta_model->getAngularVelocity());
            LinearVelocity linear_velocity =
              local_delta_model->getLocalVelocityVector().magnitude();

            // returns value between [0,1], where 0 indicates perfectly
            // perpendiular and 1 indicates facing 45 degree angles
            auto theta_func = [](Angle theta) -> double {
                return 1 - units::abs(units::cos(2 * theta));
            };

            double alpha_difference =
              abs_angular_velocity * config.ang_vel_alpha +
              linear_velocity * config.linear_vel_alpha +
              theta_func(getPose().orientation) * config.theta_to_alpha;

            double new_x_alpha =
              units::max(config.alpha_x - alpha_difference, 0);
            double new_y_alpha =
              units::max(config.alpha_y - alpha_difference, 0);

            units::V2Position difference = pose_measurement - pose_estimate;

            if (confidences->know_x)
                pose_estimate.x += new_x_alpha * difference.x;
            if (confidences->know_y)
                pose_estimate.y += new_y_alpha * difference.y;

            // only good if pose has an orientation measurement
            // pose_estimate.orientation =
            //   pose_estimate.orientation +
            //   config.alpha_theta *
            //     (pose_measurement.orientation - pose_estimate.orientation);

            last_pose_timestamp = current_pose_timestamp;
            applied_global = true;
        }

        global_pose_delta = { pose_estimate - last_pose_estimate,
                              pose_estimate.orientation -
                                last_pose_estimate.orientation };

        distance_traveled += global_pose_delta.magnitude();

        Angle avg_angle =
          last_pose_estimate.orientation + global_pose_delta.orientation / 2.0;

        if (applied_local) {
            last_local_estimate = pose_estimate;
        }

        local_pose_delta = globalToLocalDelta(global_pose_delta, avg_angle);

        // add forward travel from local delta
        forward_travel += local_pose_delta.x;
    }

    void setPose(units::Pose new_pose) override {
        std::lock_guard lock(m_mutex);
        pose_estimate = new_pose;
        last_pose_estimate = new_pose;
        last_local_estimate = new_pose;

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

    void changeConfiguration(SmootherConfig new_config) {
        std::lock_guard lock(m_mutex);
        config = new_config;
    }

    SmootherConfig getConfiguration() {
        return config;
    }

    // returns a signed distance traveled from the start of tracking
    Length getForwardTravel() override {
        return forward_travel;
    }

    // returns local velocity vector relative to the robot
    units::V2Velocity getLocalVelocityVector() override {
        // uses directly from local delta model since its likely very accurate
        return local_delta_model->getLocalVelocityVector();
    }

    // returns the latest angular velocity
    AngularVelocity getAngularVelocity() override {
        // just uses angular velocity from local delta model
        return local_delta_model->getAngularVelocity();
    }

    std::optional<Confidences> getConfidence() override {
        return std::nullopt;
    }

    ~SmootherModel() override = default;
};
} // namespace vexmaps
