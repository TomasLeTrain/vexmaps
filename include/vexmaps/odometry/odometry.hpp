/**
 * @file
 * @brief Implements arc odometry using the drivetrain motors + an imu.
 */

#pragma once

#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/units.hpp"
#include "vexmaps/localization_model.hpp"
#include "vexmaps/odometry/tracking_wheel.hpp"

namespace vexmaps {
class OdometryModel : public LocalizationModel {
  private:
    MotorGroupTracking* left_tracker;
    MotorGroupTracking* right_tracker;

    std::vector<HorizontalOdometryTracker*> horizontal_trackers;
    std::vector<VerticalOdometryTracker*> vertical_trackers;

    pros::Imu* imu;

    // cant use units since its floats and precision does matter
    // all angles in radians, all distances in inches

    double last_imu_angle;

    double local_y_delta = 0;
    double local_x_delta = 0;

    double global_y_delta = 0;
    double global_x_delta = 0;

    double angle_delta = 0;

    double pose_x;
    double pose_y;
    double angle;

    double last_pose_x;
    double last_pose_y;
    double last_angle;

    double distance_traveled;

    // TODO: figure out when to set this value to false
    bool using_drivetrain = false;

    Time delta_time = 10.0_msec;
    Time latest_update_time = 0.0_sec;

  public:
    OdometryModel(MotorGroupTracking* left_tracker,
                  MotorGroupTracking* right_tracker,
                  std::vector<HorizontalOdometryTracker*>&& horizontal_trackers,
                  std::vector<VerticalOdometryTracker*>&& vertical_trackers,
                  pros::Imu* imu)
        : left_tracker(left_tracker),
          right_tracker(right_tracker),
          horizontal_trackers(std::move(horizontal_trackers)),
          vertical_trackers(std::move(vertical_trackers)),
          imu(imu) {}

    /**
     * @brief Should be called once to initialize the model
     */
    void init() override {
        // last_imu_angle = imu->get_rotation() * (M_PI / 360.0);
        last_imu_angle = imu->get_rotation() * (M_PI / 180.0);

        double local_y_delta = 0;
        double local_x_delta = 0;

        double global_y_delta = 0;
        double global_x_delta = 0;

        double angle_delta = 0;

        // defaults to (0,0)
        double pose_x = 0;
        double pose_y = 0;
        double angle = 0;

        double last_pose_x = 0;
        double last_pose_y = 0;
        double last_angle = 0;

        double distance_traveled = 0;

        left_tracker->init();
        right_tracker->init();

        for (auto&& tracker : vertical_trackers) {
            tracker->init();
        }
        for (auto&& tracker : horizontal_trackers) {
            tracker->init();
        }
    }

    /**
     * @brief Updates pose estimate. Should be called frequently
     */
    void update() override {
        left_tracker->update();
        right_tracker->update();

        for (auto&& tracker : vertical_trackers) {
            tracker->update();
        }
        for (auto&& tracker : horizontal_trackers) {
            tracker->update();
        }

        double current_imu_angle = last_imu_angle;

        if(std::isfinite(imu->get_rotation())){
            current_imu_angle = imu->get_rotation() * (M_PI / 180.0);
        }else{
            printf("warning: imu not finite\n");
            current_imu_angle = last_imu_angle;
        }

        // printf("current_imu_angle: %f\n",current_imu_angle);
        double imu_angle_delta = current_imu_angle - last_imu_angle;
        // printf("imu_delta: %f\n",imu_angle_delta);

        // makes it so that we dont have to deal with compass -> std rad
        // conversion
        if(imu_angle_delta != 0){
            angle_delta = -imu_angle_delta;
        }else{
            angle_delta = 0;
        }
        angle = angle_delta + last_angle;

        local_x_delta = 0;
        local_y_delta = 0;

        double x_tracker_count = 0;
        double y_tracker_count = 0;
        // printf("angle_delta: %f\n",angle_delta);

        // clang-format off
        if (fabs(angle_delta) < 1e-6) {
            if (using_drivetrain) {
                local_y_delta += (left_tracker->getDeltaDistance() + left_tracker->getDeltaDistance()) / 2.0;
                y_tracker_count += 1.0;
            }
            // printf("local_y_delta so far1: %f\n",local_y_delta);

            for (auto&& tracker : vertical_trackers) {
                local_y_delta += tracker->getDeltaDistance();
                y_tracker_count += 1.0;
            }

            for (auto&& tracker : horizontal_trackers) {
                local_x_delta += tracker->getDeltaDistance();
                x_tracker_count += 1.0;
            }
        } else {
            double sin_multiplier = 2.0 * sin(angle_delta / 2.0);
            // printf("multiplier: %f\n",sin_multiplier);

            if (using_drivetrain) {
                double local_y_left_delta = sin_multiplier * (left_tracker->getDeltaDistance() / angle_delta + left_tracker->getOffset());
                double local_y_right_delta = sin_multiplier * (right_tracker->getDeltaDistance() / angle_delta + right_tracker->getOffset());
                local_y_delta += (local_y_left_delta + local_y_right_delta) / 2.0;
                y_tracker_count += 1.0;
            }
            // printf("local_y_delta so far2: %f\n",local_y_delta);

            for (auto&& tracker : vertical_trackers) {
                local_y_delta += sin_multiplier * (tracker->getDeltaDistance() / angle_delta + tracker->getOffset());
                // printf("\\left(%f,",tracker->getDeltaDistance() / angle_delta);
                y_tracker_count += 1.0;
            }

            for (auto&& tracker : horizontal_trackers) {
                local_x_delta += sin_multiplier * (tracker->getDeltaDistance() / angle_delta + tracker->getOffset());
                // printf("%f\\right),",tracker->getDeltaDistance() / angle_delta);
                x_tracker_count += 1.0;
            }
        }
        // clang-format on

        if (x_tracker_count > 1) local_x_delta /= x_tracker_count;
        if (y_tracker_count > 1) local_y_delta /= y_tracker_count;

        // printf("local_x/y: %f %f\n",local_x_delta, local_y_delta);

        // Update global position using polar coordinates
        double avg_angle = (angle + last_angle) / 2.0;
        // printf("avg_angle_y: %f %f %f\n",avg_angle,angle,last_angle);

        double sina = sin(avg_angle);
        double cosa = cos(avg_angle);

        global_x_delta = local_y_delta * cosa - local_x_delta * sina;
        global_y_delta = local_y_delta * sina + local_x_delta * cosa;
        // printf("global_x/y: %f %f\n",global_x_delta,global_y_delta);

        last_pose_x = pose_x;
        last_pose_y = pose_y;

        pose_x += global_x_delta;
        pose_y += global_y_delta;

        distance_traveled += sqrt((global_y_delta * global_y_delta) +
                                  (global_x_delta * global_x_delta));

        // update last- variables
        last_imu_angle = current_imu_angle;
        last_angle = angle;

        latest_update_time = from_msec(pros::millis());
    }

    void setPose(units::Pose new_pose) override {
        pose_x = to_in(new_pose.x);
        pose_y = to_in(new_pose.y);
        angle = new_pose.orientation.internal();

        last_pose_x = pose_x;
        last_pose_y = pose_y;
        last_angle = angle;
    }

    units::Pose getPose() override {
        return units::Pose(from_in(pose_x), from_in(pose_y), from_stRad(angle));
    }

    /**
     * @brief gets the previous available pose, if it exists
     */
    units::Pose getLastPose() override {
        return units::Pose(from_in(last_pose_x),
                           from_in(last_pose_y),
                           from_stRad(last_angle));
    }

    /**
     * @brief Get latest global pose delta, if it exists
     */
    units::Pose getGlobalPoseDelta() override {
        return units::Pose(from_in(global_x_delta),
                           from_in(global_y_delta),
                           from_stRad(angle_delta));
    }

    /**
     * @brief Get latest local pose delta
     */
    units::Pose getLocalPoseDelta() override {
        return units::Pose(from_in(local_x_delta),
                           from_in(local_y_delta),
                           from_stRad(angle_delta));
    }

    /**
     * @brief returns a value representing the confidence of the current
     * pose estimate (if the model supports it). returns nullopt if not
     * supported.
     *
     * @return the confidence on the estimate (if available)
     */
    std::optional<float> getConfidence() override {
        return std::nullopt;
    }

    /**
     * @brief Total distance the robot has traveled
     *
     * @return distance traveled by the robot
     */
    Length getDistanceTraveled() override {
        return from_in(distance_traveled);
    }

    Time getTaskDeltaTime() override {
        return delta_time;
    }

    Time getLatestUpdateTimestamp() override {
        return latest_update_time;
    }

    ~OdometryModel() override = default;
};
} // namespace vexmaps
