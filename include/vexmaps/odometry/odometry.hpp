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
#include <mutex>

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

    double local_x_delta = 0;
    double local_y_delta = 0;

    double global_y_delta = 0;
    double global_x_delta = 0;

    double angle_delta = 0;

    double pose_x = 0;
    double pose_y = 0;
    double angle = 0;

    double last_pose_x = 0;
    double last_pose_y = 0;
    double last_angle = 0;

    double distance_traveled;

    bool use_drivetrain = false;

    Time delta_time = 10.0_msec;
    Time latest_update_time = 0.0_sec;
  protected:
    mutable pros::Mutex m_mutex;

  public:
    OdometryModel(
      MotorGroupTracking* left_tracker,
      MotorGroupTracking* right_tracker,
      std::initializer_list<HorizontalOdometryTracker*> horizontal_trackers,
      std::initializer_list<VerticalOdometryTracker*> vertical_trackers,
      pros::Imu* imu,
      bool use_drivetrain = false)
        : left_tracker(left_tracker),
          right_tracker(right_tracker),
          horizontal_trackers(horizontal_trackers),
          vertical_trackers(vertical_trackers),
          imu(imu),
          use_drivetrain(use_drivetrain) {}

    /**
     * @brief Should be called once to initialize the model
     */
    void init() override {
        std::lock_guard lock(m_mutex);
        imu->set_rotation(0);

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
        std::lock_guard lock(m_mutex);

        left_tracker->update();
        right_tracker->update();

        int available_vertical = 0;
        int available_horizontal = 0;

        for (auto&& tracker : vertical_trackers) {
            tracker->update();
            available_vertical += tracker->getAvailable();
        }
        for (auto&& tracker : horizontal_trackers) {
            tracker->update();
            available_horizontal += tracker->getAvailable();
        }

        bool drivetrain_enabled = false;

        // if no vertical trackers then we use drivetrain regardless of
        // configuration
        if (available_vertical == 0 || use_drivetrain) {
            drivetrain_enabled = true;
        }

        double current_imu_angle = last_imu_angle;

        if(imu == nullptr){
            printf("WARNING: IMU IS NULL - Check config for a nullptr!\n");
            current_imu_angle = last_imu_angle;
        }else if(!imu->is_installed()){
            printf("WARNING: IMU NOT DETECTED - Check cable connection or port!\n");
            current_imu_angle = last_imu_angle;
        }else{
            // imu is installed
            if (std::isfinite(imu->get_rotation())) {
                current_imu_angle = imu->get_rotation() * (M_PI / 180.0);
            } else {
                printf("WARNING: IMU NOT FINITE - Might not be calibrated or did not calibrate properly!\n");
                current_imu_angle = last_imu_angle;
            }
        }

        double imu_angle_delta = current_imu_angle - last_imu_angle;

        // makes it so that we dont have to deal with compass -> std rad
        // conversion
        if (imu_angle_delta != 0) {
            angle_delta = -imu_angle_delta;
        } else {
            angle_delta = 0;
        }

        // update angle
        angle = angle_delta + last_angle;

        local_y_delta = 0;
        local_x_delta = 0;

        double x_tracker_count = 0;
        double y_tracker_count = 0;

        // clang-format off
        if (fabs(angle_delta) < 1e-6) {
            if (drivetrain_enabled) {
                local_x_delta += (left_tracker->getDeltaDistance() + right_tracker->getDeltaDistance()) / 2.0;
                x_tracker_count += 1.0;
            }

            for (auto&& tracker : vertical_trackers) {
                if(tracker->getAvailable()){
                    local_x_delta += tracker->getDeltaDistance();
                    x_tracker_count += 1.0;
                }
            }

            for (auto&& tracker : horizontal_trackers) {
                if(tracker->getAvailable()){
                    local_y_delta += tracker->getDeltaDistance();
                    y_tracker_count += 1.0;
                }
            }
        } else {
            double sin_multiplier = 2.0 * sin(angle_delta / 2.0);

            if (drivetrain_enabled) {
                double local_x_left_delta = sin_multiplier * (left_tracker->getDeltaDistance() / angle_delta + left_tracker->getOffset());
                double local_x_right_delta = sin_multiplier * (right_tracker->getDeltaDistance() / angle_delta + right_tracker->getOffset());
                local_x_delta += (local_x_left_delta + local_x_right_delta) / 2.0;
                x_tracker_count += 1.0;
            }

            for (auto&& tracker : vertical_trackers) {
                if(tracker->getAvailable()){
                    local_x_delta += sin_multiplier * (tracker->getDeltaDistance() / angle_delta + tracker->getOffset());
                    // printf("\\left(%f,",tracker->getDeltaDistance() / angle_delta);
                    x_tracker_count += 1.0;
                }
            }

            for (auto&& tracker : horizontal_trackers) {
                if(tracker->getAvailable()){
                    local_y_delta += sin_multiplier * (tracker->getDeltaDistance() / angle_delta + tracker->getOffset());
                    // printf("%f\\right),\n",tracker->getDeltaDistance() / angle_delta);
                    y_tracker_count += 1.0;
                }
            }
        }
        // clang-format on

        if (x_tracker_count > 1) local_x_delta /= x_tracker_count;
        if (y_tracker_count > 1) local_y_delta /= y_tracker_count;

        // Update global position using polar coordinates
        double avg_angle = (angle + last_angle) / 2.0;

        double sina = sin(avg_angle);
        double cosa = cos(avg_angle);

        global_x_delta = local_x_delta * cosa - local_y_delta * sina;
        global_y_delta = local_x_delta * sina + local_y_delta * cosa;

        last_pose_x = pose_x;
        last_pose_y = pose_y;

        pose_x += global_x_delta;
        pose_y += global_y_delta;

        distance_traveled += sqrt((global_x_delta * global_x_delta) +
                                  (global_y_delta * global_y_delta));

        // update last- variables
        last_imu_angle = current_imu_angle;
        last_angle = angle;

        latest_update_time = from_msec(pros::millis());
    }

    void setPose(units::Pose new_pose) override {
        std::lock_guard lock(m_mutex);

        pose_x = to_in(new_pose.x);
        pose_y = to_in(new_pose.y);
        angle = new_pose.orientation.internal();

        // last_set_orientation = new_pose.orientation.internal();
        // imu->set_heading(0);

        last_pose_x = pose_x;
        last_pose_y = pose_y;
        last_angle = angle;
    }

    // getters

    units::Pose getPose() override {
        return units::Pose(from_in(pose_x), from_in(pose_y), from_stRad(angle));
    }

    /**
     * @brief gets the previous available pose
     */
    units::Pose getLastPose() override {
        return units::Pose(from_in(last_pose_x),
                           from_in(last_pose_y),
                           from_stRad(last_angle));
    }

    /**
     * @brief Get latest global pose delta
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
