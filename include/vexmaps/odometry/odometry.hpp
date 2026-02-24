/**
 * @file
 * @brief Implements arc odometry using the drivetrain motors + an imu.
 */

#pragma once

#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
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
    units::V2Position m_tracking_center_offsets;

    Angle last_imu_angle = 0_stDeg;

    units::V2Position last_local_delta, local_delta;
    Angle last_angle_delta = 0_stDeg;

    units::V2Position global_delta;
    Angle angle_delta = 0_stDeg;

    units::Pose pose;
    units::Pose last_pose;

    Length distance_traveled;

    Time delta_time = 10_msec;
    Time latest_update_time = 0_sec;

    Length forward_travel = 0_m;
    units::V2Velocity local_velocity_vector;
    AngularVelocity angular_velocity;

    bool use_drivetrain = false;
    bool m_print_on_failure = false;

  protected:
    mutable pros::Mutex m_mutex;

  public:
    OdometryModel(
      MotorGroupTracking* left_tracker,
      MotorGroupTracking* right_tracker,
      std::initializer_list<HorizontalOdometryTracker*> horizontal_trackers,
      std::initializer_list<VerticalOdometryTracker*> vertical_trackers,
      pros::Imu* imu,
      units::V2Position tracking_center_offsets = { 0_in, 0_in },
      bool use_drivetrain = false,
      bool print_on_failure = false)
        : left_tracker(left_tracker),
          right_tracker(right_tracker),
          horizontal_trackers(horizontal_trackers),
          vertical_trackers(vertical_trackers),
          imu(imu),
          m_tracking_center_offsets(tracking_center_offsets),
          use_drivetrain(use_drivetrain),
          m_print_on_failure(print_on_failure) {}

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
        // int available_horizontal = 0;

        for (auto&& tracker : vertical_trackers) {
            tracker->update();
            available_vertical += tracker->getAvailable();
        }
        for (auto&& tracker : horizontal_trackers) {
            tracker->update();
            // available_horizontal += tracker->getAvailable();
        }

        bool drivetrain_enabled = false;

        // if no vertical trackers then we use drivetrain regardless of
        // configuration
        if (available_vertical == 0 || use_drivetrain) {
            drivetrain_enabled = true;
        }

        Angle current_imu_angle = last_imu_angle;

        if (imu == nullptr) {
            if (m_print_on_failure)
                printf("WARNING: IMU IS NULL - Check config for a nullptr!\n");
            current_imu_angle = last_imu_angle;
        } else if (!imu->is_installed()) {
            if (m_print_on_failure)
                printf("WARNING: IMU NOT DETECTED - Check cable connection or "
                       "port!\n");
            current_imu_angle = last_imu_angle;
        } else {
            // imu is installed
            if (std::isfinite(imu->get_rotation())) {
                current_imu_angle = from_stDeg(imu->get_rotation());
            } else {
                if (m_print_on_failure)
                    printf("WARNING: IMU NOT FINITE - Might not be calibrated "
                           "or did not calibrate properly!\n");
                current_imu_angle = last_imu_angle;
            }
        }

        // delta is in imu system
        Angle imu_angle_delta = current_imu_angle - last_imu_angle;

        // reverts into standard degrees
        if (imu_angle_delta.internal() != 0) {
            angle_delta = -imu_angle_delta;
        } else {
            angle_delta = 0_stDeg;
        }

        // update angle
        pose.orientation = angle_delta + last_pose.orientation;

        local_delta = units::V2Position();

        int x_tracker_count = 0;
        int y_tracker_count = 0;

        if (units::abs(angle_delta).internal() < 1e-6) {
            if (drivetrain_enabled) {
                local_delta.x += (left_tracker->getDeltaDistance() +
                                  right_tracker->getDeltaDistance()) /
                                 2.0;
                x_tracker_count++;
            }

            for (auto&& tracker : vertical_trackers) {
                if (tracker->getAvailable()) {
                    local_delta.x += tracker->getDeltaDistance();
                    x_tracker_count++;
                }
            }

            for (auto&& tracker : horizontal_trackers) {
                if (tracker->getAvailable()) {
                    local_delta.y += tracker->getDeltaDistance();
                    y_tracker_count++;
                }
            }
        } else {
            double sin_multiplier = 2.0 * units::sin(angle_delta / 2.0);

            if (drivetrain_enabled) {
                Length local_dt_left_delta =
                  sin_multiplier *
                  (left_tracker->getDeltaDistance() / angle_delta.internal() -
                   left_tracker->getOffset());

                Length local_dt_right_delta =
                  sin_multiplier *
                  (right_tracker->getDeltaDistance() / angle_delta.internal() -
                   right_tracker->getOffset());

                local_delta.x +=
                  (local_dt_left_delta + local_dt_right_delta) / 2.0;
                x_tracker_count++;
            }

            for (auto&& tracker : vertical_trackers) {
                if (tracker->getAvailable()) {
                    local_delta.x +=
                      sin_multiplier *
                      (tracker->getDeltaDistance() / angle_delta.internal() -
                       tracker->getOffset());
                    x_tracker_count++;
                }
            }

            for (auto&& tracker : horizontal_trackers) {
                if (tracker->getAvailable()) {
                    local_delta.y +=
                      sin_multiplier *
                      (tracker->getDeltaDistance() / angle_delta.internal() -
                       tracker->getOffset());
                    y_tracker_count++;
                }
            }
        }

        if (x_tracker_count > 1)
            local_delta.x /= static_cast<double>(x_tracker_count);
        if (y_tracker_count > 1)
            local_delta.y /= static_cast<double>(y_tracker_count);

        // Update global position using polar coordinates
        Angle avg_angle = (pose.orientation + last_pose.orientation) / 2.0;

        global_delta = localToGlobalDelta(local_delta, avg_angle);

        // only changes x/y, not orientation (it was already updated)
        pose += global_delta;

        distance_traveled += global_delta.magnitude();

        forward_travel += local_delta.x;

        // averages last and current local deltas to eliminate noise
        local_velocity_vector =
          (local_delta + last_local_delta) / (getTaskDeltaTime() * 2);

        // averages last and current delta to eliminate noise
        // angular_velocity =
        //   (angle_delta + last_angle_delta) / (getTaskDeltaTime() * 2);

		// specific to z down orientation
        angular_velocity = -imu->get_gyro_rate().z * degps;

        // update last- variables
        last_pose = pose;
        last_imu_angle = current_imu_angle;
        last_local_delta = local_delta;
        last_angle_delta = angle_delta;
        latest_update_time = from_msec(pros::millis());
    }

    void setPose(units::Pose new_pose) override {
        std::lock_guard lock(m_mutex);

        // change from the tracking center to the center of rotation
        new_pose -= m_tracking_center_offsets.rotatedBy(new_pose.orientation);

        pose = new_pose;
        last_pose = pose;
    }

    // getters
    units::Pose getPose() override {
        // change from the center of rotation to the tracking center
        return { pose + m_tracking_center_offsets.rotatedBy(pose.orientation),
                 pose.orientation };
    }

    /**
     * @brief gets the previous available pose
     */
    units::Pose getLastPose() override {
        // change from the center of rotation to the tracking center
        return { last_pose +
                   m_tracking_center_offsets.rotatedBy(last_pose.orientation),
                 last_pose.orientation };
    }

    /**
     * @brief Get latest global pose delta
     */
    units::Pose getGlobalPoseDelta() override {
        // TODO: should take into account tracking center offsets
        return units::Pose(global_delta, angle_delta);
    }

    /**
     * @brief Get latest local pose delta
     */
    units::Pose getLocalPoseDelta() override {
        return units::Pose(local_delta, angle_delta);
    }

    /**
     * @brief returns a value representing the confidence of the current
     * pose estimate (if the model supports it). returns nullopt if not
     * supported.
     *
     * @return the confidence on the estimate (if available)
     */
    std::optional<Confidences> getConfidence() override {
        return std::nullopt;
    }

    /**
     * @brief Total distance the robot has traveled
     *
     * @return distance traveled by the robot
     */
    Length getDistanceTraveled() override {
        return distance_traveled;
    }

    Time getTaskDeltaTime() override {
        return delta_time;
    }

    Time getLatestUpdateTimestamp() override {
        return latest_update_time;
    }

    // returns a signed distance traveled from the start of tracking
    Length getForwardTravel() override {
        return forward_travel;
    }

    // returns local velocity vector relative to the robot
    units::V2Velocity getLocalVelocityVector() override {
        // uses directly from local delta model since its likely very accurate
        return local_velocity_vector;
    }

    // returns the latest angular velocity
    AngularVelocity getAngularVelocity() override {
        return angular_velocity;
    }

    ~OdometryModel() override = default;
};
} // namespace vexmaps
