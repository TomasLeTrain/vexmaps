#pragma once

#include "pros/apix.h"
#include "pros/imu.h"
#include "pros/motor_group.hpp"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include "vexmaps/localization_model.hpp"
#include "vexmaps/mcl/config.hpp"
#include "vexmaps/mcl/pose.hpp"
#include "vexmaps/mcl/utils.hpp"
#include "vexmath/fast_prng/Xoroshiro128plus_vectorized.hpp"
#include "vexmath/functions/trig_taylor.hpp"
#include "vexmath/functions/vectorized_trig.hpp"
#include "vexmath/functions/vectorized_trig_taylor.hpp"
#include <arm_neon.h>
#include <cmath>
#include <random>

using namespace vexmaps::localization_settings;

namespace vexmaps {
/**
 * @class MotionModel
 * @brief Wrapper for a localization model with support for adding noise.
 *
 */
class PfMotionModel : public LocalizationModel {
  private:
    std::uniform_real_distribution<float> average_distance_distribution;
    std::uniform_real_distribution<float> angle_distribution;
    std::uniform_real_distribution<float> drift_distribution;

    Vuniform_float32_t Vaverage_distance_distribution;
    Vuniform_float32_t Vangle_distribution;
    Vuniform_float32_t Vdrift_distribution;

    Time delta_time = 0_sec, last_update_time = INFINITY * sec;

    Angle avg_angle = 0_stDeg, abs_delta_theta = 0_stDeg;

    float32x4_t Vsina, Vcosa, Vnew_sina, Vnew_cosa;
    float sina, cosa, new_sina, new_cosa;

    float32x4_t Vglobal_pose_delta_x, Vglobal_pose_delta_y;

    units::Pose last_pose, global_pose_delta;

    /**
     * @brief Used to get an estimate for the Robot's movements. Owned and
     * managed by this class only.
     */
    std::unique_ptr<LocalizationModel> base_motion_model;

  public:
    PfMotionModel(std::unique_ptr<LocalizationModel> base_motion_model)
        : base_motion_model(std::move(base_motion_model)) {}

    void init() override {
        base_motion_model->init();
    }

    units::Pose getPose() override {
        return base_motion_model->getPose();
    }

    void setPose(units::Pose new_pose) override {
        base_motion_model->setPose(new_pose);
        // avoids super large pose delta
        last_pose = new_pose;
    }

    std::optional<float> getConfidence() override {
        return base_motion_model->getConfidence();
    }

    Length getDistanceTraveled() override {
        return base_motion_model->getDistanceTraveled();
    }

    Time getDeltaTime() override {
        return base_motion_model->getDeltaTime();
    }

    ~PfMotionModel() override = default;

    // TODO: update with timestamps so that the same pose is not used twice
    void update() override {
        Time current_time = from_msec(pros::millis());
        if (!std::isfinite(last_update_time.internal())) {
            delta_time = current_time - last_update_time;
        } else {
            // assume default delta time
            delta_time = base_motion_model->getDeltaTime();
        }
        last_update_time = current_time;

        // update base motion model first
        base_motion_model->update();

        auto current_pose = base_motion_model->getPose();
        auto global_pose_delta =
          units::Pose(current_pose - last_pose, // returns a vector
                      current_pose.orientation - last_pose.orientation);
        last_pose = current_pose;

        // here we must use the current angle for both angles
        // this should be fine unless we are rotating by a lot right as we start
        // the particle filter
        if (!std::isfinite(last_pose.orientation.internal())) {
            avg_angle = current_pose.orientation;
            global_pose_delta.orientation = 0.1_stDeg;
        } else {
            avg_angle = (current_pose.orientation + last_pose.orientation) / 2;
        }

        abs_delta_theta = units::abs(global_pose_delta.orientation);
        // does not include the orientation in the subraction

        // noise factors based on acceleration
        // const Length slip_noise =
        //   units::abs(slip_distance_ratio * average_distance);
        // const Length velocity_noise =
        //   slip_velocity_factor * units::abs(average_velocity);
        // const Length acceleration_slip_noise =
        //   slip_acceleration_factor * units::abs(average_acceleration);

        const Length distance_noise =
          DRIVE_NOISE2 + angle_vertical_noise_relation_factor * abs_delta_theta;
        const Angle angle_noise = abs_delta_theta * ANGLE_NOISE;
        const Length drift_noise =
          DRIFT_NOISE + angle_drift_relation_factor * abs_delta_theta;

        average_distance_distribution =
          std::uniform_real_distribution<float>((-distance_noise).internal(),
                                                (+distance_noise).internal());
        angle_distribution =
          std::uniform_real_distribution<float>((-angle_noise).internal(),
                                                (+angle_noise).internal());
        drift_distribution =
          std::uniform_real_distribution<float>((-drift_noise).internal(),
                                                (+drift_noise).internal());

        sina = units::sin(current_pose.orientation);
        cosa = units::cos(current_pose.orientation);

        if (usingVectorizedMotion) {
            Vaverage_distance_distribution =
              Vuniform_float32_t((-distance_noise).internal(),
                                 (+distance_noise).internal(),
                                 robot_rng());
            Vangle_distribution = Vuniform_float32_t((-angle_noise).internal(),
                                                     (+angle_noise).internal(),
                                                     robot_rng());
            Vdrift_distribution = Vuniform_float32_t((-drift_noise).internal(),
                                                     (+drift_noise).internal(),
                                                     robot_rng());

            Vsina = vld1q_dup_f32(&sina);
            Vcosa = vld1q_dup_f32(&cosa);

            // needed since vld1q_dup_f32 takes a float pointer
            float global_pose_delta_x = global_pose_delta.x.internal();
            float global_pose_delta_y = global_pose_delta.y.internal();

            Vglobal_pose_delta_x = vld1q_dup_f32(&global_pose_delta_x);
            Vglobal_pose_delta_y = vld1q_dup_f32(&global_pose_delta_y);
        }
    }

    /**
     * @brief makes a vector of noisy global deltas (in base units)
     *
     * @param result vector where the motion updates get stored
     */
    void VnoisyGlobalDelta(float32x4x2_t* result) {
        const float32x4_t vertical_noise = Vaverage_distance_distribution();
        const float32x4_t horizontal_noise = Vdrift_distribution();
        const float32x4_t angle_noise = Vangle_distribution();

        Vsincos_taylor_delta(angle_noise, Vsina, Vcosa, &Vnew_sina, &Vnew_cosa);

        // x
        result->val[0] = Vglobal_pose_delta_x + vertical_noise * Vnew_cosa -
                         horizontal_noise * Vnew_sina;
        // y
        result->val[1] = Vglobal_pose_delta_y + vertical_noise * Vnew_sina +
                         horizontal_noise * Vnew_cosa;
    }

    //
    /**
     * @brief odometry change with some noise added for each particle
     *
     * @return Noisy global delta
     */
    Point noisyGlobalDelta() {
        const Length vertical_noise = average_distance_distribution(rng) * m;
        const Length horizontal_noise = drift_distribution(rng) * m;
        const float angle_noise = angle_distribution(rng);

        sincos_taylor_delta(angle_noise, sina, cosa, &new_sina, &new_sina);

        // rotates forward and sideways movement to face the robot's heading
        return { global_pose_delta.x + vertical_noise * new_cosa -
                   horizontal_noise * new_sina,
                 global_pose_delta.y + vertical_noise * new_sina +
                   horizontal_noise * new_cosa };
    }
};

} // namespace vexmaps
