#pragma once

#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/units.hpp"
#include "vexmaps/localization_model.hpp"
#include "vexmaps/mcl/config.hpp"
#include "vexmaps/mcl/point.hpp"
#include "vexmaps/mcl/utils.hpp"
#include "vexmath/fast_prng/Xoroshiro128plus_vectorized.hpp"
#include "vexmath/functions/trig_taylor.hpp"
#include "vexmath/functions/vectorized_trig_taylor.hpp"
#include <arm_neon.h>
#include <cmath>
#include <memory>
#include <random>

// TODO: add settings struct to be able to configure settings
//
namespace vexmaps {

/**
 * @class BasePfMotionModel
 * @brief Base class without template specialization. Allows passing around
 * pointers without template arguments.
 *
 */
class BasePfMotionModel : public LocalizationModel {
  public:
    /**
     * @brief Vectorized version of noisyGlobalDelta
     */
    virtual inline void VnoisyGlobalDelta(float32x4_t* Xresult,
                                          float32x4_t* Yresult) = 0;
    /**
     * @brief Returns a noisy global delta
     */
    virtual Point noisyGlobalDelta() = 0;
    /**
     * @brief Called by the particle filter to update the lost iteration count
     */
    virtual void updateLostIterationCount(int new_count) = 0;
    /**
     * @brief Precomputes values based on a global pose delta. Should be called
     * right before getting noisy global deltas.
     */
    virtual units::Pose precompute() = 0;
};

/**
 * @class MotionModel
 * @brief Wrapper for a localization model with support for adding noise.
 *
 */
template<typename ModelType>
    requires std::derived_from<ModelType, LocalizationModel>
class PfMotionModel : public BasePfMotionModel {
  private:
    std::uniform_real_distribution<float> average_distance_distribution;
    std::uniform_real_distribution<float> angle_distribution;
    std::uniform_real_distribution<float> drift_distribution;

    Vuniform_float32_t forwards_distribution;

    float angle_a = 0;
    float angle_b = 0;
    float angle_k = 0;

    float drift_a = 0;
    float drift_b = 0;
    float drift_k = 0;

    Time update_timestamp = 0_sec;

    Angle abs_delta_theta = 0_stDeg;

    float32x4_t Vsina, Vcosa;
    float sina, cosa;

    float global_pose_delta_x, global_pose_delta_y;

    float lost_iteration_count = 0;

    // used to compute the delta time between motion updates
    Time last_precomputed_time = infinity() * sec;

    units::Pose accumulated_global_delta = units::Pose(), global_pose_delta;

    MotionModelConfig motionModelConfig;

    /**
     * @brief Used to get an estimate for the Robot's movements. Owned and
     * managed by this class only.
     */
    ModelType base_motion_model;

  public:
    template<typename... Args>
        requires std::is_constructible_v<ModelType, Args...>
    PfMotionModel(MotionModelConfig motionModelConfig, Args&&... args)
        : base_motion_model(std::forward<Args>(args)...),
          motionModelConfig(motionModelConfig),
          forwards_distribution(robot_rng()) {}

    // wrapper functions for the base motion model

    void init() override {
        base_motion_model.init();
    }

    units::Pose getPose() override {
        return base_motion_model.getPose();
    }

    void setPose(units::Pose new_pose) override {
        base_motion_model.setPose(new_pose);
    }

    std::optional<float> getConfidence() override {
        return base_motion_model.getConfidence();
    }

    units::Pose getGlobalPoseDelta() override {
        return base_motion_model.getGlobalPoseDelta();
    }

    units::Pose getLocalPoseDelta() override {
        return base_motion_model.getLocalPoseDelta();
    }

    units::Pose getLastPose() override {
        return base_motion_model.getLastPose();
    }

    Length getDistanceTraveled() override {
        return base_motion_model.getDistanceTraveled();
    }

    Time getTaskDeltaTime() override {
        return base_motion_model.getTaskDeltaTime();
    }

    Time getLatestUpdateTimestamp() override {
        return update_timestamp;
    }

    ~PfMotionModel() override = default;

    // TODO: incorporate confidence value from base motion model to increase or
    // decrease noise
    void update() override {
        // update base motion model first
        base_motion_model.update();

        accumulated_global_delta += base_motion_model.getGlobalPoseDelta();

        // update timestamps
        update_timestamp = from_msec(pros::millis());
    }

    units::Pose precompute() override {
        units::Pose current_pose = base_motion_model.getPose();

        global_pose_delta = accumulated_global_delta;
        accumulated_global_delta = units::Pose();

        Time current_precompute_time = from_msec(pros::millis());
        Time delta_time;

        if (std::isfinite(last_precomputed_time.internal())) {
            delta_time = current_precompute_time - last_precomputed_time;
        } else {
            // assume task delta time
            delta_time = base_motion_model.getTaskDeltaTime();
        }
        last_precomputed_time = current_precompute_time;

        abs_delta_theta = units::abs(global_pose_delta.orientation);

        // scales noise according to the amount of time that has passed
        float delta_times = (delta_time / motionModelConfig.process_time);

        if (delta_times < 1) {
            delta_times = 0;
        } else {
            delta_times -= 1;
        }

        float time_noise_multiplier =
          1 + delta_times * motionModelConfig.process_time_noise_factor;

        const Length time_dependent_forwards_noise =
          motionModelConfig.forwards_noise +
          motionModelConfig.angle_to_forwards_noise * abs_delta_theta;

        const Angle time_dependent_angle_noise =
          abs_delta_theta * motionModelConfig.angle_noise;

        const Length time_dependent_drift_noise =
          motionModelConfig.drift_noise +
          motionModelConfig.angle_to_drift_noise * abs_delta_theta;

        const Length distance_noise =
          time_dependent_forwards_noise * time_noise_multiplier +
          motionModelConfig.lost_iter_to_forwards_noise * lost_iteration_count;

        const Angle angle_noise =
          time_dependent_angle_noise * time_noise_multiplier +
          motionModelConfig.lost_iter_to_angle_noise * lost_iteration_count;

        const Length drift_noise =
          time_dependent_drift_noise * time_noise_multiplier +
          motionModelConfig.lost_iter_to_drift_noise * lost_iteration_count;

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

        forwards_distribution = Vuniform_float32_t((-distance_noise).internal(),
                                                   (+distance_noise).internal(),
                                                   robot_rng());
        angle_a = (-angle_noise).internal();
        angle_b = (+angle_noise).internal();
        angle_k = (angle_b - angle_a) / static_cast<float>(UINT32_MAX);

        drift_a = (-drift_noise).internal();
        drift_b = (+drift_noise).internal();
        drift_k = (drift_b - drift_a) / static_cast<float>(UINT32_MAX);

        global_pose_delta_x = global_pose_delta.x.internal();
        global_pose_delta_y = global_pose_delta.y.internal();

        return global_pose_delta;
    }

    /**
     * @brief makes a vector of noisy global deltas (in base units)
     *
     * @param result vector where the motion updates get stored
     */
    inline void VnoisyGlobalDelta(float32x4_t* Xresult,
                                  float32x4_t* Yresult) override {
        float32x4_t vertical_noise = forwards_distribution();
        float32x4_t horizontal_noise =
          forwards_distribution.get_float(drift_a, drift_k);
        float32x4_t angle_noise =
          forwards_distribution.get_float(angle_a, angle_k);

        float32x4_t Vsina = vdupq_n_f32(sina);
        float32x4_t Vcosa = vdupq_n_f32(cosa);

        float32x4_t Vnew_sina, Vnew_cosa;
        Vsincos_taylor_delta(angle_noise, Vsina, Vcosa, &Vnew_sina, &Vnew_cosa);

        *Xresult = vdupq_n_f32(global_pose_delta_x);
        *Yresult = vdupq_n_f32(global_pose_delta_y);

        *Xresult = vmlaq_f32(*Xresult, vertical_noise, Vnew_cosa);
        *Yresult = vmlaq_f32(*Yresult, vertical_noise, Vnew_sina);

        *Xresult = vmlsq_f32(*Xresult, horizontal_noise, Vnew_sina);
        *Yresult = vmlaq_f32(*Yresult, horizontal_noise, Vnew_cosa);
    }

    //
    /**
     * @brief odometry change with some noise added for each particle
     *
     * @return Noisy global delta
     */
    Point noisyGlobalDelta() override {
        const Length vertical_noise = average_distance_distribution(rng) * m;
        const Length horizontal_noise = drift_distribution(rng) * m;
        const float angle_noise = angle_distribution(rng);

        float new_sina, new_cosa;
        sincos_taylor_delta(angle_noise, sina, cosa, &new_sina, &new_cosa);

        // rotates forward and sideways movement to face the robot's heading
        return { global_pose_delta.x + vertical_noise * new_cosa -
                   horizontal_noise * new_sina,
                 global_pose_delta.y + vertical_noise * new_sina +
                   horizontal_noise * new_cosa };
    }

    void updateLostIterationCount(int new_count) override {
        lost_iteration_count = static_cast<float>(new_count);
    }
};
} // namespace vexmaps
