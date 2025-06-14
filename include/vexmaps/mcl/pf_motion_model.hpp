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
#include <random>

// TODO: add settings struct to be able to configure settings
//
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

    Vuniform_float32_t forwards_distribution;

    float angle_a = 0;
    float angle_b = 0;
    float angle_k = 0;

    float drift_a = 0;
    float drift_b = 0;
    float drift_k = 0;

    // Vuniform_float32_t Vaverage_distance_distribution;
    // Vuniform_float32_t Vangle_distribution;
    // Vuniform_float32_t Vdrift_distribution;

    Time update_timestamp = 0_sec;

    Angle abs_delta_theta = 0_stDeg;

    float32x4_t Vsina, Vcosa;
    float sina, cosa;

    float global_pose_delta_x, global_pose_delta_y;

    units::Pose last_pose = { INFINITY * m, INFINITY* m, INFINITY* rad },
                global_pose_delta;

    MotionModelConfig motionModelConfig;

    /**
     * @brief Used to get an estimate for the Robot's movements. Owned and
     * managed by this class only.
     */
    std::unique_ptr<LocalizationModel> base_motion_model;

  public:
    PfMotionModel(std::unique_ptr<LocalizationModel> base_motion_model,
                  MotionModelConfig motionModelConfig)
        : base_motion_model(std::move(base_motion_model)),
          motionModelConfig(motionModelConfig),
          forwards_distribution(robot_rng()) {}

    void init() override {
        base_motion_model->init();
    }

    units::Pose getPose() override {
        return base_motion_model->getPose();
    }

    void setPose(units::Pose new_pose) override {
        base_motion_model->setPose(new_pose);
    }

    std::optional<float> getConfidence() override {
        return base_motion_model->getConfidence();
    }

    units::Pose getGlobalPoseDelta() override {
        return base_motion_model->getGlobalPoseDelta();
    }

    units::Pose getLocalPoseDelta() override {
        return base_motion_model->getLocalPoseDelta();
    }

    units::Pose getLastPose() override {
        return base_motion_model->getLastPose();
    }

    Length getDistanceTraveled() override {
        return base_motion_model->getDistanceTraveled();
    }

    Time getTaskDeltaTime() override {
        return base_motion_model->getTaskDeltaTime();
    }

    Time getLatestUpdateTimestamp() override {
        return update_timestamp;
    }

    ~PfMotionModel() override = default;

    // TODO: update with timestamps so that the same pose is not used twice
    // TODO: incorporate confidence value from base motion model to increase or
    // decrease noise
    void update() override {
        // update base motion model first
        base_motion_model->update();

        units::Pose current_pose = base_motion_model->getPose();

        units::Pose global_pose_delta = base_motion_model->getGlobalPoseDelta();

        abs_delta_theta = units::abs(global_pose_delta.orientation);

        const Length distance_noise =
          motionModelConfig.forwards_noise +
          motionModelConfig.angle_to_forwards_noise * abs_delta_theta;

        const Angle angle_noise =
          abs_delta_theta * motionModelConfig.angle_noise;

        const Length drift_noise =
          motionModelConfig.drift_noise +
          motionModelConfig.angle_to_drift_noise * abs_delta_theta;

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

        //
        // vector-related
        // Vaverage_distance_distribution =
        //   Vuniform_float32_t((-distance_noise).internal(),
        //                      (+distance_noise).internal(),
        //                      robot_rng());
        // Vangle_distribution = Vuniform_float32_t((-angle_noise).internal(),
        //                                          (+angle_noise).internal(),
        //                                          robot_rng());
        // Vdrift_distribution = Vuniform_float32_t((-drift_noise).internal(),
        //                                          (+drift_noise).internal(),
        //                                          robot_rng());

        global_pose_delta_x = global_pose_delta.x.internal();
        global_pose_delta_y = global_pose_delta.y.internal();

        // update timestamps
        update_timestamp = from_msec(pros::millis());
    }

    /**
     * @brief makes a vector of noisy global deltas (in base units)
     *
     * @param result vector where the motion updates get stored
     */
    inline void VnoisyGlobalDelta(float32x4_t* Xresult, float32x4_t* Yresult) {
        // float32x4_t vertical_noise = Vaverage_distance_distribution();
        // float32x4_t horizontal_noise = Vdrift_distribution();
        // float32x4_t angle_noise = Vangle_distribution();

        float32x4_t vertical_noise = forwards_distribution();
        float32x4_t horizontal_noise =
          forwards_distribution.get_float(drift_a, drift_k);
        float32x4_t angle_noise =
          forwards_distribution.get_float(angle_a, angle_k);

        float32x4_t Vsina = vdupq_n_f32(sina);
        float32x4_t Vcosa = vdupq_n_f32(cosa);

        float32x4_t Vnew_sina, Vnew_cosa;
        Vsincos_taylor_delta(angle_noise, Vsina, Vcosa, &Vnew_sina, &Vnew_cosa);

        // TODO: check this actually gets inlined, or that pointers dont
        // actually get dereferenced
        //
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
    Point noisyGlobalDelta() {
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
};

} // namespace vexmaps
