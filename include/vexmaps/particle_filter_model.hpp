#pragma once

#include "vexmaps/localization_model.hpp"
#include "vexmaps/mcl/config.hpp"
#include "vexmaps/mcl/particle_filter.hpp"
#include "vexmaps/mcl/pf_motion_model.hpp"
#include "vexmaps/mcl/utils.hpp"
#include <memory>

// TODO: implement all the required methods
namespace vexmaps {
template<size_t N>
class ParticleFilterModel : public LocalizationModel {
  private:
    Length set_pose_normal_deviation = 2_in;
    Time taskDeltaTime = 10.0_msec;
    Time latest_update_time = 0.0_sec;

    ParticleFilter<N> particle_filter;

    units::Pose last_pose;
    units::Pose global_delta;
    units::Pose local_delta;

  public:
    ParticleFilterModel(BasePfMotionModel* motion_model,
                        std::vector<Sensor*>&& sensors,
                        PFConfiguration config)
        : particle_filter(motion_model, std::move(sensors), config) {}

    void changeSetPoseNormalDeviation(Length new_stdev) {
        set_pose_normal_deviation = new_stdev;
    }

    void init() override {
        particle_filter.init();
    }

    Time getTaskDeltaTime() override {
        return taskDeltaTime;
    }

    void update() override {
        particle_filter.update();

        units::Pose curr_pose = getPose();

        global_delta =
          units::Pose(curr_pose - last_pose,
                      curr_pose.orientation - last_pose.orientation);

        Angle avg_angle = (curr_pose.orientation + last_pose.orientation) / 2.0;

        local_delta = globalToLocalDelta(global_delta, avg_angle);

        latest_update_time = from_msec(pros::millis());

        last_pose = getPose();
    }

    units::Pose getPose() override {
        return particle_filter.getPose();
    }

    void setPose(units::Pose new_pose) override {
        particle_filter.initNormal(new_pose, set_pose_normal_deviation);
    }

    void setPoseUniform(const Length min_x,
                        const Length min_y,
                        const Length max_x,
                        const Length max_y,
                        const Angle orientation) {
        particle_filter.initUniform(min_x, min_y, max_x, max_y, orientation);
    }

    std::optional<float> getConfidence() override {
        return std::nullopt;
    }

    Length getDistanceTraveled() override {
        return particle_filter.getDistanceTraveled();
    }

    Time getLatestUpdateTimestamp() override {
        return latest_update_time;
    }

    ~ParticleFilterModel() override = default;

    /**
     * @brief gets the previous available pose
     */
    units::Pose getLastPose() override {
        return last_pose;
    }

    /**
     * @brief Get latest global pose delta
     */
    units::Pose getGlobalPoseDelta() override {
        return global_delta;
    }

    /**
     * @brief Get latest local pose delta
     */
    units::Pose getLocalPoseDelta() override {
        return local_delta;
    }
};
}; // namespace vexmaps
