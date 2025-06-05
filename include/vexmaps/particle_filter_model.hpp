#pragma once

#include "vexmaps/localization_model.hpp"
#include "vexmaps/mcl/particle_filter.hpp"
#include <memory>

// TODO: implement all the required methods
namespace vexmaps {
template<size_t N>
class ParticleFilterModel : public LocalizationModel {
  private:
    Length set_pose_normal_deviation = 2_in;
    Time taskDeltaTime = 10.0_msec;

    ParticleFilter<N> particle_filter;
  public:

    ParticleFilterModel(std::vector<std::unique_ptr<Sensor>>&& sensors)
        : particle_filter(std::move(sensors)) {}

    void changeSetPoseNormalDeviation(Length new_stdev) {
        set_pose_normal_deviation = new_stdev;
    }

    void init() override {
        particle_filter.initUniform(-1.78308_m,
                                    -1.78308_m,
                                    1.78308_m,
                                    1.78308_m);
    }

    Time getTaskDeltaTime() override { return taskDeltaTime; }

    void update() override {
        particle_filter.update();
    }

    units::Pose getPose() override {
        return particle_filter.getPrediction();
    }

    void setPose(units::Pose new_pose) override {
        particle_filter.init_normal_around_point({ new_pose.x, new_pose.y },
                                                 set_pose_normal_deviation);
    }

    std::optional<float> getConfidence() override {
        return std::nullopt;
    }

    Length getDistanceTraveled() override {
        return 1_in;
    }

    Time getLatestUpdateTimestamp() override {
        return 0_sec;
    }

    ~ParticleFilterModel() override = default;
};
}; // namespace vexmaps
