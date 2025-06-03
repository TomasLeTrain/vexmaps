#pragma once

#include "vexmaps/localization_model.h"
#include "vexmaps/mcl/particle_filter.h"
#include <memory>

namespace vexmaps {
template<size_t N>
class ParticleFilterModel : public LocalizationModel {
  private:
    Length set_pose_normal_deviation = 2_in;
    Time deltaTime = 10.0_msec;

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

    ~ParticleFilterModel() override = default;
};
}; // namespace vexmaps
