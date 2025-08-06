#pragma once

#include "vexmaps/localization_model.hpp"
#include "vexmaps/mcl/config.hpp"
#include "vexmaps/mcl/particle_filter.hpp"
#include "vexmaps/mcl/pf_motion_model.hpp"
#include "vexmaps/mcl/utils.hpp"
#include <memory>
#include <mutex>

// TODO: implement all the required methods
namespace vexmaps {
template<size_t N>
class ParticleFilterModel : public LocalizationModel {
  private:
    FLength set_pose_normal_deviation = 2_in;
    FTime taskDeltaTime = 10.0_msec;
    FTime latest_update_time = 0.0_sec;

    ParticleFilter<N> particle_filter;

    units::FPose last_pose;
    units::FPose global_delta;
    units::FPose local_delta;

  protected:
    mutable pros::Mutex m_mutex;

  public:
    ParticleFilterModel(BasePfMotionModel* motion_model,
                        std::vector<Sensor*>&& sensors,
                        PFConfiguration config)
        : particle_filter(motion_model, std::move(sensors), config) {}

    void init() override {
        std::lock_guard lock(m_mutex);
        particle_filter.init();
    }

    void update() override {
        std::lock_guard lock(m_mutex);
        particle_filter.update();

        units::FPose curr_pose = particle_filter.getPose();

        global_delta =
          units::FPose(curr_pose - last_pose,
                      curr_pose.orientation - last_pose.orientation);

        FAngle avg_angle = (curr_pose.orientation + last_pose.orientation) / 2.0;

        local_delta = globalToLocalDelta(global_delta, avg_angle);

        latest_update_time = from_msec(pros::millis());

        last_pose = curr_pose;
    }

    void setPose(units::FPose new_pose) override {
        std::lock_guard lock(m_mutex);
        particle_filter.initNormal(new_pose, set_pose_normal_deviation);
    }

    void setPoseUniform(const FLength min_x,
                        const FLength min_y,
                        const FLength max_x,
                        const FLength max_y,
                        const FAngle orientation) {
        std::lock_guard lock(m_mutex);
        particle_filter.initUniform(min_x, min_y, max_x, max_y, orientation);
    }

    void changeSetPoseNormalDeviation(FLength new_stdev) {
        set_pose_normal_deviation = new_stdev;
    }

    // getters 
    FTime getTaskDeltaTime() override {
        return taskDeltaTime;
    }

    units::FPose getPose() override {
        return particle_filter.getPose();
    }

    std::optional<float> getConfidence() override {
        return particle_filter.getConfidence();
    }

    FLength getDistanceTraveled() override {
        return particle_filter.getDistanceTraveled();
    }

    FTime getLatestUpdateTimestamp() override {
        return latest_update_time;
    }

    /**
     * @brief gets the previous available pose
     */
    units::FPose getLastPose() override {
        return last_pose;
    }

    /**
     * @brief Get latest global pose delta
     */
    units::FPose getGlobalPoseDelta() override {
        return global_delta;
    }

    /**
     * @brief Get latest local pose delta
     */
    units::FPose getLocalPoseDelta() override {
        return local_delta;
    }

    void setDisabled(bool new_state){
        std::lock_guard lock(m_mutex);
        particle_filter.setDisabled(new_state);
    }

    bool getDisabled(){
        return particle_filter.getDisabled();
    }

    ~ParticleFilterModel() override = default;
};
}; // namespace vexmaps
