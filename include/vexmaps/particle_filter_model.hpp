#pragma once

#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
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
    Length set_pose_normal_deviation = 2_in;
    Time taskDeltaTime = 10.0_msec;
    Time latest_update_time = 0.0_sec;

    ParticleFilter<N> particle_filter;

    units::Pose last_pose;
    units::Pose global_delta;
    units::Pose local_delta;

    Length forward_travel = 0_m;
    units::V2Velocity local_velocity_vector;
    AngularVelocity angular_velocity;

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

        const units::Pose curr_pose = particle_filter.getPose();

        global_delta =
          units::Pose(curr_pose - last_pose,
                      curr_pose.orientation - last_pose.orientation);

        local_delta = globalToLocalDelta(global_delta, curr_pose.orientation);

        // add forward travel from local delta
        forward_travel += local_delta.x;

        local_velocity_vector = local_delta / getTaskDeltaTime();
        angular_velocity = global_delta.orientation / getTaskDeltaTime();

        latest_update_time = from_msec(pros::millis());

        last_pose = curr_pose;
    }

    void setPose(units::Pose new_pose) override {
        std::lock_guard lock(m_mutex);
        particle_filter.initNormal(new_pose, set_pose_normal_deviation);
    }

    void setPoseUniform(const Length min_x,
                        const Length min_y,
                        const Length max_x,
                        const Length max_y,
                        const Angle orientation) {
        std::lock_guard lock(m_mutex);
        particle_filter.initUniform(min_x, min_y, max_x, max_y, orientation);
    }

    void changeSetPoseNormalDeviation(Length new_stdev) {
        set_pose_normal_deviation = new_stdev;
    }

    // getters
    Time getTaskDeltaTime() override {
        return taskDeltaTime;
    }

    units::Pose getPose() override {
        return particle_filter.getPose();
    }

    std::optional<float> getConfidence() override {
        return particle_filter.getConfidence();
    }

    Length getDistanceTraveled() override {
        return particle_filter.getDistanceTraveled();
    }

    Time getLatestUpdateTimestamp() override {
        return latest_update_time;
    }

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

    /**
     * @brief Disables the particle filter from applying filtering
     */
    void setDisabled(bool new_state) {
        std::lock_guard lock(m_mutex);
        particle_filter.setDisabled(new_state);
    }

    /**
     * @brief Returns the disabled state of the particle filter
     */
    bool getDisabled() {
        return particle_filter.getDisabled();
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

    void setCustomParticles(
      std::vector<std::pair<units::V2FPosition, float>> newParticles) {
        std::lock_guard lock(m_mutex);
        particle_filter.setCustomParticles(newParticles);
    }

    void setCustomPrediction(units::FPose pose) {
        std::lock_guard lock(m_mutex);
        particle_filter.setCustomPrediction(pose);
    }

    void setCustomData(std::string data) {
        std::lock_guard lock(m_mutex);
        particle_filter.setCustomData(data);
    }

    void setReferenceModel(LocalizationModel* model) {
        particle_filter.setReferenceModel(model);
    }

    ~ParticleFilterModel() override = default;
};
}; // namespace vexmaps
