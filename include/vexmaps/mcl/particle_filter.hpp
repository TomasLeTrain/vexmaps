#pragma once

#include "pros/rtos.hpp"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include "vexmaps/mcl/config.hpp"
#include "vexmaps/mcl/pf_motion_model.hpp"
#include "vexmaps/mcl/sensor.hpp"
#include "vexmaps/mcl/utils.hpp"
#include <arm_neon.h>

namespace vexmaps {

template<size_t N>
class ParticleFilter {
  private:
    // divisible by 4,8,16, and 24
    static constexpr size_t block_size = 48;
    static constexpr size_t max_size = block_size * (N / block_size + 1);

    // aligned to 16 bytes (4 floats)
    alignas(16) FLength x[max_size];
    alignas(16) FLength y[max_size];
    alignas(16) float weights[max_size];

    // used for sensor operations
    alignas(16) float tmp_list1[max_size];
    alignas(16) float tmp_list2[max_size];

    BasePfMotionModel* motion_model;
    PFConfiguration PFConfig;
    std::vector<Sensor*> sensors;

    // global pose delta from the base motion model
    units::FPose globalPoseDelta;

    std::uniform_real_distribution<float> field_dist { -wall_length.internal(),
                                                       wall_length.internal() };

    uint64_t start_time;

    units::FPose prediction;

    int lost_iteration_count = 0;

    bool appliedMotionModel;
    bool weightedParticles;
    bool appliedResampling;

    /**
     * @brief number to which all particles sum to. Basically normalizing the
     * particles then multiplying them by this factor
     */
    // const float sum_factor = N;
    const float sum_factor = N;
    /**
     * @brief average weight of a particle. Effectively we "normalize" but not
     * to 1, instead to sum_factor
     */
    const float average_weight = sum_factor / N;

    FLength bordered_wall_length;

    // the sum of the particles before normalization
    float total_weight = 0;
    // maximum weight of normalized particles
    float max_unnormalized_weight = 0;
    float max_weight = 0;

    // holds if none of the sensors detect values
    // useful for determining how to handle weights when there is no current
    // update
    bool no_active_sensors = true;
    int active_sensors = 0;

    float ess;

    // ensures all updates work off of the same angle even if its not the latest
    FAngle current_angle = 0_FstDeg;

    // set to 1 when we have a global measurement, otherwise nullopt
    std::optional<float> confidence = std::nullopt;

    // disables most of the particle filters actions (still applies noise and
    // motion model to prediction)
    bool disabled = false;

    // -- functions called in update -- //

    void applyMotionModel() {
        globalPoseDelta = units::FPose(0_Fm, 0_Fm, 0_FstDeg);
        // applies noise regardless if we have new information or not
        // should be fine since the noise is scaled based on time
        units::FPose current_global_delta = motion_model->precompute();

        globalPoseDelta = current_global_delta;
        current_angle = motion_model->getPose().orientation;

        appliedMotionModel = true;

        if (PFConfig.usingVectorizedMotion) {
            for (size_t i = 0; i < N; i += 4) {
                float32x4_t motionDataX, motionDataY;
                motion_model->VnoisyGlobalDelta(&motionDataX, &motionDataY);

                float32x4_t Vx = vld1q_f32((float*)&x[i]);
                float32x4_t Vy = vld1q_f32((float*)&y[i]);

                Vx = vaddq_f32(Vx, motionDataX);
                Vy = vaddq_f32(Vy, motionDataY);

                vst1q_f32((float*)&x[i], Vx);
                vst1q_f32((float*)&y[i], Vy);
            }
        } else {
            for (size_t i = 0; i < N; i++) {
                units::V2Position noisy_global_delta =
                  motion_model->noisyGlobalDelta();
                x[i] += noisy_global_delta.x;
                y[i] += noisy_global_delta.y;
            }
        }
    }

    void updateSensors() {
        // perform the one time updates on the sensors
        for (auto&& sensor : this->sensors) {
            sensor->update(current_angle);
        }
    }

    inline bool outOfField(const size_t i) {
        return x[i] > bordered_wall_length || y[i] > bordered_wall_length ||
               x[i] < -bordered_wall_length || y[i] < -bordered_wall_length;
    }

    void checkOutOfFieldParticles() {
        for (size_t i = 0; i < N; i++) {
            // places the particle randomly on the field if its out of the field
            if (outOfField(i)) {
                // useful if the robot is ramming into a wall, as that will stop
                // them from going off the map while also not leaving the
                // current robot's spot
                x[i] = units::clamp(x[i],
                                    -bordered_wall_length,
                                    bordered_wall_length);
                y[i] = units::clamp(y[i],
                                    -bordered_wall_length,
                                    bordered_wall_length);
            }
        }
    }

    void weightParticles() {
        // Likely vectorized
        for (size_t i = 0; i < N; i++) {
            weights[i] = 1.0;
        }

        for (auto&& sensor : this->sensors) {
            if (sensor->hasAvailableReading()) {
                if (sensor->canProcessArray()) {
                    // sensor can process in a list, use
                    sensor->evaluate_array(tmp_list1, x, y, tmp_list2, N);

                    // multiply with actual weights
                    for (int i = 0; i < N; i++) {
                        weights[i] *= tmp_list1[i];
                    }
                }
                // we assume the weights are valid (not infinity)
                else if (sensor->getVectorized()) {
                    for (size_t i = 0; i < N; i += 4) {
                        float32x4_t Vx = vld1q_f32((float*)&x[i]);
                        float32x4_t Vy = vld1q_f32((float*)&y[i]);
                        float32x4_t current_weights = vld1q_f32(&weights[i]);

                        float32x4_t sensor_weights = sensor->Vevaluate(Vx, Vy);

                        current_weights =
                          vmulq_f32(current_weights, sensor_weights);

                        vst1q_f32(&weights[i], current_weights);
                    }
                } else {
                    for (size_t i = 0; i < N; i++) {
                        const float sensor_weight =
                          sensor->evaluate(x[i], y[i]);

                        if (std::isfinite(sensor_weight)) {
                            weights[i] *= sensor_weight;
                        }
                    }
                }
            }
        }
    }

    void updatePredictionBasedOnParticles() {
        if (active_sensors <= 1) {
            updatePrediction(getPose().x + globalPoseDelta.x,
                             getPose().y + globalPoseDelta.y,
                             current_angle);
            return;
        }

        // sum of included particles multiplied by their respective weights
        FLength weighted_x_sum = 0.0_Fm;
        FLength weighted_y_sum = 0.0_Fm;

        // sum of the weights of the particles included in the prediction
        float weight_sum = 0;

        // TODO: vectorize
        const float max_weight_threshold =
          max_weight * PFConfig.weightPredictionFactor;

        for (size_t i = 0; i < N; i++) {
            if (max_weight_threshold <= weights[i]) {
                weighted_x_sum += x[i] * weights[i];
                weighted_y_sum += y[i] * weights[i];
                weight_sum += weights[i];
            }
        }

        // updates prediction before resampling, as resampling sets all
        // weights to 1/N which can significantly shift the prediction
        updatePrediction(weighted_x_sum / weight_sum,
                         weighted_y_sum / weight_sum,
                         current_angle,
                         // sets confidence to 1
                         1);
    }

    // resamples particles using stochastic universal sampling
    void resampleParticles() {
        std::uniform_real_distribution<float> weight_distribution(
          0.0,
          average_weight);
        const float start_weight = weight_distribution(rng);

        // makes pointer[0] = start_weight at the start of the loop
        float pointer = start_weight - average_weight;
        float weight_sum = weights[0];
        size_t I = 0;
        // performs resampling of the particles (does not affect generated ones)
        for (size_t i = 0; i < N; i++) {
            pointer = pointer + average_weight;
            // weight_sum = sum(weights[0...I])
            while (weight_sum < pointer && I <= N - 2) {
                I++;
                weight_sum = weight_sum + weights[I];
            }

            x[i] = x[I];
            y[i] = y[I];

            // sets weight to average value
            weights[i] = average_weight;
        }
    }

    void updateLostIterationCount() {
        if (active_sensors >= 2) {
            if (max_unnormalized_weight < PFConfig.lost_max_weight_threshold) {
                // none of the particles are likely at all, meaning we have no
                // clue where the robot could be
                lost_iteration_count++;
                lost_iteration_count =
                  std::min(lost_iteration_count,
                           PFConfig.max_lost_iteration_count);

                // printf(
                //   "No particles are likely: max is: %f, threshold is: "
                //   "%f\n, " "lost " "iterat" "ion " "count " "now: " "%d",
                //   total_weight,
                //   PFConfig.lost_max_weight_threshold,
                //   lost_iteration_count);
                //
            } else {
                // we are not lost this iteration
                // (and we have enough sensors to accurately determine this),
                // so reset the lost iteration count
                lost_iteration_count = 0;
            }
        }
        motion_model->updateLostIterationCount(lost_iteration_count);
    }

    void endUpdate() {
        if (PFConfig.logging) {
            printf("total weight: %f, time taken: %lld, timestamp: ",
                   total_weight,
                   pros::micros() - start_time);
            printf("%ud\n", pros::millis());
            printf("things done:%d,%d,%d,%d\n",
                   this->appliedMotionModel,
                   this->weightedParticles,
                   this->appliedResampling,
                   N);
            printf("prediction:%f,%f,%f\n",
                   this->prediction.x.convert(in),
                   this->prediction.y.convert(in),
                   this->prediction.orientation.convert(Fdeg));
            printf("end generation\n");
        }
    }

    void updatePrediction(FLength x,
                          FLength y,
                          FAngle angle,
                          std::optional<float> confidence = std::nullopt) {
        // set to nullopt by default
        this->confidence = confidence;

        prediction.x = x;
        prediction.y = y;
        prediction.orientation = angle;
    }

  public:
    // managed by the base motion model
    ParticleFilter(BasePfMotionModel* motionModel,
                   std::vector<Sensor*>&& sensors,
                   PFConfiguration config)
        : motion_model(motionModel),
          PFConfig(config),
          sensors(std::move(sensors)) {
        bordered_wall_length = wall_length - (PFConfig.wall_border_width);
    }

    void addSensor(Sensor* sensor) {
        sensors.emplace_back(sensor);
    }

    units::FPose getPose() {
        return prediction;
    }

    void update() {
        start_time = pros::micros();
        // set flags
        appliedMotionModel = false;
        weightedParticles = false;
        appliedResampling = false;

        // if no movement happened then globalPoseDelta will be all zeroes
        applyMotionModel();

        if (PFConfig.logging) printf("start generation\n");

        if (PFConfig.logging) printf("start distances\n");
        updateSensors();
        if (PFConfig.logging) printf("end distances\n");

        // active if any of the sensors have a reading
        no_active_sensors = true;
        // number of sensors with a reading
        active_sensors = 0;

        for (auto&& sensor : this->sensors) {
            if (sensor->hasAvailableReading()) {
                no_active_sensors = false;
                active_sensors++;
            }
        }

        if (no_active_sensors || disabled) {
            // there is nothing we can do in this iteration
            // instead we just update the prediction using the deltas from the
            // base motion model

            // if there is no update deltas since last time globalPoseDelta will
            // be zero so it won't matter
            updatePrediction(getPose().x + globalPoseDelta.x,
                             getPose().y + globalPoseDelta.y,
                             current_angle);
            endUpdate();
            return;
        }

        // all routines from here on assume at least one sensor has a reading

        checkOutOfFieldParticles();

        weightParticles();
        weightedParticles = true;

        total_weight = 0;
        max_unnormalized_weight = 0;
        max_weight = 0;

        // likely vectorized
        for (size_t i = 0; i < N; i++) {
            total_weight += weights[i];
        }

        // also vectorized?
        for (size_t i = 0; i < N; i++) {
            max_unnormalized_weight =
              std::max(max_unnormalized_weight, weights[i]);
        }

        const float normalization_factor = sum_factor / total_weight;

        // normalizes weights to add up to sum_factor
        // most likely vectorized
        for (size_t i = 0; i < N; i++) {
            weights[i] *= normalization_factor;
        }

        max_weight = max_unnormalized_weight * normalization_factor;

        updateLostIterationCount();

        if (PFConfig.logging) {
            printf("start particles\n");
            if (PFConfig.particle_logging) {
                for (size_t i = 0; i < N; i++) {
                    printf("%.1f %.1f %.1f\n",
                           x[i].convert(in),
                           y[i].convert(in),
                           weights[i] * 100);
                }
            }
            printf("end particles\n");
        }

        updatePredictionBasedOnParticles();

        // int zero_particles = 0;

        bool resampling = false;

        // effective_sample_size = sum(w[i]) / sum (w[i]^2) -> 1 / sum (w[i]^2)
        ess = 0;

        // very likely vectorized
        for (size_t i = 0; i < N; i++) {
            ess += weights[i] * weights[i];
        }

        ess = sum_factor / ess;

        if (ess < PFConfig.near_zero_particle_percentage) {
            resampling = true;
        }

        if (resampling) {
            appliedResampling = true;
            resampleParticles();
        }

        endUpdate();
    }

    /**
     * @brief Initializes particles around a point based on some covariance.
     * Useful for initializing particles around a known starting point.
     *
     * @param pose Pose around which particles are to be initialized
     * @param std_deviation a measure of how dispersed the particles would be. A
     * greater value will cause particles to deviate more from the reference
     * point.
     */
    void initNormal(const units::FPose pose, const FLength std_deviation) {
        std::normal_distribution<float> x_dist(pose.x.internal(),
                                               std_deviation.internal());
        std::normal_distribution<float> y_dist(pose.y.internal(),
                                               std_deviation.internal());
        for (size_t i = 0; i < N; i++) {
            x[i] = x_dist(rng) * Fm;
            y[i] = y_dist(rng) * Fm;
        }
        for (size_t i = 0; i < N; i++) {
            weights[i] = average_weight;
        }
        // need to update motion model as well
        updatePrediction(pose.x, pose.y, pose.orientation);
        motion_model->setPose(pose);
    }

    void initUniform(const FLength min_x,
                     const FLength min_y,
                     const FLength max_x,
                     const FLength max_y,
                     const FAngle orientation) {
        std::uniform_real_distribution<float> x_dist(min_x.internal(),
                                                     max_x.internal());
        std::uniform_real_distribution<float> y_dist(min_y.internal(),
                                                     max_y.internal());

        for (size_t i = 0; i < N; i++) {
            x[i] = x_dist(rng) * Fm;
            y[i] = y_dist(rng) * Fm;
        }
        for (size_t i = 0; i < N; i++) {
            weights[i] = average_weight;
        }
        const FLength avg_x = (max_x + min_x) / 2.0;
        const FLength avg_y = (max_y + min_y) / 2.0;

        // need to update motion model as well
        updatePrediction(avg_x, avg_y, orientation);
        motion_model->setPose({ avg_x, avg_y, orientation });
    }

    void init() {
        for (size_t i = 0; i < N; i++) {
            x[i] = 0.0_Fm;
            y[i] = 0.0_Fm;
            weights[i] = average_weight;
        }

        initUniform(-wall_length,
                    -wall_length,
                    wall_length,
                    wall_length,
                    0_FstDeg);
    }

    std::optional<float> getConfidence() {
        return confidence;
    }

    // uses the motion model distance traveled since its less noisy
    FLength getDistanceTraveled() {
        return motion_model->getDistanceTraveled();
    }

    void setDisabled(bool new_state) {
        disabled = new_state;
    }

    bool getDisabled() {
        return disabled;
    }
};

} // namespace vexmaps
