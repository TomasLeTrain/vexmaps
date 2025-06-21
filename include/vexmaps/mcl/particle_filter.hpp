#pragma once

#include "pros/rtos.hpp"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
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
    // used for vectorization
    static constexpr size_t remaining_particles = (N - (N % 4));

    Length x[N];
    Length y[N];
    float weights[N];

    std::vector<Sensor*> sensors;

    BasePfMotionModel* motion_model;
    PFConfiguration PFConfig;

    // global pose delta from the base motion model
    units::Pose globalPoseDelta;

    std::uniform_real_distribution<float> field_dist { -wall_length.internal(),
                                                       wall_length.internal() };

    uint64_t start_time;

    units::Pose prediction;

    int lost_iteration_count = 0;

    bool appliedMotionModel;
    bool weightedParticles;
    bool appliedResampling;

    /**
     * @brief number to which all particles sum to. Basically normalizing the
     * particles then multiplying them by this factor
     */
    // const float sum_factor = N;
    const float sum_factor = 1;
    /**
     * @brief average weight of a particle. Effectively we "normalize" but not
     * to 1, instead to sum_factor
     */
    const float average_weight = sum_factor / N;

    Length bordered_wall_length;

    // the sum of the particles before normalization
    float total_weight = 0;
    float max_weight = 0;

    // holds if none of the sensors detect values
    // useful for determining how to handle weights when there is no current
    // update
    bool no_active_sensors = true;
    int active_sensors = 0;

    // -- functions called in update -- //

    void applyMotionModel() {
        globalPoseDelta = units::Pose(0_m, 0_m, 0_stDeg);
        // applies noise regardless if we have new information or not
        // should be fine since the noise is scaled based on time
        auto current_global_delta = motion_model->precompute();

        globalPoseDelta = current_global_delta;

        appliedMotionModel = true;

        if (PFConfig.usingVectorizedMotion) {

            for (size_t i = 0; i < remaining_particles; i += 4) {
                float32x4_t motionDataX, motionDataY;
                motion_model->VnoisyGlobalDelta(&motionDataX, &motionDataY);
                float32x4_t Vx = vld1q_f32((float*)&x[i]);
                float32x4_t Vy = vld1q_f32((float*)&y[i]);

                Vx = vaddq_f32(Vx, motionDataX);
                Vy = vaddq_f32(Vy, motionDataY);

                vst1q_f32((float*)&x[i], Vx);
                vst1q_f32((float*)&y[i], Vy);
            }
            // proccess remaining
            for (size_t i = remaining_particles; i < N; i++) {
                Point noisy_global_delta = motion_model->noisyGlobalDelta();
                x[i] += noisy_global_delta.x;
                y[i] += noisy_global_delta.y;
            }
        } else {
            for (size_t i = 0; i < N; i++) {
                Point noisy_global_delta = motion_model->noisyGlobalDelta();
                x[i] += noisy_global_delta.x;
                y[i] += noisy_global_delta.y;
            }
        }
    }

    void updateSensors() {
        // perform the one time updates on the sensors
        for (auto&& sensor : this->sensors) {
            sensor->update(motion_model->getPose().orientation);
        }
    }

    inline bool outOfField(const size_t i) {
        return x[i] > bordered_wall_length || y[i] > bordered_wall_length ||
               x[i] < -bordered_wall_length || y[i] < -bordered_wall_length;
    }

    void checkOutOfFieldParticles() {
        // TODO: check/fix performance
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
                // we assume the weights are valid (not infinity)
                if (sensor->getVectorized2()) {
                    static constexpr size_t remaining_particles8 =
                      (N - (N % 8));

                    for (size_t i = 0; i < remaining_particles8; i += 8) {
                        float32x4_t Vx1 = vld1q_f32((float*)&x[i]);
                        float32x4_t Vy1 = vld1q_f32((float*)&y[i]);
                        float32x4_t current_weights1 = vld1q_f32(&weights[i]);

                        float32x4_t Vx2 = vld1q_f32((float*)&x[i + 4]);
                        float32x4_t Vy2 = vld1q_f32((float*)&y[i + 4]);
                        float32x4_t current_weights2 =
                          vld1q_f32(&weights[i + 4]);

                        float32x4_t sensor_weights1, sensor_weights2;

                        sensor->Vevaluate2(Vx1,
                                           Vy1,

                                           Vx2,
                                           Vy2,

                                           &sensor_weights1,
                                           &sensor_weights2);

                        current_weights1 =
                          vmulq_f32(current_weights1, sensor_weights1);
                        current_weights2 =
                          vmulq_f32(current_weights2, sensor_weights2);

                        vst1q_f32(&weights[i], current_weights1);
                        vst1q_f32(&weights[i + 4], current_weights2);
                    }

                    // process remaining particles (if any)
                    for (size_t i = remaining_particles8; i < N; i++) {
                        const float sensor_weight =
                          sensor->evaluate(x[i], y[i]);

                        if (std::isfinite(sensor_weight)) {
                            weights[i] *= sensor_weight;
                        }
                    }
                } else if (sensor->getVectorized()) {
                    for (size_t i = 0; i < remaining_particles; i += 4) {
                        float32x4_t Vx = vld1q_f32((float*)&x[i]);
                        float32x4_t Vy = vld1q_f32((float*)&y[i]);
                        float32x4_t current_weights = vld1q_f32(&weights[i]);

                        float32x4_t sensor_weights = sensor->Vevaluate(Vx, Vy);

                        current_weights =
                          vmulq_f32(current_weights, sensor_weights);

                        vst1q_f32(&weights[i], current_weights);
                    }

                    // process remaining particles (if any)
                    for (size_t i = remaining_particles; i < N; i++) {
                        const float sensor_weight =
                          sensor->evaluate(x[i], y[i]);

                        if (std::isfinite(sensor_weight)) {
                            weights[i] *= sensor_weight;
                        }
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

        float weighted_x_sum = 0.0;
        float weighted_y_sum = 0.0;

        // for (size_t i = 0; i < N; i++) {
        //     weighted_x_sum += x[i].internal() * weights[i];
        //     weighted_y_sum += y[i].internal() * weights[i];
        // }

        if (PFConfig.usingVectorizedMotion) {
            float32x4_t Vweighted_x_sum = vdupq_n_f32(0.0);
            float32x4_t Vweighted_y_sum = vdupq_n_f32(0.0);

            for (size_t i = 0; i < remaining_particles; i += 4) {
                float32x4_t Vx = vld1q_f32((float*)&x[i]);
                float32x4_t Vy = vld1q_f32((float*)&y[i]);
                float32x4_t current_weights = vld1q_f32(&weights[i]);

                Vweighted_x_sum =
                  vmlaq_f32(Vweighted_x_sum, Vx, current_weights);
                Vweighted_y_sum =
                  vmlaq_f32(Vweighted_y_sum, Vy, current_weights);
            }

            for (size_t i = remaining_particles; i < N; i++) {
                weighted_x_sum += x[i].internal() * weights[i];
                weighted_x_sum += y[i].internal() * weights[i];
            }

            weighted_x_sum += vgetq_lane_f32(Vweighted_x_sum, 0) +
                              vgetq_lane_f32(Vweighted_x_sum, 1) +
                              vgetq_lane_f32(Vweighted_x_sum, 2) +
                              vgetq_lane_f32(Vweighted_x_sum, 3);

            weighted_y_sum += vgetq_lane_f32(Vweighted_y_sum, 0) +
                              vgetq_lane_f32(Vweighted_y_sum, 1) +
                              vgetq_lane_f32(Vweighted_y_sum, 2) +
                              vgetq_lane_f32(Vweighted_y_sum, 3);
        } else {
            for (size_t i = 0; i < N; i++) {
                weighted_x_sum += x[i].internal() * weights[i];
                weighted_x_sum += y[i].internal() * weights[i];
            }
        }

        if (active_sensors >= 2) {
            // updates prediction before resampling, as resampling sets all
            // weights to 1/N whcih can significantly shift the prediction
            updatePrediction((weighted_x_sum / sum_factor) * m,
                             (weighted_y_sum / sum_factor) * m,
                             motion_model->getPose().orientation);
        } else {
            // updating the prediction with only one sensor might be a bad idea,
            // as it might be heavily biased towards that specific sensor. it
            // might be better that the prediction gets affected by the
            // new distribution of particles after resampling
            // TODO: see if it would be possble to determine what axis a sensor
            // "locks" and using the weighted sum to determine that axis
            // basically simulating a distance sensor reset
            // this might be impossible since the wall being detected (and
            // therefore the axis) can change between particles
            //
            // we always need to update the prediction even if not based on
            // particles
            updatePrediction(getPose().x + globalPoseDelta.x,
                             getPose().y + globalPoseDelta.y,
                             getPose().orientation +
                               globalPoseDelta.orientation);
        }
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

    void endUpdate() {
        if (PFConfig.logging) {
            printf("total weight: %f, time taken: %d, timestamp: ",
                   total_weight,
                   pros::micros() - start_time);
            printf("%d\n", pros::millis());
            printf("things done:%d,%d,%d\n",
                   this->appliedMotionModel,
                   this->weightedParticles,
                   this->appliedResampling);
            printf("prediction:%f,%f,%f\n",
                   this->prediction.x.convert(in),
                   this->prediction.y.convert(in),
                   this->prediction.orientation.convert(deg));
            printf("end generation\n");
        }
    }

    void updatePrediction(Length x, Length y, Angle angle) {
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

    units::Pose getPose() {
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

        if (no_active_sensors) {
            // there is nothing we can do in this iteration
            // instead we just update the prediction using the deltas from the
            // base motion model

            // if there is no update deltas since last time globalPoseDelta will
            // be zero so it won't matter
            updatePrediction(getPose().x + globalPoseDelta.x,
                             getPose().y + globalPoseDelta.y,
                             getPose().orientation +
                               globalPoseDelta.orientation);
            endUpdate();
            return;
        }

        // all routines from here on assume at least one sensor has a reading

        checkOutOfFieldParticles();

        weightParticles();
        weightedParticles = true;

        total_weight = 0;
        max_weight = 0;

        // likely vectorized
        for (size_t i = 0; i < N; i++) {
            total_weight += weights[i];
        }

        // should be pretty fast because of hard abi
        for (size_t i = 0; i < N; i++) {
            max_weight = fmaxf(max_weight, weights[i]);
        }

        // TODO: this might not be the best metric since it might depend on how
        // many sensor there are and their pdf. it would be better to have a
        // metric for how good the best guesses are
        // Maybe using max_weight instead could be better
        if (active_sensors >= 2) {
            if (total_weight <= PFConfig.low_weight_sum_threshold) {
                // none of the particles are likely at all, meaning we have no
                // clue where the robot could be
                lost_iteration_count++;

                printf(
                  "No particles are likely: sum is: %f, threshold is: " "%f\n, " "lost " "iterat" "ion " "count " "now: " "%d",
                  total_weight,
                  PFConfig.low_weight_sum_threshold,
                  lost_iteration_count);
            } else {
                // we are not lost this iteration
                // (and we have enough sensors to accurately determine this),
                // so reset the lost iteration count
                lost_iteration_count = 0;
            }
        }
        motion_model->updateLostIterationCount(lost_iteration_count);

        // this would allow us to change normalization to make the particles sum
        // to a different number this could affect how much previous weights /
        // current weights affect the final weights
        const float normalization_factor = sum_factor / total_weight;

        // normalizes weights to add up to sum_factor
        // most likely vectorized
        for (size_t i = 0; i < N; i++) {
            weights[i] *= normalization_factor;
        }

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

        int zero_particles = 0;

        // effective_sample_size = sum(w[i]) / sum (w[i]^2) -> 1 / sum (w[i]^2)
        float ess = 0;

        if (PFConfig.usingVectorizedMotion) {
            float32x4_t Vess = vdupq_n_f32(0.0);

            for (size_t i = 0; i < remaining_particles; i += 4) {
                float32x4_t Vweights = vld1q_f32(&weights[i]);

                Vess = vmlaq_f32(Vess, Vweights, Vweights);
            }

            for (size_t i = remaining_particles; i < N; i++) {
                ess += weights[i] * weights[i];
            }

            ess += vgetq_lane_f32(Vess, 0) + vgetq_lane_f32(Vess, 1) +
                   vgetq_lane_f32(Vess, 2) + vgetq_lane_f32(Vess, 3);

        } else {
            for (size_t i = 0; i < N; i++) {
                ess += weights[i] * weights[i];
            }
        }

        ess = sum_factor / ess;

        bool resampling = false;

        if (ess <
            PFConfig.near_zero_particle_percentage * static_cast<float>(N)) {
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
     * @param point Point around which particles are to be initialized
     * @param std_deviation a measure of how dispersed the particles would be. A
     * greater value will cause particles to deviate more from the reference
     * point.
     *
     * @b Example
     * @code {.cpp}
     * Point start_point = {10.0_in,5.0_in};
     * particle_filter.init_normal_around_point(start_point, 5_in);
     * @endcod
     */
    void initNormal(const units::Pose pose, const Length std_deviation) {
        std::normal_distribution x_dist(pose.x.internal(),
                                        std_deviation.internal());
        std::normal_distribution y_dist(pose.y.internal(),
                                        std_deviation.internal());
        for (size_t i = 0; i < N; i++) {
            x[i] = x_dist(rng) * m;
            y[i] = y_dist(rng) * m;
        }
        for (size_t i = 0; i < N; i++) {
            weights[i] = average_weight;
        }
        // need to update motion model as well
        motion_model->setPose(pose);
    }

    void initUniform(const Length min_x,
                     const Length min_y,
                     const Length max_x,
                     const Length max_y,
                     const Angle orientation) {
        std::uniform_real_distribution x_dist(min_x.internal(),
                                              max_x.internal());
        std::uniform_real_distribution y_dist(min_y.internal(),
                                              max_y.internal());

        for (size_t i = 0; i < N; i++) {
            x[i] = x_dist(rng) * m;
            y[i] = y_dist(rng) * m;
        }
        for (size_t i = 0; i < N; i++) {
            weights[i] = average_weight;
        }
        const Length avg_x = (max_x + min_x) / 2.0;
        const Length avg_y = (max_y + min_y) / 2.0;

        // need to update motion model as well
        motion_model->setPose({ avg_x, avg_y, orientation });
    }

    void init() {
        for (size_t i = 0; i < N; i++) {
            x[i] = 0.0_m;
            y[i] = 0.0_m;
            weights[i] = average_weight;
        }

        initUniform(-wall_length,
                    -wall_length,
                    wall_length,
                    wall_length,
                    0_stDeg);
    }

    std::optional<float> getConfidence() {
        return std::nullopt;
    }

    // uses the motion model distance traveled since its less noisy
    Length getDistanceTraveled() {
        return motion_model->getDistanceTraveled();
    }
};

} // namespace vexmaps
