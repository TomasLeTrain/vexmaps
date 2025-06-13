#pragma once

#include "pros/rtos.hpp"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/units.hpp"
#include "vexmaps/mcl/config.hpp"
#include "vexmaps/mcl/pf_motion_model.hpp"
#include "vexmaps/mcl/sensor.hpp"
#include <arm_neon.h>

namespace vexmaps {

template<size_t N>
class ParticleFilter {
  private:
    // used for vectorization
    static constexpr size_t remaining_particles = (N - (N % 4));

    Point particles[N];
    float weights[N];

    std::vector<std::unique_ptr<Sensor>> sensors;

    PfMotionModel* motion_model;

    std::uniform_real_distribution<float> field_dist { -wall_length.internal(),
                                                       wall_length.internal() };

    // TODO: make this configurable
    std::uniform_real_distribution<float> cloud_dist { -(2_in).internal(),
                                                       (2_in).internal() };

    uint64_t start_time;

    units::Pose prediction;

    int lost_iteration_count = 0;

    /**
     * @brief number to which all particles sum to. Basically normalizing the
     * particles then multiplying them by this factor
     */
    const float sum_factor = N;
    /**
     * @brief average weight of a particle. Effectively we "normalize" but not
     * to 1, instead to sum_factor
     */
    const float average_weight = sum_factor / N;

    // the sum of the particles before normalization
    float total_weight = 0;

    // holds if none of the sensors detect values
    // useful for determining how to handle weights when there is no current
    // update
    bool no_active_sensors = true;
    int active_sensors = 0;

    Time last_motion_model_timestamp;

    // -- functions called in update -- //

    void applyMotionModel() {
        // only applies motion updates if there are new available deltas
        auto current_timestamp = motion_model->getLatestUpdateTimestamp();
        auto current_global_delta = motion_model->getGlobalPoseDelta();

        if (current_timestamp == last_motion_model_timestamp &&
            current_global_delta.has_value()) {
            // same update from before or no update available
            // either way don't move particles
            return;
        }
        last_motion_model_timestamp = current_timestamp;

        if (usingVectorizedMotion) {

            for (size_t i = 0; i < remaining_particles; i += 4) {
                float32x4_t motionDataX, motionDataY;
                motion_model->VnoisyGlobalDelta(&motionDataX, &motionDataY);
                // points need to be located in memory like so
                // [x][y][x][y]...
                float32x4x2_t data = vld2q_f32((float*)&particles[i]);

                data.val[0] = vaddq_f32(data.val[0], motionDataX);
                data.val[1] = vaddq_f32(data.val[1], motionDataY);

                vst2q_f32((float*)&particles[i], data);
            }
            // proccess remaining
            for (size_t i = remaining_particles; i < N; i++) {
                particles[i] += motion_model->noisyGlobalDelta();
            }
        } else {
            for (size_t i = 0; i < N; i++) {
                particles[i] += motion_model->noisyGlobalDelta();
            }
        }
    }

    void updateSensors() {
        if (localization_settings::logging) printf("start distances\n");

        // perform the one time updates on the sensors
        for (auto&& sensor : this->sensors) {
            sensor->update(motion_model->getPose().orientation);
        }

        if (localization_settings::logging) printf("end distances\n");
    }

    inline static bool outOfField(const Point& point) {
        return point.x > wall_length || point.x < -wall_length ||
               point.y < -wall_length || point.y > wall_length;
    }

    void checkOutOfFieldParticles() {
        for (size_t i = 0; i < N; i++) {
            // places the particle randomly on the field if its out of the field
            if (outOfField(particles[i])) {
                // useful if the robot is ramming into a wall, as that will stop
                // them from going off the map while also not leaving the
                // current robot's spot
                particles[i] = {
                    units::clamp(particles[i].x,
                                 -(wall_length - 3.5_in),
                                 (wall_length - 3.5_in)),
                    units::clamp(particles[i].y,
                                 -(wall_length - 3.5_in),
                                 (wall_length - 3.5_in)),
                };
            }
        }
    }

    void weightParticles() {
        // TODO: see if this is vectorized (likely)
        for (size_t i = 0; i < N; i++) {
            weights[i] = 1.0;
        }

        // TODO: see if this could be vectorized in some way - or if its already
        // being vectorized (unlikely)
        for (auto&& sensor : this->sensors) {
            if (sensor->hasAvailableReading()) {
                // we assume the weights are valid (not infinity)
                if (sensor->vectorized) {
                    for (size_t i = 0; i < remaining_particles; i += 4) {
                        float32x4x2_t points = vld2q_f32((float*)&particles[i]);
                        float32x4_t current_weights = vld1q_f32(&weights[i]);

                        float32x4_t sensor_weights = sensor->Vevaluate(points);

                        current_weights =
                          vmulq_f32(current_weights, sensor_weights);

                        vst1q_f32(&weights[i], current_weights);
                    }

                    // process remaining particles (if any)
                    for (size_t i = remaining_particles; i < N; i++) {
                        const auto sensor_weight =
                          sensor->evaluate(particles[i]);

                        if (sensor_weight.has_value() &&
                            isfinite(sensor_weight.value())) {
                            weights[i] *= sensor_weight.value();
                        }
                    }
                } else {
                    for (size_t i = 0; i < N; i++) {
                        const auto sensor_weight =
                          sensor->evaluate(particles[i]);

                        if (sensor_weight.has_value() &&
                            isfinite(sensor_weight.value())) {
                            weights[i] *= sensor_weight.value();
                        }
                    }
                }
            }
        }
    }

    void updatePredictionBasedOnParticles() {

        // Length weighted_x_sum = 0.0_m;
        // Length weighted_y_sum = 0.0_m;
        //
        // for (size_t i = 0; i < N; i++) {
        //     weighted_x_sum += particles[i].x * weights[i];
        //     weighted_y_sum += particles[i].y * weights[i];
        // }

        float weighted_x_sum = 0.0;
        float weighted_y_sum = 0.0;


        if (usingVectorizedMotion) {
            float32x4_t Vweighted_x_sum = vdupq_n_f32(0.0);
            float32x4_t Vweighted_y_sum = vdupq_n_f32(0.0);

            for (size_t i = 0; i < remaining_particles; i += 4) {
                float32x4x2_t points = vld2q_f32((float*)&particles[i]);
                float32x4_t current_weights = vld1q_f32(&weights[i]);

                Vweighted_x_sum =
                  vmlaq_f32(Vweighted_x_sum, points.val[0], current_weights);
                Vweighted_y_sum =
                  vmlaq_f32(Vweighted_y_sum, points.val[1], current_weights);
            }

            for (size_t i = remaining_particles; i < N; i++) {
                weighted_x_sum += particles[i].x.internal() * weights[i];
                weighted_x_sum += particles[i].y.internal() * weights[i];
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
                weighted_x_sum += particles[i].x.internal() * weights[i];
                weighted_x_sum += particles[i].y.internal() * weights[i];
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
            // this might be impossible since the wall being detected and
            // therefore the axis can change between particles
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

            particles[i].x = particles[I].x;
            particles[i].y = particles[I].y;

            // sets weight to average value
            weights[i] = average_weight;
        }
    }

    // used for recovering the system when all the particles are not close to
    // correct
    void makeCloudAroundPrediction() {
        // considers the number of iterations its been lost
        // the larger it is the larger the cloud becomes
        const float lost_literation_factor = 0.25;
        const float lost_iteration_multiplier =
          1 +
          lost_literation_factor * static_cast<float>(lost_iteration_count - 5);

        for (size_t i = 0; i < N; i++) {
            // here we can use slow rng since we expect to call this only a few
            // times
            particles[i].x =
              prediction.x + cloud_dist(rng) * m * lost_iteration_multiplier;
            particles[i].y =
              prediction.y + cloud_dist(rng) * m * lost_iteration_multiplier;
        }
        for (size_t i = 0; i < N; i++) {
            weights[i] = average_weight;
        }
    }

    void endUpdate() {
        if (localization_settings::logging) {
            printf("total weight: %f, time taken: %d, timestamp: %d\n",
                   total_weight,
                   pros::micros() - start_time,
                   pros::millis());
            printf("prediction:%f,%f,%f\n",
                   this->prediction.x.convert(in),
                   this->prediction.y.convert(in),
                   this->prediction.orientation.convert(deg));
        }
        if (localization_settings::logging) printf("end generation\n");
    }

  public:
    // managed by the base motion model
    ParticleFilter(PfMotionModel* motionModel,
                   std::vector<std::unique_ptr<Sensor>>&& sensors)
        : motion_model(motionModel),
          sensors(std::move(sensors)) {
        for (size_t i = 0; i < N; i++) {
            particles[i] = { 0.0_m, 0.0_m };
            weights[i] = average_weight;
        }
    }

    // util functions
    void addSensor(Sensor* sensor) {
        sensors.emplace_back(sensor);
    }

    void updatePrediction(Length x, Length y, Angle angle) {
        prediction.x = x;
        prediction.y = y;
        prediction.orientation = angle;
    }

    units::Pose getPrediction() {
        return prediction;
    }

    void update() {
        start_time = pros::micros();

        applyMotionModel();

        if (localization_settings::logging) printf("start generation\n");

        updateSensors();

        bool resampling = false;

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
            auto globalPoseDelta = motion_model->getGlobalPoseDelta();
            if (globalPoseDelta.has_value()) {
                updatePrediction(getPrediction().x + globalPoseDelta.value().x,
                                 getPrediction().y + globalPoseDelta.value().y,
                                 getPrediction().orientation +
                                   globalPoseDelta.value().orientation);
            }
            endUpdate();
            return;
        }
        // all routines from here on assume at least one sensor has a reading

        if (active_sensors >= 2 && lost_iteration_count >= 5) {
            // to recover the system we distribute particles around the
            // prediction to clamp to the actual position we only perform this
            // if we have enough data to determine the new position and if we
            // have been lost for multiple iterations (to avoid clamping because
            // of innacurate readings)
            makeCloudAroundPrediction();
        }

        checkOutOfFieldParticles();

        weightParticles();

        total_weight = 0;

        // update total_weight
        // TODO: check for vectorization (most likely is)
        for (size_t i = 0; i < N; i++) {
            total_weight += weights[i];
        }

        // only check for lost iterations if we have at least two distance
        // sensors
        // TODO: come up with a better metric for the accuracy of particles
        if (active_sensors >= 2) {
            if (total_weight <=
                localization_settings::low_weight_sum_threshold) {
                // none of the particles are likely at all, meaning we have no
                // clue where the robot could be
                lost_iteration_count++;

                printf(
                  "No particles are likely: sum is: %f, threshold is: " "%f\n, " "lost " "iterat" "ion " "count " "now: " "%d",
                  total_weight,
                  localization_settings::low_weight_sum_threshold,
                  lost_iteration_count);
            } else {
                // we are not lost this iteration
                // (and we have enough sensors to accurately determine this),
                // so reset the lost iteration count
                lost_iteration_count = 0;
            }
        }

        // this would allow us to change normalization to make the particles sum
        // to a different number this could affect how much previous weights /
        // current weights affect the final weights
        const float normalization_factor = sum_factor / total_weight;

        // normalizes weights to add up to sum_factor
        // TODO: check for vectorization (most likely is)
        for (size_t i = 0; i < N; i++) {
            weights[i] *= normalization_factor;
        }

        if (localization_settings::logging) {
            printf("start particles\n");
            if (localization_settings::particle_logging) {
                for (size_t i = 0; i < N; i++) {
                    printf("%d:%.2f,%.2f,%.2f\n",
                           i,
                           particles[i].x.convert(in),
                           particles[i].y.convert(in),
                           weights[i]);
                }
            }
            printf("end particles\n");
        }

        updatePredictionBasedOnParticles();

        int zero_particles = 0;

        // TODO: switch to a better metric for non-contributing particles
        // base the near zero particle percentage only on non sensor
        // generated particles, as we would like to resample based on their
        // accuracy, not the generated sensor particles
        for (size_t i = 0; i < N; i++) {
            if (weights[i] < localization_settings::near_zero_epsilon) {
                zero_particles++;
            }
        }

        if (static_cast<float>(zero_particles) >
            localization_settings::near_zero_particle_percentage *
              static_cast<float>(N)) {
            resampling = true;
        }

        if (resampling) {
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
    void init_normal_around_point(const units::Pose pose,
                                  const Length std_deviation) {
        std::normal_distribution x_dist(pose.x.internal(),
                                        std_deviation.internal());
        std::normal_distribution y_dist(pose.y.internal(),
                                        std_deviation.internal());
        for (size_t i = 0; i < N; i++) {
            particles[i].x = x_dist(rng) * m;
            particles[i].y = y_dist(rng) * m;
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
            particles[i].x = x_dist(rng) * m;
            particles[i].y = y_dist(rng) * m;
        }
        for (size_t i = 0; i < N; i++) {
            weights[i] = average_weight;
        }
        const Length avg_x = (max_x + min_x) / 2.0;
        const Length avg_y = (max_y + min_y) / 2.0;

        // need to update motion model as well
        motion_model->setPose({ avg_x, avg_y, orientation });
    }
};

// template<size_t number_of_particles>
// static void init_localization_task(ParticleFilter<number_of_particles>&
// particle_filter){
//     uint32_t start_time = 0;
//     pros::Task localization_task = pros::Task([&] {
//         while(true){
//             start_time = pros::millis();
//
//             particle_filter.update();
//
//             pros::c::task_delay_until(&start_time, 10);
//         }
//     });
// }
} // namespace vexmaps
