#pragma once

#include "units/units.hpp"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "pros/apix.h"
#include "pros/rtos.hpp"
#include <functional>

#include "vexmaps/mcl/motion_model.h"
#include "vexmaps/mcl/sensor.h"
#include "vexmaps/mcl/config.h"


namespace vexmaps {

    template<size_t N>
    class ParticleFilter {
        private:

        Point particles[N];
        float weights[N];

        std::vector<std::unique_ptr<Sensor>> sensors;

        MotionModel* motion_model;

        std::function<Angle()> angleFunction;

        std::uniform_real_distribution<float> field_dist{-wall_length.internal(), wall_length.internal()};

        std::uniform_real_distribution<float> cloud_dist{-(2_in).internal(), (2_in).internal()};

        Length last_distance_since_update = 0_m;

        Length distance_since_update = 0_m;
        Time last_update_time = 0_sec;
        Angle angle_change_since_update = 0_stDeg;

        Length max_distance_since_update = 1.0_in;
        Time max_update_interval = 2_sec;
        Angle max_angle_change_since_update = 30_stDeg;

        units::Pose prediction = {0.0_m,0.0_m,0_stDeg};

        int lost_iteration_count = 0;

        /**
         * @brief number to which all particles sum to. Basically normalizing the particles then multiplying them by this factor
         */
        const float sum_factor = N;
        /**
         * @brief average weight of a particle. Effectively we "normalize" but not to 1, instead to sum_factor
         */
        const float average_weight = sum_factor/N;

        // the sum of the particles before normalization
        float total_weight = 0;

        // holds if none of the sensors detect values
        // useful for determining how to handle weights when there is no current update
        bool no_active_sensors = true;
        int active_sensors = 0;

        // -- functions called in update -- //
        
        void applyMotionModel(){
            // one time motion model update
            motion_model->motionUpdate(this->angle);

            if(usingVectorizedMotion){
                static constexpr size_t remaining_particles = (N - (N%4));

                float32x4x2_t motionData, data;
                for(size_t i = 0;i < remaining_particles; i += 4){
                    motion_model->fastMotionUpdates(&motionData);
                    // points need to be located in memory like so
                    // [x][y][x][y]
                    data = vld2q_f32((float *) &particles[i]);

                    data.val[0] += motionData.val[0];
                    data.val[1] += motionData.val[1];

                    vst2q_f32((float*) &particles[i], data);
                }
                // proccess remaining 
                for(size_t i = remaining_particles; i < N;i++){
                    particles[i] += motion_model->singleParticleMotion();
                }
            } else{
                for(size_t i = 0;i < N;i++){
                    particles[i] += motion_model->singleParticleMotion();
                }
            }

            // TODO: look at this closer
            // distance_since_update = distance_since_update + distance(motion_model->actualOdomChange());
            distance_since_update = motion_model->actualDistanceTraveled() - last_distance_since_update;

            angle_change_since_update = angle_change_since_update + motion_model->getAngleChange();
        }

        void updateSensors(){
            if(localization_settings::logging) printf("start distances\n");
            
            // perform the one time updates on the sensors
            for (auto &&sensor : this->sensors){
                sensor->update(angle);
            }

            if(localization_settings::logging) printf("end distances\n");
            

        }

        static bool outOfField(const Point& point) {
            return point.x > wall_length || point.x < -wall_length || point.y < -wall_length || point.y > wall_length;
        }

        void checkOutOfFieldParticles(){
            for(size_t i = 0; i < N;i++){
                // places the particle randomly on the field if its out of the field
                if(outOfField(particles[i])){
                    // useful if the robot is ramming into a wall, as that will stop them from going off the map while also not leaving the current robot's spot
                    particles[i] = {
                        units::clamp(particles[i].x,-(wall_length-3.5_in),(wall_length-3.5_in)),
                        units::clamp(particles[i].y,-(wall_length-3.5_in),(wall_length-3.5_in)),
                    };
                    // sets their weights so that they dont accidently shift the prediction before being updated
                    weights[i] = near_zero_epsilon/2;
                }
            }
        }

        void weightParticles(int sensors_enabled){
            for(size_t i = 0; i < N;i++){
                float current_weight = 1.0;

                for (auto &&sensor : this->sensors){
                    const auto sensor_weight = sensor->p(particles[i]);
                    
                    // allows underterminate readings through infinity
                    if(sensor_weight.has_value() && isfinite(sensor_weight.value())){
                        current_weight = current_weight * sensor_weight.value();
                    }
                }

                // if there is only one sensor enabled then prediction can become messed up
                // multiplying the values allows us to incorporate the current sensor reading
                // while not messing too much with the prediction
                if(sensors_enabled <= 1) weights[i] = weights[i] * current_weight;
                // if we have more than one reading we can use these weights to replace the current ones
                else weights[i] = current_weight;
            }
        }

        void updatePredictionBasedOnParticles(){
            Length weighted_x_sum = 0.0_m;
            Length weighted_y_sum = 0.0_m;

            // TODO: check for vectorization
            for(size_t i = 0;i < N;i++){
                weighted_x_sum = weighted_x_sum + particles[i].x * weights[i];
                weighted_y_sum = weighted_y_sum + particles[i].y * weights[i];
            }

            // updates prediction before resampling, as resampling sets all weights to 1/N
            // whcih can significantly shift the prediction
            updatePrediction(
                    weighted_x_sum / sum_factor,
                    weighted_y_sum / sum_factor,
                    angle
                    );
        }

        void resampleParticles(){
            std::uniform_real_distribution<float> weight_distribution(0.0, average_weight);
            const float start_weight = weight_distribution(rng);

            // makes pointer[0] = start_weight at the start of the loop
            float pointer = start_weight - average_weight;
            float weight_sum = weights[0];
            size_t I = 0;
            // performs resampling of the particles (does not affect generated ones)
            for(size_t i = 0;i < N;i++){
                pointer = pointer + average_weight;
                // weight_sum = sum(weights[0...I])
                while(weight_sum < pointer && I <= N-2){
                    I++;
                    weight_sum = weight_sum + weights[I];
                }

                particles[i].x = particles[I].x;
                particles[i].y = particles[I].y;
                
                // sets weight to average value
                weights[i] = average_weight;
            }
            // we might want to re-weight the new samples,
            // as we are not guaranteed the next update will have any sensor updates 
        }

        // used for recovering the system when all the particles are not close to correct
        void makeCloudAroundPrediction(){
            // considers the number of iterations its been lost
            // the larger it is the larger the cloud becomes
            const float lost_literation_factor =  0.25;
            const float lost_iteration_multiplier = 1 + lost_literation_factor * static_cast<float>(lost_iteration_count - 5);

            for(size_t i = 0;i < N;i++){
		    // here we can use slow rng since we expect to call this only a few times
                particles[i].x = prediction.x + cloud_dist(rng) * m * lost_iteration_multiplier;
                particles[i].y = prediction.y + cloud_dist(rng) * m * lost_iteration_multiplier;
            }
            for(size_t i = 0;i < N;i++){
                weights[i] = average_weight;
            }

            // update angle of prediction
            updateAngle();
            // ensure an update immediately
            this->distance_since_update = this->max_distance_since_update;
        }

    public:
        Angle angle = 0_stDeg;

        ParticleFilter(
                MotionModel* motionModel,
                std::vector<std::unique_ptr<Sensor>>&& sensors,
                std::function<Angle()> angle_function)
            : motion_model(motionModel),
            sensors(std::move(sensors)),
            angleFunction(angle_function) {
            for(size_t i = 0;i < N;i++){
                particles[i] = {0.0_m, 0.0_m};
                weights[i] = average_weight;
            }
        }

        // util functions
        void addSensor(Sensor * sensor){
            sensors.emplace_back(sensor);
        }

        void updateAngle(){
            this->angle = angleFunction();
        }

        void updatePrediction(Length x, Length y, Angle angle){
            prediction.x = x;
            prediction.y = y;
            prediction.orientation = angle;
        }

        units::Pose getPrediction(){
            return prediction;
        }

        void update(){ 
            updateAngle();

            if (!std::isfinite(angle.internal())) {
                return;
            }

            auto start_time = pros::micros();

            applyMotionModel();

            if (
                    // either the angle has changed enough
                    angle_change_since_update < max_angle_change_since_update &&
                    // or travelled enough distance to warrant an update
                    distance_since_update < max_distance_since_update &&
                    // or enough time has passed to warrant an update
                    max_update_interval > (pros::millis()*msec - last_update_time)
                    ) {
                // we wont update the particles just yet, however we do want to update the prediction accordingly
                // to do this we can just use the actual odom change to move the prediction without having the go through all the particles
                // it also avoids having to consider the randomness of the particles as they are moved
                Point actual_odom_change = motion_model->actualOdomChange();
                updatePrediction(
                    getPrediction().x + actual_odom_change.x,
                    getPrediction().y + actual_odom_change.y,
                    getPrediction().orientation
                );
                return;
            }

            if(localization_settings::logging) printf("start generation\n");

            updateSensors();

            bool resampling = false;

            // active if any of the sensors have a reading
            no_active_sensors = true;
            // number of sensors with a reading
            active_sensors = 0;

            for (auto &&sensor : this->sensors){
                if(!sensor->exit){
                    no_active_sensors = false;
                    active_sensors++;
                }
            }

            if(active_sensors >= 2 && lost_iteration_count >= 5){
                // to recover the system we distribute particles around the prediction to clamp to the actual position
                // we only perform this if we have enough data to determine the new position
                // and if we have been lost for multiple iterations (to avoid clamping because of innacurate readings)
                makeCloudAroundPrediction();
            }

            checkOutOfFieldParticles();

            total_weight = 0;

            // only update weights if there are new readings
            // if there arent any new readings we keep the weights the same, although we do update the prediction
            if(!no_active_sensors){
                weightParticles(active_sensors);
            }

            // update total_weight
            for(size_t i = 0; i < N;i++){
                total_weight = total_weight + weights[i];
            }

            // only check for lost iterations if we have at least two distance sensors
            if(active_sensors >= 2){
                if(total_weight <= localization_settings::low_weight_sum_threshold){
                    // none of the particles are likely at all, meaning we have no clue where the robot could be
                    printf("No particles are likely: sum is: %f, threshold is: %f\n, lost iteration count: %d",
                    total_weight,localization_settings::low_weight_sum_threshold,lost_iteration_count);
                    
                    lost_iteration_count++;
                }else if(active_sensors >= 2){
                    // we are not lost this iteration
                    // (and we have enough sensors to accurately determine this),
                    // so reset the lost iteration count
                    lost_iteration_count = 0;
                }
            }

            // this would allow us to change normalization to make the particles sum to a different number
            // this could affect how much previous weights / current weights affect the final weights
            const float normalization_factor = sum_factor / total_weight;

            // normalizes weights to add up to sum_factor
            // TODO: check for vectorization (most likely is)
            for(size_t i = 0; i < N;i++){
                weights[i] = weights[i] * normalization_factor;
            }

            if(localization_settings::logging){
                printf("start particles\n");
            }
            if(localization_settings::logging && localization_settings::particle_logging){
                for(size_t i = 0; i < N;i++){
                    printf("%d:%.2f,%.2f,%.2f\n",i,particles[i].x.convert(in),particles[i].y.convert(in),
                            weights[i]);
                }
            }
            if(localization_settings::logging){
                printf("end particles\n");
            }

            int zero_particles = 0;

            // TODO: switch to a better metric for non-contributing particles
            // base the near zero particle percentage only on non sensor generated particles,
            // as we would like to resample based on their accuracy, not the generated sensor particles
            for(size_t i = 0;i < N;i++){
                if(weights[i] < localization_settings::near_zero_epsilon){
                    zero_particles++;
                }
            }

            if(static_cast<float>(zero_particles) >
            localization_settings::near_zero_particle_percentage * static_cast<float>(N)
                // we should also have access to two readings, else we might resample when we dont know anything
                && active_sensors > 1
            ){
                resampling = true;
            }

            updatePredictionBasedOnParticles();

            // resamples particles using stochastic universal sampling
            //
            // only resample if we have reached the threshold and there are meaningful weights
            if(resampling && !no_active_sensors){
                resampleParticles();
            }

            if(localization_settings::logging){
                std::cout << "total weight: " << total_weight
                    << ", time taken: " << pros::micros() - start_time
                    << ", timestamp: " << pros::millis()
                    << '\n';

                std::cout << "prediction:"
                    << this->prediction.x.convert(in) << ','
                    << this->prediction.y.convert(in) << ','
                    << this->prediction.orientation.convert(deg) << '\n';
            }

            this->last_update_time = pros::millis() * msec;
            this->last_distance_since_update = distance_since_update;
            this->angle_change_since_update = 0_stDeg;

            if(localization_settings::logging) printf("end generation\n");
        }

        /**
         * @brief Initializes particles around a point based on some covariance.
         * Useful for initializing particles around a known starting point.
         *
         * @param point Point around which particles are to be initialized 
         * @param std_deviation a measure of how dispersed the particles would be. A greater value will cause particles to deviate more from the reference point. 
         *
         * @b Example
         * @code {.cpp}
         * Point start_point = {10.0_in,5.0_in};
         * particle_filter.init_normal_around_point(start_point, 5_in);
         * @endcode
         */
        void init_normal_around_point(const Point& point, const Length std_deviation) {
		std::normal_distribution x_dist(point.x.internal(), std_deviation.internal());
		std::normal_distribution y_dist(point.y.internal(), std_deviation.internal());
            for(size_t i = 0;i < N;i++){
                particles[i].x = x_dist(rng) * m;
                particles[i].y = y_dist(rng) * m;
            }
            for(size_t i = 0;i < N;i++){
                // weights[i] = 1.0 / static_cast<float>(N);
                weights[i] = average_weight;
            }

            // update angle of prediction
            updateAngle();
            // ensure an update immediately
            this->distance_since_update = this->max_distance_since_update;
        }

        void initUniform(const Length min_x, const Length min_y, const Length max_x, const Length max_y) {
            std::uniform_real_distribution x_dist(min_x.internal(), max_x.internal());
            std::uniform_real_distribution y_dist(min_y.internal(), max_y.internal());

            for(size_t i = 0;i < N;i++){
                particles[i].x = x_dist(rng) * m;
                particles[i].y = y_dist(rng) * m;
            }
            for(size_t i = 0;i < N;i++){
                weights[i] = average_weight;
            }
            // update angle of prediction
            updateAngle();
            // ensure an update immediately
            distance_since_update = max_distance_since_update;
        }

    };

    // template<size_t number_of_particles>
    // static void init_localization_task(ParticleFilter<number_of_particles>& particle_filter){
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
}
