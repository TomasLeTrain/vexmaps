#pragma once

#include "units/Angle.hpp"
#include "units/units.hpp"

namespace vexmaps {

// Distance Config concept
template<typename Config>
concept ValidDistanceConfig = requires {
    // clang-format off
    { Config::exp_l }         -> std::convertible_to<double>;
    { Config::std_deviation } -> std::convertible_to<double>;
    { Config::randomCoeff }   -> std::convertible_to<double>;
    { Config::expCoeff }      -> std::convertible_to<double>;
    { Config::normalCoeff }   -> std::convertible_to<double>;
    { Config::logging } -> std::convertible_to<bool>;
    // clang-format on
};

// default configs
struct PFConfiguration {
    // using vectorized routines. set to off if using untested routines or stuff
    // breaks
    bool usingVectorizedMotion = true;

    bool logging = false;
    bool particle_logging = false;

    // threshold for sum of weights before normalization which determines if the
    // iteration is lost this should be tuned so iterations which are clearly
    // lost can be determined and counted so the algorithm can recover
    float lost_max_weight_threshold = 0;

    int max_lost_iteration_count = 15;

    // weight which is considered to not contribute - its weight is very low
    // used to see number of non contributing particles for resampling
    // float near_zero_epsilon = 0.8;

    // percentage of particles which have weights near zero to resample
    float near_zero_particle_percentage = 0.9;

    // percentage of the max weight at which a particle is included for the prediction
    // the lower the number the less accurate prediction will be but it might be smoother
    float weightPredictionFactor = 0.8;

    // should be at most half the width of the robot
    Length wall_border_width = 6_in;
};

struct MotionModelConfig {
    Length forwards_noise = 0.2_in;

    // relates slip to change in distance
    // higher distance travel usually results in wheel slipage, therefore we use
    // a ratio to add noise based on change in distance x inches of noise added
    // / x inches traveled
    Divided<Length, LinearVelocity> slip_velocity_factor = 2_in / 30_inps;

    // relates slip to change in distance
    // higher distance travel usually results in wheel slipage, therefore we use
    // a ratio to add noise based on change in distance x inches of noise added
    // / x inches traveled
    Divided<Length, LinearAcceleration> slip_acceleration_factor =
      2_in / 100_inps2;

    // factor of how much the avg angle change should be applied as noise
    // setting to 1.0 means some particles would move in the direction of the
    // last angle while some would move only in the direction of the new angle
    float angle_noise = 0.15;

    // applies drift to particles
    Length drift_noise = 0.2_in;

    // relation factor between the change in angle and drift
    // big changes in angle plus movement is usually what results in drift
    // therefore we can increase drift when we have big angles changes to better
    // model the robot.
    Divided<Length, Angle> angle_to_drift_noise = 0.1_in / 30_stDeg;

    // same as angle drift relation factor
    // instead this one relates the angle change in movement vertically
    // this also accounts for the drastical drift that happens on angle changes
    Divided<Length, Angle> angle_to_forwards_noise = 0.01_in / 30_stDeg;

    // relates number of lost iterations to additional noise in the system
    // set to zero to disable lost iterations from applying at all
    Length lost_iter_to_forwards_noise = 0.2_in;
    Length lost_iter_to_drift_noise = 0.2_in;
    Angle lost_iter_to_angle_noise = 1_stDeg;

    // amount of time expected between the process noise being applied
    // this is done to make sure the noise stays consistent regardless of the
    // intervals in which it is being applied
    // lost_iter... variables are not affected by this
    Time process_time = 10_msec;

    // percentage by which noise increases for every process_time time period
    // done since doubling the noise values may result in way too much noise
    // uniform(2 * a, 2 * b) != 2 * uniform(a, b) + uniform(a, b)
    float process_time_noise_factor = 0.5;
};

struct DistanceSensorConfiguration {
    // all floats without units are in meters
    static constexpr double exp_l = 1.5;
    static constexpr double std_deviation = (2_in).internal();

    // all these should add to one
    static constexpr double randomCoeff = 0.175;
    static constexpr double expCoeff = 0.3;
    static constexpr double normalCoeff = 0.525;

    static constexpr bool logging = false;
};

} // namespace vexmaps
