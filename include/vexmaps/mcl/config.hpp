#pragma once

#include "units/Angle.hpp"
#include "units/units.hpp"

namespace vexmaps {

// Particle Filter concept
template<typename Config>
concept ValidPFConfig = requires {
    // clang-format off
    { Config::usingVectorizedMotion } -> std::convertible_to<bool>;
    { Config::logging } -> std::convertible_to<bool>;
    { Config::particle_logging } -> std::convertible_to<bool>;
    { Config::low_weight_sum_threshold } -> std::convertible_to<float>;
    { Config::near_zero_epsilon } -> std::convertible_to<float>;
    { Config::near_zero_particle_percentage } -> std::convertible_to<float>;
    // clang-format on
};

// Motion Model Config Concept
template<typename Config>
concept ValidMotionModelConfig = requires {
    // clang-format off
    { Config::DRIVE_NOISE2 } -> std::convertible_to<Length>;
    { Config::slip_velocity_factor } -> std::convertible_to<Divided<Length, LinearVelocity>>;
    { Config::slip_acceleration_factor } -> std::convertible_to<Divided<Length, LinearAcceleration>>;
    { Config::ANGLE_NOISE } -> std::convertible_to<float>;
    { Config::DRIFT_NOISE } -> std::convertible_to<Length>;
    { Config::angle_drift_relation_factor } -> std::convertible_to<Divided<Length, Angle>>;
    { Config::angle_vertical_noise_relation_factor } -> std::convertible_to<Divided<Length, Angle>>;
    // clang-format on
};

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
struct DefaultPFSettings {
    // using vectorized routines. set to off if using untested routines or stuff
    // breaks
    static constexpr bool usingVectorizedMotion = true;

    static constexpr bool logging = true;
    static constexpr bool particle_logging = false;

    // threshold for sum of weights before normalization which determines if the
    // iteration is lost this should be tuned so iterations which are clearly
    // lost can be determined and counted so the algorithm can recover
    static constexpr float low_weight_sum_threshold = 0;

    // weight which is considered to not contribute - its weight is very low
    // used to see number of non contributing particles for resampling
    static constexpr float near_zero_epsilon = 0.8;

    // percentage of particles which have weights near zero to resample
    static constexpr float near_zero_particle_percentage = 0.5;
};

struct DefaultMotionModelSettings {
    static constexpr Length DRIVE_NOISE2 = 0.02_in;

    // relates slip to change in distance
    // higher distance travel usually results in wheel slipage, therefore we use
    // a ratio to add noise based on change in distance x inches of noise added
    // / x inches traveled
    static constexpr auto slip_velocity_factor = 2_in / 30_inps;

    // relates slip to change in distance
    // higher distance travel usually results in wheel slipage, therefore we use
    // a ratio to add noise based on change in distance x inches of noise added
    // / x inches traveled
    static constexpr auto slip_acceleration_factor = 2_in / 100_inps2;

    // factor of how much the avg angle change should be applied as noise
    // setting to 1.0 means some particles would move in the direction of the
    // last angle while some would move only in the direction of the new angle
    static constexpr float ANGLE_NOISE = 0.15;

    // applies drift to particles
    static constexpr Length DRIFT_NOISE = 0.1_in;

    // relation factor between the change in angle and drift
    // big changes in angle plus movement is usually what results in drift
    // therefore we can increase drift when we have big angles changes to better
    // model the robot.
    static constexpr auto angle_drift_relation_factor = 0.1_in / 30_stDeg;

    // same as angle drift relation factor
    // instead this one relates the angle change in movement vertically
    // this also accounts for the drastical drift that happens on angle changes
    static constexpr auto angle_vertical_noise_relation_factor =
      0.01_in / 30_stDeg;
};

struct DefaultDistanceSensorConfig {
    // all floats without units are in meters
    static constexpr double exp_l = 1.68;
    static constexpr double std_deviation = 0.03175; // 1.25 inches

    // the final distribution should integrates to 1
    static constexpr double randomCoeff = 0.189;
    static constexpr double expCoeff = 0.618;
    static constexpr double normalCoeff = 0.194;

    static constexpr bool logging = true;
};


} // namespace vexmaps
