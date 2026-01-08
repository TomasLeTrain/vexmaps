#pragma once

#include "units/Angle.hpp"
#include "units/units.hpp"

namespace vexmaps {

// default configs
struct PFConfiguration {
    // using vectorized routines. set to off if using untested routines or stuff
    // breaks
    bool usingVectorizedMotion = true;

    bool logging = false;
    bool particle_logging = false;
    bool custom_particle_logging = false;
    bool print_custom_data = false;

    // threshold for sum of weights before normalization which determines if the
    // iteration is lost this should be tuned so iterations which are clearly
    // lost can be determined and counted so the algorithm can recover
    float lost_max_weight_threshold = 0.001;

    int max_lost_iteration_count = 15;

    // weight which is considered to not contribute - its weight is very low
    // used to see number of non contributing particles for resampling
    // float near_zero_epsilon = 0.8;

    // percentage of particles which have weights near zero to resample
    float near_zero_particle_percentage = 0.75;

    // percentage of the max weight at which a particle is included for the
    // prediction the lower the number the less accurate prediction will be but
    // it might be smoother
    float weightPredictionFactor = 0.8;

    // should be at most half the width of the robot
    FLength wall_border_width = 6_in;
};

struct MotionModelConfig {
    FLength forwards_noise = 0.4_in;

    // relates slip to change in distance
    // higher distance travel usually results in wheel slipage, therefore we use
    // a ratio to add noise based on change in distance x inches of noise added
    // / x inches traveled
    Divided<FLength, FLinearVelocity> slip_velocity_factor = 2_in / 30_inps;

    // relates slip to change in distance
    // higher distance travel usually results in wheel slipage, therefore we use
    // a ratio to add noise based on change in distance x inches of noise added
    // / x inches traveled
    Divided<FLength, FLinearAcceleration> slip_acceleration_factor =
      2_in / 100_inps2;

    // factor of how much the avg angle change should be applied as noise
    // setting to 1.0 means some particles would move in the direction of the
    // last angle while some would move only in the direction of the new angle
    float angle_noise = 1.0;

    // applies drift to particles
    FLength drift_noise = 0.4_in;

    // relation factor between the change in angle and drift
    // big changes in angle plus movement is usually what results in drift
    // therefore we can increase drift when we have big angles changes to better
    // model the robot.
    Divided<FLength, FAngle> angle_to_drift_noise = 0.1_in / 30_stDeg;

    // same as angle drift relation factor
    // instead this one relates the angle change in movement vertically
    // this also accounts for the drastical drift that happens on angle changes
    Divided<FLength, FAngle> angle_to_forwards_noise = 0.01_in / 30_stDeg;

    // relates number of lost iterations to additional noise in the system
    // set to zero to disable lost iterations from applying at all
    FLength lost_iter_to_forwards_noise = 0.2_in;
    FLength lost_iter_to_drift_noise = 0.2_in;
    FAngle lost_iter_to_angle_noise = 1_stDeg;

    // amount of time expected between the process noise being applied
    // this is done to make sure the noise stays consistent regardless of the
    // intervals in which it is being applied
    // lost_iter... variables are not affected by this
    FTime process_time = 10_msec;
};

struct DistanceSensorConfig {
    // all floats without units are in meters
    float exp_l = 1.5;
    float std_deviation = (2_in).internal();
    float map_deviation = (3_in).internal();

    // sum of coefficients 1
    float randomCoeff = 0.0;
    float expCoeff = 0.15;
    float normalCoeff = 0.6;
    float mapCoeff = 0.25;

    FLength maxDistanceDifference = 5_in;
    FLength maxOutDistanceDifference = 5_in;

    // threshold for max distance that is still used in the mcl
    // at long distances the distance sensors tend to be significantly
    // inaccurate (might be tunable to be better?)
    FLength maxUsableDistance = 70_in;

    bool detect_obstacles = true;

    bool logging = true;
};

} // namespace vexmaps
