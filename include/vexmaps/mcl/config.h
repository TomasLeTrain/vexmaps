#pragma once

#include "vexmaps/mcl/utils.h"
#include "units/units.hpp"
#include "units/Angle.hpp"
#include "pros/apix.h"

namespace vexmaps {
    namespace localization_settings {
        // may break everything
        static constexpr bool usingVectorizedMotion = true;

        static constexpr bool logging = true;
        static constexpr bool particle_logging = false;

        // threshold for sum of weights before normalization which determines if the iteration is lost
        // this should be tuned so iterations which are clearly lost can be determined and counted so the algorithm can recover
        static constexpr float low_weight_sum_threshold = 0;

        // weight which is considered to not contribute - its weight is very low
        // used to see number of non contributing particles for resampling
        static constexpr float near_zero_epsilon = 0.8;
        // percentage of particles which have weights near zero to resample
        static constexpr float near_zero_particle_percentage = 0.5;

        // % of noise in drivetrain
        // we want the lower bound for particles to be 0, as in the worst case the wheels slip / the bot tilts
        // causing all the wheels moving a lot more than they actually do
        // however we also want the upper bound to be high,
        // as a high angle noise causes the likely straight movements of the robot to drift into sides  
        static constexpr float DRIVE_NOISE = 1.0;

        // different method of introducing noise to the odometry model
        // this constant is suppposed to introduce constant noise, not dependent on how much the robot actually moves
        
        // good constant
        static constexpr Length DRIVE_NOISE2 = 0.02_in;
        // static constexpr Length DRIVE_NOISE2 = 0.018_in;

        // relates slip to change in distance
        // higher distance travel usually results in wheel slipage, therefore we use a ratio to add noise based on change in distance
        // x inches of noise added / x inches traveled
        // static constexpr float slip_distance_ratio = (2_in/10_in).internal();
        
        // relates slip to change in distance
        // higher distance travel usually results in wheel slipage, therefore we use a ratio to add noise based on change in distance
        // x inches of noise added / x inches traveled
        static constexpr auto slip_velocity_factor = 2_in/30_inps;

        // relates slip to change in distance
        // higher distance travel usually results in wheel slipage, therefore we use a ratio to add noise based on change in distance
        // x inches of noise added / x inches traveled
        static constexpr auto slip_acceleration_factor = 2_in/100_inps2;
            
        // 5956G was here, i love sonny and tom
        
        // factor of how much the avg angle change should be applied as noise
        // setting to 1.0 means some particles would move in the direction of the last angle
        // while some would move only in the direction of the new angle
        // static constexpr float ANGLE_NOISE = 0.75;
        
        // good constants
        static constexpr float ANGLE_NOISE = 0.15;
        // static constexpr float ANGLE_NOISE = 0.14;

        // drift noise
        // applies drift to particles 
        // +/- drift applied to particles
        
        // good constant
        static constexpr Length DRIFT_NOISE = 0.1_in;
        // static constexpr Length DRIFT_NOISE = 0.07_in;

        // relation factor between the change in angle and drift
        // big changes in angle plus movement is usually what results in drift
        // therefore we can increase drift when we have big angles changes to better model the robot
        // this factor is multiplied by the angle change, then this gets added to the drift noise

        // good constant
        static constexpr auto angle_drift_relation_factor = 0.1_in / 30_stDeg;
        // static constexpr auto angle_drift_relation_factor = 0.08_in / 30_stDeg;

        // same as angle drift relation factor
        // instead this one relates the angle change in movement vertically
        // this also accounts for the drastical drift that happens on angle changes
        
        // good constant
        static constexpr auto angle_vertical_noise_relation_factor = 0.01_in / 30_stDeg;
        // static constexpr auto angle_vertical_noise_relation_factor = 0.008_in / 30_stDeg;

        // if the difference between an expected value and this sensor is higher than this value, then we disregard the sensor
        static constexpr Length sensor_disparity_threshold = 12_in;
        
        // localization_settings(bool logging,float near_zero_particle_percentage)
        // :logging(logging), near_zero_particle_percentage(near_zero_particle_percentage)
        // {}
    }
}
