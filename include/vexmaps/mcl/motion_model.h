#pragma once

#include "pros/apix.h"
#include "pros/imu.h"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include "vexmaps/mcl/config.h"
#include "vexmaps/mcl/pose.h"
#include "vexmaps/mcl/utils.h"
#include "vexmath/fast_prng/Xoroshiro128plus_vectorized.hpp"
#include "vexmath/functions/trig_taylor.hpp"
#include "vexmath/functions/vectorized_trig.hpp"
#include "vexmath/functions/vectorized_trig_taylor.hpp"
#include <arm_neon.h>
#include <cmath>
#include <random>

using namespace vexmaps::localization_settings;

namespace vexmaps {
class MotionModel {
  private:
    // --- linear odometry variables / functions --- //
    const Length wheelDiameter = 3.25_in;
    const float gear_ratio = (48.0 / 36.0) * 300;
    const Length circumference = M_PI * wheelDiameter;

    Length getMotorGroupDistance(const pros::MotorGroup* motors) {
        Length totalPosition = 0.0_m;

        // rotations / second
        for (double& position : motors->get_position_all()) {
            totalPosition +=
              (position * rot) * (circumference / rot) * (1 / gear_ratio);
        }

        return totalPosition / motors->size();
    }

    const Length odom_wheel_diameter = 2_in;
    const float odom_gear_ratio = 1;
    const Length odom_circumference = M_PI * odom_wheel_diameter;

    Length getRotationDistance(const pros::Rotation* rotation) {
        const Length odom_circumference = 1.995_in * M_PI;
        float rotations =
          from_stDeg(static_cast<float>(rotation->get_position()) / 100.0) /
          rot;
        Length totalPosition = rotations * odom_circumference;
        return totalPosition;
    }

    // can only have one on or it explodes
    pros::Rotation* horizontal_rot;
    pros::Rotation* vertical_rot;

    // --- arc based odometry variables / functions ---/
    // const Length SH = 0_m, SV = 0_m;
    // Length dH = 0_m, dV = 0_m;
    // Angle dTheta = 0_stDeg,avgHeading = 0_stDeg;
    //
    // Point local_delta = {0_m,0_m}, global_delta = {0_m,0_m};
    //
    // Point last_distances = {0_m,0_m};

    // arc-based odometry - sets global_delta to the calculated x and y offset
    // void arcOdomUpdate(Angle avgAngle, Angle dTheta){
    //     if(chosen_delta_model != arc_odometry){
    //         // somehow this function got called without being enabled,
    //         probably not intended printf("error: arc update called without
    //         arc odometry being enabled\n");
    //         // return;
    //     }
    //
    //     bool horizontal_available = true, vertical_available = true;
    //     if(horizontal_rot == nullptr || !horizontal_rot->is_installed()){
    //         // the horizontal rotational sensor cannot be used, we fall back
    //         to printf("error: horizontal rotational sensor not available\n");
    //         horizontal_available = false;
    //     }
    //
    //     if(vertical_rot == nullptr || !vertical_rot->is_installed()){
    //         // the horizontal rotational sensor cannot be used, we fall back
    //         to printf("error: horizontal rotational sensor not available\n");
    //         vertical_available = false;
    //     }
    //
    //     // update delta's
    //     Point distances;
    //     if(horizontal_available){
    //         distances.x = getOdomDistance(horizontal_rot);
    //     }else{
    //         // no available information on horizontal change
    //         distances.x = 0_m;
    //     }
    //
    //     if(vertical_available){
    //         distances.y = getOdomDistance(vertical_rot);
    //     }else{
    //         distances.y = getDistance(drivetrain->rightMotors);
    //
    //     }
    //
    //     dH = distances.x - last_distances.x;
    //     dV = distances.y - last_distances.y;
    //
    //     Length local_offset_x = 0_m,local_offset_y = 0_m;
    //     if(dTheta == 0_stRad){
    //         local_offset_x = dH;
    //         local_offset_y = dV;
    //     } else{
    //         Number factor = 2 * units::sin(dTheta/2);
    //        local_delta.x = factor * (dH/dTheta.internal()) + SH;
    //        local_delta.y = factor * (dV/dTheta.internal()) + SV;
    //     }
    //     const float sina = units::sin(avgAngle).internal();
    //     const float cosa = units::cos(avgAngle).internal();
    //
    //     // calculate global x and y
    //     global_delta.x += local_delta.y *  sina;
    //     global_delta.y += local_delta.y *  cosa;
    //     global_delta.x += local_delta.x * -cosa;
    //     global_delta.y += local_delta.x *  sina;
    //
    //
    //     last_distances = distances;
    // }

    // --- imu odometry variables/functions --- //
    // Time last_imu_update_time = infinity() * sec;
    // pros::Imu * imu;
    //
    // LinearAcceleration x_accel,y_accel,z_accel;
    // LinearVelocity x_vel,y_vel,z_vel;
    // Length x_imu,y_imu,z_imu;
    //
    // void getHorizontalChangeFromImu(){
    //     const Time current_time = pros::millis() * msec;
    //     Time dt = 0_sec;
    //     if(isfinite(last_imu_update_time.internal())){
    //         Time dt = current_time - last_imu_update_time;
    //     }
    //     last_imu_update_time = current_time;
    //
    //     pros::imu_accel_s_t accels = imu->get_accel();
    //     x_accel = accels.x * 9.81_mps2;
    //     y_accel = accels.y * 9.81_mps2;
    //     z_accel = accels.z * 9.81_mps2;
    //
    //     x_vel += x_accel * dt;
    //     y_vel += y_accel * dt;
    //     z_vel += z_accel * dt;
    //
    //     x_imu += x_vel * dt;
    //     y_imu += y_vel * dt;
    //     z_imu += z_vel * dt;
    // }

    std::uniform_real_distribution<float> average_distance_distribution;
    std::uniform_real_distribution<float> angle_distribution;
    std::uniform_real_distribution<float> drift_distribution;

    Vuniform_float32_t Vaverage_distance_distribution;
    Vuniform_float32_t Vangle_distribution;
    Vuniform_float32_t Vdrift_distribution;

    // --- linear odometry variables/functions --- //
    Length distance_traveled = 0_m;

    Length last_left_travel = 0_m, last_right_travel = 0_m;

    // changes between last and current travel values
    Length left_distance_delta = 0_m, right_distance_delta = 0_m;
    Length average_distance_delta = 0_m;
    //
    // changes between last and current travel values
    Length last_hor_distance = 0_m, last_ver_distance = 0_m;
    Length hor_distance_delta = 0_m, ver_distance_delta = 0_m;

    Time last_update_time;
    Time current_time;
    Time delta_time;

    // LinearVelocity last_left_velocity = 0_mps, last_right_velocity = 0_mps,
    // average_velocity = 0_mps;
    //
    // LinearAcceleration average_acceleration;
    //
    // LinearVelocity getVelocity(const pros::MotorGroup * motors) {
    //     LinearVelocity totalVelocity = 0.0_mps;
    //
    //     // rotations / second
    //     for (double & velocity : motors->get_actual_velocity_all()) {
    //         // need to convert from angular velocity of the motor to linear
    //         velocity totalVelocity += (from_rpm(velocity)) * (circumference /
    //         rot) * (1/gear_ratio);
    //         // gearset / 2
    //     }
    //
    //     return totalVelocity / motors->size();
    // }
    //
    Point distanceDelta;

    void linearOdometryUpdate() {
        bool horizontal_available = true, vertical_available = true;
        if (horizontal_rot == nullptr || !horizontal_rot->is_installed()) {
            // the horizontal rotational sensor cannot be used, we fall back to
            // printf("error: horizontal rotational sensor not available\n");
            horizontal_available = false;
        }

        if (vertical_rot == nullptr || !vertical_rot->is_installed()) {
            // the horizontal rotational sensor cannot be used, we fall back to
            // printf("error: horizontal rotational sensor not available\n");
            vertical_available = false;
        }

        // if(horizontal_available){
        //     const Length curr_hor_distance =
        //     getRotationDistance(horizontal_rot); hor_distance_delta =
        //     curr_hor_distance - last_hor_distance; last_hor_distance =
        //     curr_hor_distance; distanceDelta.x = hor_distance_delta;
        // }else{
        // no available information on horizontal change
        distanceDelta.x = 0_m;
        // }

        // calculate drivetrain distances even if we dont use it in case we need
        // it eventually
        const Length left_travel =
          getMotorGroupDistance(drivetrain->leftMotors);
        const Length right_travel =
          getMotorGroupDistance(drivetrain->rightMotors);
        left_distance_delta = left_travel - last_left_travel;
        right_distance_delta = right_travel - last_right_travel;
        average_distance_delta =
          (left_distance_delta + right_distance_delta) / 2.0;

        last_left_travel = left_travel;
        last_right_travel = right_travel;

        // if(vertical_available){
        //     const Length curr_ver_distance =
        //     getRotationDistance(vertical_rot); ver_distance_delta =
        //     curr_ver_distance - last_ver_distance; last_ver_distance =
        //     curr_ver_distance; distanceDelta.y = ver_distance_delta;
        // }else{
        distanceDelta.y = average_distance_delta;
        // }

        // const Length left_travel =
        // getMotorGroupDistance(drivetrain->leftMotors); const Length
        // right_travel = getMotorGroupDistance(drivetrain->rightMotors); const
        // LinearVelocity left_velocity = getVelocity(drivetrain->leftMotors);
        // const LinearVelocity right_velocity =
        // getVelocity(drivetrain->rightMotors);

        // const LinearAcceleration left_acceleration =  (left_velocity  -
        // last_left_velocity) /delta_time; const LinearAcceleration
        // right_acceleration = (right_velocity -
        // last_right_velocity)/delta_time;

        // left_distance_delta = left_travel - last_left_travel;
        // right_distance_delta = right_travel - last_right_travel;

        // average_distance_delta = (left_distance_delta + right_distance_delta)
        // / 2.0; average_velocity       = (left_velocity + right_velocity)
        // / 2.0; average_acceleration   = (left_acceleration +
        // right_acceleration) / 2.0;

        // update last variables
        // last_left_velocity = left_velocity;
        // last_right_velocity = right_velocity;
        // last_left_travel = left_travel;
        // last_right_travel = right_travel;
    }

    Angle last_angle = INFINITY * deg, avg_angle = 0_stDeg,
          angle_change = 0_stDeg, abs_angle_change = 0_stDeg;

    Point change_point = { 0_m, 0_m };

    Length distance_noise = 0_m, angle_drift_noise = 0_m;
    Angle angle_noise = 0_stDeg;

    float32x4_t Vavg_sina, Vavg_cosa;
    float32x4_t Vsina, Vcosa;

    float avg_sina, avg_cosa;
    float sina, cosa;

    lemlib::Drivetrain* drivetrain;

  public:
    MotionModel(lemlib::Drivetrain* drivetrain,
                pros::Rotation* vertical_rot,
                pros::Rotation* horizontal_rot)
        : drivetrain(drivetrain),
          vertical_rot(vertical_rot),
          horizontal_rot(horizontal_rot) {}

    // calculates change in x and y since last update
    void motionUpdate(const Angle angle) {
        current_time = from_msec(pros::millis());
        delta_time = current_time - last_update_time;
        last_update_time = current_time;

        // mutable version of the current angle
        Angle current_angle = angle;

        // here we must use the current angle for both angles
        // this should be fine unless we are rotating by a lot right as we start
        // the particle filter
        if (!isfinite(last_angle.internal())) {
            avg_angle = current_angle;
            angle_change = 0.1_stDeg;
        } else {
            // avg_angle = (current_angle + last_angle)/2;
            // angle_change = current_angle - last_angle;
            //
            // here we assume angle and last_angle are both postive and below
            // 360

            // makes sure differences in angles are kept counter-clockwise
            //
            // here it is guarateed that angle > last_angle
            const Angle bigger_angle = units::max(current_angle, last_angle);
            const Angle smaller_angle = units::min(current_angle, last_angle);

            Angle difference = bigger_angle - smaller_angle;
            Angle average_angle =
              units::constrainAngle360_2(smaller_angle + difference);

            //
            // we always assume we took the shortest path to the current angle
            // if the difference is greater than 180 then going the other way is
            // faster
            if (difference > rot / 2) {
                difference = rot - difference;
                Angle average_angle =
                  units::constrainAngle360_2(smaller_angle - difference);
            }

            // this may become more than 360, however we only operate with trig
            // calls on this meaning having it be between 0 and 360 is not
            // strictly necesarly
            avg_angle = average_angle;

            angle_change = difference;

            // if(half_angle_change < 0_stDeg) half_angle_change += rot;
        }
        // abs_angle_change = units::abs(angle_change);

        // uses the original given angle instead of the modified one
        last_angle = angle;

        linearOdometryUpdate();

        // const Length slip_noise = units::abs(slip_distance_ratio *
        // average_distance); const Length velocity_noise = slip_velocity_factor
        // * units::abs(average_velocity); const Length acceleration_slip_noise
        // = slip_acceleration_factor * units::abs(average_acceleration);

        distance_noise = DRIVE_NOISE2 + angle_vertical_noise_relation_factor *
                                          abs_angle_change;
        angle_noise = abs_angle_change * ANGLE_NOISE;
        angle_drift_noise =
          DRIFT_NOISE + angle_drift_relation_factor * abs_angle_change;

        // distance_noise = 0_m;
        // angle_noise = 0_stDeg;
        // angle_drift_noise = 0_m;

        average_distance_distribution = std::uniform_real_distribution<float>(
          (distanceDelta.y - distance_noise).internal(),
          (distanceDelta.y + distance_noise).internal());
        // angle_distribution = std::uniform_real_distribution<float>(
        //     (avg_angle - angle_noise).internal(),
        //     (avg_angle + angle_noise).internal()
        // );
        angle_distribution =
          std::uniform_real_distribution<float>((-angle_noise).internal(),
                                                (+angle_noise).internal());
        drift_distribution = std::uniform_real_distribution<float>(
          (distanceDelta.x - angle_drift_noise).internal(),
          (distanceDelta.x + angle_drift_noise).internal());

        avg_sina = units::sin(avg_angle);
        avg_cosa = units::cos(avg_angle);

        if (usingVectorizedMotion) {
            Vaverage_distance_distribution =
              Vuniform_float32_t((distanceDelta.y - distance_noise).internal(),
                                 (distanceDelta.y + distance_noise).internal(),
                                 robot_rng());
            // Vangle_distribution = Vuniform_float32_t(
            //         (avg_angle - angle_noise).internal(),
            //         (avg_angle + angle_noise).internal(),
            //         robot_rng());
            Vangle_distribution = Vuniform_float32_t((-angle_noise).internal(),
                                                     (+angle_noise).internal(),
                                                     robot_rng());
            Vdrift_distribution = Vuniform_float32_t(
              (distanceDelta.x - angle_drift_noise).internal(),
              (distanceDelta.x + angle_drift_noise).internal(),
              robot_rng());

            Vavg_sina = vld1q_dup_f32(&avg_sina);
            Vavg_cosa = vld1q_dup_f32(&avg_cosa);
        }

        // update "actual" odometry information
        change_point = {
            distanceDelta.y * avg_cosa - distanceDelta.x * avg_sina,
            distanceDelta.y * avg_sina + distanceDelta.x * avg_cosa
        };

        distance_traveled += average_distance_delta;
    }

    Point actualOdomChange() {
        return change_point;
    }

    Length actualDistanceTraveled() {
        return distance_traveled;
    }

    Angle getAngleChange() {
        return angle_change;
    }

    /**
     * @brief makes a vector of motion updates (in base units)
     *
     * @param result vector where the motion updates get stored
     */
    void fastMotionUpdates(float32x4x2_t* result) {
        const float32x4_t distance = Vaverage_distance_distribution();
        const float32x4_t drift = Vdrift_distribution();
        const float32x4_t angle = Vangle_distribution();

        // sincos_ps(angle, &sina, &cosa);
        Vsincos_taylor_delta(angle, Vavg_sina, Vavg_cosa, &Vsina, &Vcosa);

        //  x = distance * cosa - noisy_drift * sina,
        result->val[0] = distance * Vcosa - drift * Vsina;
        // y = distance * sina + drift * cosa
        result->val[1] = distance * Vsina + drift * Vcosa;
    }

    // odometry change with some noise added for each particle
    Point singleParticleMotion() {
        const Length noisy_distance = average_distance_distribution(rng) * m;
        const Length noisy_drift = drift_distribution(rng) * m;
        const float noise_angle = angle_distribution(rng); // * rad;

        sincos_taylor_delta(noise_angle, avg_sina, avg_cosa, &sina, &cosa);

        // rotates forward and sideways movement to face the robot's heading
        return { noisy_distance * cosa - noisy_drift * sina,
                 noisy_distance * sina + noisy_drift * cosa };
    }
};

} // namespace vexmaps
