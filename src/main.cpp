#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/apix.h"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/units.hpp"
#include "vexmaps/api.hpp"
#include "vexmaps/mcl/map_reader.hpp"
#include "vexmath/fast_prng/Xoshiro128plus.hpp"
#include <initializer_list>
#include <random>
#include <vector>

constexpr size_t particle_count = 500;
// constexpr size_t particle_count = 16384;
// constexpr size_t particle_count = 30000;
constexpr bool general_logging = false;

vexmaps::MotionModelConfig motion_model_config = {};
vexmaps::PFConfiguration Pfconfig = {
    .logging = general_logging,
    .particle_logging = false,
};

vexmaps::DistanceSensorConfig distance_sensor_config { .detect_obstacles =
                                                         true };

class FakeDistance : public pros::Distance {
  private:
    Length fixed_length = 0_m;

  public:
    FakeDistance()
        : pros::Distance(10) {}

    void set_length(Length new_length) {
        fixed_length = new_length;
    };

    bool is_installed() override {
        return true;
    }

    std::int32_t get() override {
        return static_cast<std::int32_t>(fixed_length.convert(mm));
    }
};

// Inertial Sensor on port 8
vexmaps::ScaledIMU imu(13, 363.0 / 360.0);

using horizontalTrackers =
  std::initializer_list<vexmaps::HorizontalOdometryTracker*>;
using verticalTrackers =
  std::initializer_list<vexmaps::VerticalOdometryTracker*>;

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Motor
  Intake(15,
         pros::v5::MotorGears::blue,
         pros::v5::MotorUnits::rotations); // intake motor. port 16, reversed

pros::Distance left_sensor(6);
pros::Distance back_sensor(5);
pros::Distance right_sensor(16);
pros::Distance front_sensor(20);

// motor groups
pros::MotorGroup
  leftMotors({ -3, 10, -2 },
             pros::v5::MotorGears::blue,
             pros::v5::MotorUnits::rotations); // left motor group
pros::MotorGroup
  rightMotors({ 1, -9, 4 },
              pros::v5::MotorGears::blue,
              pros::v5::MotorUnits::rotations); // right motor group

// vertical tracking wheel in port 7, reversed direction
pros::Rotation verticalEnc(-7);
pros::Rotation horizontalEnc(-12);

AngularVelocity target_rpm = 480_rpm;

Length dt_diameter = 2.75_in;
Length track_width = 10.5_in;

Length odom_wheel_diameter = 1.995_in;

vexmaps::MotorGroupTracking
  left_dt_tracker(&leftMotors, dt_diameter, target_rpm, (-track_width) / 2);
vexmaps::MotorGroupTracking
  right_dt_tracker(&rightMotors, dt_diameter, target_rpm, (track_width) / 2);

vexmaps::HorizontalOdometryTracker
  horizontal1(&horizontalEnc, odom_wheel_diameter, 1, -0.7_in);
vexmaps::VerticalOdometryTracker
  vertical1(&verticalEnc, odom_wheel_diameter, 1, -0.525_in);

vexmaps::PfMotionModel<vexmaps::OdometryModel>
  odom_model(motion_model_config,
             &left_dt_tracker,
             &right_dt_tracker,
             horizontalTrackers { &horizontal1 },
             verticalTrackers { &vertical1 },
             // horizontalTrackers {},
             // verticalTrackers {},
             &imu,
             false);

MapReader<> map_reader;

FakeDistance fake_distance;
FakeDistance fake_front_distance;
FakeDistance fake_back_distance;
FakeDistance fake_left_distance;
FakeDistance fake_right_distance;

units::Pose front_distance_offsets = { 3.5_in,
                                       +(12.5_in / 2) - 1.25_in,
                                       0_stDeg };

units::Pose left_distance_offsets = { 3.5_in + 0.625_in,
                                      +(12.5_in / 2) - 1.25_in - 0.4_in,
                                      90_stDeg };

units::Pose back_distance_offsets = { -(15.5_in / 2) + 1.4_in,
                                      3.0_in,
                                      180_stDeg };

units::Pose right_distance_offsets = { -0.7_in,
                                       -(12.5_in / 2) + 2.25_in,
                                       270_stDeg };

double front_distance_scale_factor = 0.986105769705;
double left_distance_scale_factor = 0.985;
double back_distance_scale_factor = 0.97905795044;
double right_distance_scale_factor = 0.985454688793;

vexmaps::DistanceSensorModel front_laser_model(&fake_front_distance,
                                               front_distance_offsets,
                                               front_distance_scale_factor,
                                               "front",
                                               distance_sensor_config,
                                               &map_reader);
vexmaps::DistanceSensorModel left_laser_model(&fake_left_distance,
                                              left_distance_offsets,
                                              left_distance_scale_factor,
                                              "left",
                                              distance_sensor_config,
                                              &map_reader);
vexmaps::DistanceSensorModel back_laser_model(&fake_back_distance,
                                              back_distance_offsets,
                                              back_distance_scale_factor,
                                              "back",
                                              distance_sensor_config,
                                              &map_reader);
vexmaps::DistanceSensorModel right_laser_model(&fake_right_distance,
                                               right_distance_offsets,
                                               right_distance_scale_factor,
                                               "right",
                                               distance_sensor_config,
                                               &map_reader);

vexmaps::DistanceSensorModel fake_distance_model(&fake_distance,
                                                 { 0_in, 0_in, 0_stDeg },
                                                 1.0,
                                                 "fake",
                                                 distance_sensor_config,
                                                 &map_reader);

vexmaps::ParticleFilterModel<particle_count> pf_model(&odom_model,
                                                      { &front_laser_model,
                                                        &left_laser_model,
                                                        &back_laser_model,
                                                        &right_laser_model },
                                                      Pfconfig);
vexmaps::SmootherModel
  smoother_model(&odom_model, &pf_model, vexmaps::SmootherConfig());

vexmaps::ModelManager model_manager(
  {
    { &odom_model,     "odom model",     1 },
    { &pf_model,       "pf model",       2 },
    { &smoother_model, "smoother model", 3 },
},
  &smoother_model);

void initialize() {
    pros::c::serctl(SERCTL_DISABLE_COBS, NULL);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
    std::cout << "entered opcontrol" << std::endl;

    float sum_of_dists = 0.0;

    int n = 100;

    // std::uniform_real_distribution<float> xs(-70, 70);
    // std::uniform_real_distribution<float> ys(-70, 70);
    // std::uniform_real_distribution<float> thetas(0, 360);

    // Xoshiro128plus rng(10);

    // std::vector<FLength> v_x(n);
    // std::vector<FLength> v_y(n);
    //
    // std::vector<float> curr_weights(n);
    // std::vector<float> curr_weights2(n);
    // std::vector<float> tmp_list(n);

    // Length measured_distance = 18.4_in;
    units::Pose curr_pose = { 45.3_in, -42.2_in, 269.13_stDeg };

    // fake_distance.set_length(measured_distance);
    fake_front_distance.set_length(24.575_in);
    fake_left_distance.set_length(15.822_in);

    std::cout << "front" << std::endl;
    front_laser_model.update(curr_pose.orientation, curr_pose);
    std::cout << "left" << std::endl;
    left_laser_model.update(curr_pose.orientation, curr_pose);
    std::cout << "right" << std::endl;
    right_laser_model.update(curr_pose.orientation, curr_pose);
    std::cout << "back" << std::endl;
    back_laser_model.update(curr_pose.orientation, curr_pose);
    // fake_distance_model.update(curr_pose.orientation, curr_pose);

    // std::cout << "exit is "
    //           << (fake_distance_model.hasAvailableReading() ? "false" :
    //           "true")
    //           << std::endl;

    // v_x[0] = -20_in;
    // v_y[0] = 20_in;
    //
    // auto wall_start_time = pros::micros();
    //
    // fake_distance_model.evaluate_wall_array(
    //   curr_weights.data(),
    //   reinterpret_cast<float*>(v_x.data()),
    //   reinterpret_cast<float*>(v_y.data()),
    //   tmp_list.data(),
    //   n);
    //
    // auto wall_end_time = pros::micros();
    //
    // auto old_start_time = pros::micros();
    // for (int i = 0; i < n; i++) {
    //     curr_weights2[i] = fake_distance_model.evaluate(v_x[i], v_y[i]);
    // }
    // auto old_end_time = pros::micros();
    //
    // std::cout << "before all, curr_weights[0] = " << curr_weights[0]
    //           << std::endl;
    //
    // // check if curr weights is valid
    // for (int i = 0; i < n; i++) {
    //     if (auto diff = std::abs(curr_weights2[i] - curr_weights[i]);
    //         diff > 1e-3) {
    //         std::cout << "differ by: " << diff << std::endl;
    //     }
    // }
    // std::cout << "stopped checking!" << std::endl;
    //
    // std::cout << "new wall in  " << wall_end_time - wall_start_time
    //           << " microseconds." << std::endl;
    // std::cout << "old wall in  " << old_end_time - old_start_time
    //           << " microseconds." << std::endl;
    //
    // auto all_start_time = pros::micros();
    // fake_distance_model.evaluate_array(curr_weights.data(),
    //                                    v_x.data(),
    //                                    v_y.data(),
    //                                    tmp_list.data(),
    //                                    n);
    // auto all_end_time = pros::micros();
    //
    // // make it so the above is not optimized away
    // for (int i = 0; i < n; i++) {
    //     sum_of_dists += curr_weights[i];
    // }
    //
    // std::cout << "all wall in  " << all_end_time - all_start_time
    //           << " microseconds." << std::endl;
    //
    // std::cout << "result of computation was " << sum_of_dists << std::endl;
    //
    // auto query1 = map_reader.query(-20_Fin, 20_Fin, 60_FstDeg);
    // std::cout << "custom queyr " << (query1).convert(in) << std::endl;
    //
    // std::cout << "curr weight[0] = " << curr_weights[0] << std::endl;
}
