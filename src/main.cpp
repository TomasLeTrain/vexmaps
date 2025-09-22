#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/apix.h"
#include "pros/misc.h"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include "vexmaps/api.hpp"
#include <initializer_list>
#include "tests/circle_intersection_test.hpp"
#include "tests/distribution_tests.hpp"

constexpr size_t particle_count = 500;
// constexpr size_t particle_count = 16384;
// constexpr size_t particle_count = 30000;
constexpr bool general_logging = false;

vexmaps::MotionModelConfig motion_model_config = {};
vexmaps::PFConfiguration Pfconfig = {
    .logging = general_logging,
    .particle_logging = false,
};

struct CustomDistanceSensorConfiguration {
    // all floats without units are in meters
    static constexpr double exp_l = 1.5;
    static constexpr double std_deviation = (2_in).internal();

    // all these should add to one
    static constexpr double randomCoeff = 0.15;
    static constexpr double expCoeff = 0.1;
    static constexpr double normalCoeff = 0.75;

    static constexpr bool logging = general_logging;
    // static constexpr bool logging = false;
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

vexmaps::DistanceSensorModel<CustomDistanceSensorConfiguration>
  front_laser_model(&front_sensor, { 5.25_in, 5.4375_in, 0_stDeg }, "front");
vexmaps::DistanceSensorModel<CustomDistanceSensorConfiguration>
  left_laser_model(&left_sensor, { 3_in, 5.25_in, 90_stDeg }, "left");
vexmaps::DistanceSensorModel<CustomDistanceSensorConfiguration>
  back_laser_model(&back_sensor, { -4_in, -1.84375_in, 180_stDeg }, "back");
vexmaps::DistanceSensorModel<CustomDistanceSensorConfiguration>
  right_laser_model(&right_sensor, { 4.25_in, -5.375_in, 270_stDeg }, "right");

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
    pros::c::serctl(SERCTL_DISABLE_COBS,NULL);

    testCircleIntersections();
    testDistributions();

    // reset the imu
    imu.reset(true);

    // initialize all models and create their tasks
    model_manager.init();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
    // set the pose
    model_manager.setPose({ 48_in, -48_in, 0_stDeg });

    // printf("doing more stuff\n");
    bool manual_logging = true;

    while (true) {
        if (manual_logging) {
            int start_time = pros::millis();
            printf(
              "start generation\nstart distances\nend distances\nstart " "parti" "cles" "\n");

            printf("%.1f %.1f %.1f\n",
                   odom_model.getPose().x.convert(in),
                   odom_model.getPose().y.convert(in),
                   0.0);
            printf("%.1f %.1f %.1f\n",
                   pf_model.getPose().x.convert(in),
                   pf_model.getPose().y.convert(in),
                   5.0);
            printf("%.1f %.1f %.1f\n",
                   smoother_model.getPose().x.convert(in),
                   smoother_model.getPose().y.convert(in),
                   10.0);

            printf(
              "end particles\ntotal weight: 0, time taken: 30000, " "timestamp:" " %d\n",
              start_time);
            printf("things done:1,1,0,%d\n", particle_count);
            printf("prediction:%.1f,%.1f,%.1f\n",
                   smoother_model.getPose().x.convert(in),
                   smoother_model.getPose().y.convert(in),
                   smoother_model.getPose().orientation.convert(deg));
            printf("end generation\n");
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            smoother_model.setPose({ 48_in, -48_in, 90_stDeg });
        }

        // // Arcade control scheme
        int dir = master.get_analog(
          ANALOG_LEFT_Y); // Gets amount forward/backward from left joystick
        int turn = master.get_analog(
          ANALOG_RIGHT_X); // Gets the turn left/right from right joystick
        leftMotors.move(dir + turn); // Sets left motor voltage
        rightMotors.move(dir - turn); // Sets right motor voltage
        pros::delay(20); // Run for 20 ms then update
    }
}
