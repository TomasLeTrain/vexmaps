#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "units/units.hpp"
#include "vexmaps/mcl/config.hpp"
#include "vexmaps/mcl/distance_model.hpp"
#include "vexmaps/mcl/pf_motion_model.hpp"
#include "vexmaps/odometry/odometry.hpp"
#include "vexmaps/odometry/tracking_wheel.hpp"
#include "vexmaps/particle_filter_model.hpp"
#include "pros/apix.h"
#include <initializer_list>

vexmaps::MotionModelConfig motion_model_config = { .forwards_noise = 0.15_in };
vexmaps::PFConfiguration Pfconfig = { .logging = true, .particle_logging = true };


// Inertial Sensor on port 8
pros::Imu imu(13);

using horizontalTrackers = std::initializer_list<vexmaps::HorizontalOdometryTracker*>;
using verticalTrackers = std::initializer_list<vexmaps::VerticalOdometryTracker*>;

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Motor frontLeft(-3,   pros::v5::MotorGears::blue,  pros::v5::MotorUnits::rotations); // left front motor. port 11, reversed
pros::Motor middleLeft(10,  pros::v5::MotorGears::blue,  pros::v5::MotorUnits::rotations); // left middle motor. port 12, reversed
pros::Motor backLeft(-2,    pros::v5::MotorGears::blue,  pros::v5::MotorUnits::rotations); // left back motor. port 13, reversed
                                                                                           //
pros::Motor frontRight(1,   pros::v5::MotorGears::blue,  pros::v5::MotorUnits::rotations); // right front motor. port 18
pros::Motor middleRight(-9, pros::v5::MotorGears::blue,  pros::v5::MotorUnits::rotations); // right middle motor. port 19
pros::Motor backRight(4,    pros::v5::MotorGears::blue,  pros::v5::MotorUnits::rotations); // right back motor. port 21
                                                                                           //
pros::Motor Intake(15,      pros::v5::MotorGears::blue,  pros::v5::MotorUnits::rotations); // intake motor. port 16, reversed

pros::Motor Lift(18,        pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations);
pros::Motor Lift2(-19,      pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations);

pros::Distance mogo_sensor(4);

pros::Optical optical_sensor(17);
pros::adi::DigitalIn sensor('C');
pros::adi::DigitalIn descoreLimit('A');

pros::Distance left_sensor(6);
pros::Distance back_sensor(5);
pros::Distance right_sensor(16);
pros::Distance front_sensor(20);

// motor groups
pros::MotorGroup leftMotors({frontLeft.get_port(), middleLeft.get_port(), backLeft.get_port()}); // left motor group
pros::MotorGroup rightMotors({frontRight.get_port(), middleRight.get_port(), backRight.get_port()}); // right motor group
pros::MotorGroup liftMotors({Lift.get_port(), Lift2.get_port()});

// vertical tracking wheel in port 7, reversed direction
pros::Rotation verticalEnc(-7);
pros::Rotation horizontalEnc(-12);

double dt_gear_ratio = (60.0 / 48.0);
Length dt_diameter = 2.75_in;
Length track_width = 10.5_in; // inches

Length odom_wheel_diameter = 1.995_in; // inches

vexmaps::MotorGroupTracking left_dt_tracker(&leftMotors,
                                            dt_diameter,
                                            dt_gear_ratio,
                                            -(track_width) / 2);
vexmaps::MotorGroupTracking right_dt_tracker(&rightMotors,
                                             dt_diameter,
                                             dt_gear_ratio,
                                             (track_width) / 2);

vexmaps::HorizontalOdometryTracker horizontal1(&horizontalEnc,
                                               odom_wheel_diameter,
                                               1,
                                               // 1.125_in);
                                               0.7_in);
vexmaps::VerticalOdometryTracker vertical1(&verticalEnc,
                                           odom_wheel_diameter,
                                           1,
                                           // 0.375_in);
                                           // 0.4_in);
                                           0.525_in);

vexmaps::PfMotionModel<vexmaps::OdometryModel> pf_motion_model(
        motion_model_config,
        &left_dt_tracker,
        &right_dt_tracker,
        horizontalTrackers{&horizontal1},
        verticalTrackers{&vertical1},
        &imu);

vexmaps::DistanceSensorModel<vexmaps::DistanceSensorConfiguration>
  front_laser_model(&front_sensor, { 5.25_in, 5.4375_in, 0_stDeg }, "front");
vexmaps::DistanceSensorModel<vexmaps::DistanceSensorConfiguration>
  left_laser_model(&left_sensor, { 3_in, 5.25_in, 90_stDeg }, "left");
vexmaps::DistanceSensorModel<vexmaps::DistanceSensorConfiguration>
  back_laser_model(&back_sensor, { -4_in, -1.84375_in, 180_stDeg }, "back");
vexmaps::DistanceSensorModel<vexmaps::DistanceSensorConfiguration>
  right_laser_model(&right_sensor, { 4.25_in, -5.375_in, 270_stDeg }, "right");

vexmaps::ParticleFilterModel<300> pf_model(
  &pf_motion_model,
  {
  &front_laser_model,
  &left_laser_model,
  &back_laser_model,
  &right_laser_model },
  Pfconfig);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::c::serctl(SERCTL_DISABLE_COBS,NULL);

    // reset the imu
    imu.reset(true);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    // initialize both models
    pf_motion_model.init();
    pf_model.init();

    // create the odom task
    pros::Task odom_task{[&] {
        while(true){
            uint32_t current_time = pros::millis();
            pf_motion_model.update();
            pros::c::task_delay_until(&current_time, to_msec(pf_motion_model.getTaskDeltaTime()));
        }
    }};

    pros::Task pf_task{[&] {
        while(true){
            uint32_t current_time = pros::millis();
            pf_model.update();
            pros::c::task_delay_until(&current_time, to_msec(pf_model.getTaskDeltaTime()));
        }
    }};
    
    // set the pose
    pf_model.setPose({48_in,-48_in,0_stDeg});

    while (true) {
        // printf(" start generation\n start distances\n front:18.7795,50,0.03175,false,25\n back:37.9528,63,0.03175,false,105\n left:37.9528,63,0.03175,false,105\n right:393.661,0,0.03175,true,-1\n end distances\n start particles\n end particles\n total weight: 1491.866211, time taken: 4928, timestamp: 0\n things done:1,1,0\n prediction:%f,%f,%f\n end generation\n",
        //         pf_model.getPose().x.convert(in),
        //         pf_model.getPose().y.convert(in),
        //         pf_model.getPose().orientation.convert(deg)
        //         );
        // printf("start generation\nstart distances\nend distances\nstart particles\nend particles\ntotal weight: 0, time taken: 0, timestamp: 0\nthings done:1,1,0\nprediction:%f,%f,%f\nend generation\n",
        //         pf_motion_model->getPose().x.convert(in),
        //         pf_motion_model->getPose().y.convert(in),
        //         pf_motion_model->getPose().orientation.convert(deg)
        //         );

        // printf("pose: %f, %f, %f\n",
        //         pf_motion_model.getPose().x.convert(in),
        //         pf_motion_model.getPose().y.convert(in),
        //         pf_motion_model.getPose().orientation.convert(deg)
        //         );
        // printf("pose: %f, %f, %f\n",
        //         pf_model.getPose().x.convert(in),
        //         pf_model.getPose().y.convert(in),
        //         pf_model.getPose().orientation.convert(deg)
        //         );
        //
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
