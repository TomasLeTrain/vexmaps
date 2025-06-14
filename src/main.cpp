#include "main.h"
#include "pros/abstract_motor.hpp"
#include "units/units.hpp"
#include "vexmaps/localization_model.hpp"
#include "vexmaps/mcl/config.hpp"
#include "vexmaps/mcl/distance_model.hpp"
#include "vexmaps/mcl/pf_motion_model.hpp"
#include "vexmaps/odometry/odometry.hpp"
#include "vexmaps/odometry/tracking_wheel.hpp"
#include "vexmaps/particle_filter_model.hpp"
#include <memory>

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    // hardware
    pros::Motor left_motor_1(1,
                             pros::MotorCartridge::blue,
                             pros::MotorUnits::rotations);
    pros::Motor right_motor_1(2,
                              pros::MotorCartridge::blue,
                              pros::MotorUnits::rotations);

    pros::Rotation horizontal_rotation(3);
    pros::Rotation vertical_rotation(4);

    pros::Imu imu(5);

    pros::Distance laser1(6);
    pros::Distance laser2(7);
    pros::Distance laser3(8);
    pros::Distance laser4(9);

    pros::MotorGroup left_motors(left_motor_1);
    pros::MotorGroup right_motors(right_motor_1);

    double dt_gear_ratio = (48.0 / 36.0);
    Length dt_diameter = 3.25_in;
    Length track_width = 13_in; // inches

    Length odom_wheel_diameter = 2.0_in; // inches

    vexmaps::MotorGroupTracking left_dt_tracker(&left_motors,
                                                dt_diameter,
                                                dt_gear_ratio,
                                                -(track_width) / 2);
    vexmaps::MotorGroupTracking right_dt_tracker(&right_motors,
                                                 dt_diameter,
                                                 dt_gear_ratio,
                                                 (track_width) / 2);

    vexmaps::HorizontalOdometryTracker horizontal1(&horizontal_rotation,
                                                   odom_wheel_diameter,
                                                   1,
                                                   0_m);
    vexmaps::VerticalOdometryTracker vertical1(&vertical_rotation,
                                               odom_wheel_diameter,
                                               1,
                                               0_m);

    std::unique_ptr<vexmaps::LocalizationModel> odometry_model =
      std::make_unique<vexmaps::OdometryModel>(&left_dt_tracker,
                                               &right_dt_tracker,
                                               std::vector { &horizontal1 },
                                               std::vector { &vertical1 },
                                               &imu);


    vexmaps::MotionModelConfig motion_model_config;

    // make the PF motion model wrapper
    vexmaps::PfMotionModel pf_motion_model(odometry_model, motion_model_config);

    // sensors
    vexmaps::DistanceSensorModel<vexmaps::DistanceSensorConfiguration>
      laser_model1(&laser1, { 0_m, 0_m, 0_stDeg }, "front");
    vexmaps::DistanceSensorModel<vexmaps::DistanceSensorConfiguration>
      laser_model3(&laser3, { 0_m, 0_m, 90_stDeg }, "left");
    vexmaps::DistanceSensorModel<vexmaps::DistanceSensorConfiguration>
      laser_model2(&laser2, { 0_m, 0_m, 180_stDeg }, "back");
    vexmaps::DistanceSensorModel<vexmaps::DistanceSensorConfiguration>
      laser_model4(&laser4, { 0_m, 0_m, 270_stDeg }, "right");

    vexmaps::ParticleFilterModel<1024> pf_model(
      &pf_motion_model,
      { &laser_model1, &laser_model2, &laser_model3, &laser_model4 },
      vexmaps::PFConfiguration());

    pf_model.init();
    
    // create the odom task
    pros::Task odom_task = createLocalizationTask(&pf_motion_model);

    // create a task to automatically update the particle filter
    pros::Task pf_task = createLocalizationTask(&pf_model);
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
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    pros::MotorGroup left_mg(
      { 1, -2, 3 }); // Creates a motor group with forwards ports 1 & 3 and
                     // reversed port 2
    pros::MotorGroup right_mg(
      { -4, 5, -6 }); // Creates a motor group with forwards port 5 and reversed
                      // ports 4 & 6

    while (true) {
        pros::lcd::print(0,
                         "%d %d %d",
                         (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
                         (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
                         (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >>
                           0); // Prints status of the emulated screen LCDs

        // Arcade control scheme
        int dir = master.get_analog(
          ANALOG_LEFT_Y); // Gets amount forward/backward from left joystick
        int turn = master.get_analog(
          ANALOG_RIGHT_X); // Gets the turn left/right from right joystick
        left_mg.move(dir - turn); // Sets left motor voltage
        right_mg.move(dir + turn); // Sets right motor voltage
        pros::delay(20); // Run for 20 ms then update
    }
}
