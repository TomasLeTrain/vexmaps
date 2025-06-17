#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/motors.h"
#include "units/units.hpp"
#include "vexmaps/localization_model.hpp"
#include "vexmaps/mcl/config.hpp"
#include "vexmaps/mcl/distance_model.hpp"
#include "vexmaps/mcl/pf_motion_model.hpp"
#include "vexmaps/odometry/odometry.hpp"
#include "vexmaps/odometry/tracking_wheel.hpp"
#include "vexmaps/particle_filter_model.hpp"
#include <memory>

// Inertial Sensor on port 8
pros::Imu imu(13);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    printf("bruh\n");
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
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    pros::Motor frontLeft(-3,   pros::v5::MotorGears::blue,  pros::v5::MotorUnits::rotations); // left front motor. port 11, reversed
    pros::Motor middleLeft(10,  pros::v5::MotorGears::blue,  pros::v5::MotorUnits::rotations); // left middle motor. port 12, reversed
    pros::Motor backLeft(-2,    pros::v5::MotorGears::blue,  pros::v5::MotorUnits::rotations); // left back motor. port 13, reversed
    pros::Motor frontRight(1,   pros::v5::MotorGears::blue,  pros::v5::MotorUnits::rotations); // right front motor. port 18
    pros::Motor middleRight(-9, pros::v5::MotorGears::blue,  pros::v5::MotorUnits::rotations); // right middle motor. port 19
    pros::Motor backRight(4,    pros::v5::MotorGears::blue,  pros::v5::MotorUnits::rotations); // right back motor. port 21
    pros::Motor Intake(15,      pros::v5::MotorGears::blue,  pros::v5::MotorUnits::rotations); // intake motor. port 16, reversed
    pros::Motor Lift(18,        pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations); // second intake motor. port 14, reversed
    pros::Motor Lift2(-19,      pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations); // second intake motor. port 14, reversed

    pros::Distance mogo_sensor(4);

    pros::Optical optical_sensor(17);
    pros::adi::DigitalIn sensor('C');
    pros::adi::DigitalIn descoreLimit('A');

    pros::Distance left_sensor(6);
    pros::Distance back_sensor(6);
    pros::Distance right_sensor(16);
    pros::Distance front_sensor(20);
    
    // motor groups
    pros::MotorGroup leftMotors({frontLeft.get_port(), middleLeft.get_port(), backLeft.get_port()}); // left motor group
    pros::MotorGroup rightMotors({frontRight.get_port(), middleRight.get_port(), backRight.get_port()}); // right motor group
    pros::MotorGroup liftMotors({Lift.get_port(), Lift2.get_port()});

    //pros::MotorGroup Intake({Intake, Intake2}); // Intake motor group
    
    // vertical tracking wheel in port 7, reversed direction
    pros::Rotation verticalEnc(-7);
    pros::Rotation horizontalEnc(-12);
    //horizontal tracking wheel. 2.75" diameter, 3.7" offset, back of the robot
    //lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::2, -1.5);
    // lemlib::TrackingWheel vertical(&verticalEnc, 2, -0.375);
    // lemlib::TrackingWheel horizontal(&horizontalEnc, 2, -1.125);
    // hardware

    double dt_gear_ratio = (48.0 / 48.0);
    Length dt_diameter = 2.75_in;
    Length track_width = 10_in; // inches

    Length odom_wheel_diameter = 2.0_in; // inches
                                         //
    printf("good on hardware\n");

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
                                                   -1.125_in);
    vexmaps::VerticalOdometryTracker vertical1(&verticalEnc,
                                               odom_wheel_diameter,
                                               1,
                                               // -0.375_in);
                                               -0.4_in);
    printf("good on trackers\n");

    std::unique_ptr<vexmaps::LocalizationModel> odometry_model =
      std::make_unique<vexmaps::OdometryModel>(&left_dt_tracker,
                                               &right_dt_tracker,
                                               std::vector { &horizontal1 },
                                               std::vector { &vertical1 },
                                               &imu);


    vexmaps::MotionModelConfig motion_model_config;
    printf("good on odom\n");

    // make the PF motion model wrapper
    vexmaps::PfMotionModel pf_motion_model(odometry_model, motion_model_config);

    printf("good on pf model\n");
    // sensors
    vexmaps::DistanceSensorModel<vexmaps::DistanceSensorConfiguration>
      laser_model1(&front_sensor, { 0_m, 0_m, 0_stDeg }, "front");
    vexmaps::DistanceSensorModel<vexmaps::DistanceSensorConfiguration>
      laser_model3(&left_sensor, { 0_m, 0_m, 90_stDeg }, "left");
    vexmaps::DistanceSensorModel<vexmaps::DistanceSensorConfiguration>
      laser_model2(&back_sensor, { 0_m, 0_m, 180_stDeg }, "back");
    vexmaps::DistanceSensorModel<vexmaps::DistanceSensorConfiguration>
      laser_model4(&right_sensor, { 0_m, 0_m, 270_stDeg }, "right");

    printf("good on distances\n");

    vexmaps::PFConfiguration Pfconfig;
    Pfconfig.logging = false;

    vexmaps::ParticleFilterModel<2000> pf_model(
      &pf_motion_model,
      { &laser_model1, &laser_model2, &laser_model3, &laser_model4 },
      Pfconfig);

    printf("good on pf\n");

    pf_model.init();
    printf("good on pf.init\n");
    
    // pf_motion_model.init();

    // create the odom task
    pros::Task odom_task{[&] {
        while(true){
            uint32_t current_time = pros::millis();
            pf_motion_model.update();
            pros::c::task_delay_until(&current_time, to_msec(pf_motion_model.getTaskDeltaTime()));
        }
    }};
    printf("good on odom task\n");

    pros::Task pf_task{[&] {
        while(true){
            uint32_t current_time = pros::millis();
            pf_model.update();
            pros::c::task_delay_until(&current_time, to_msec(pf_model.getTaskDeltaTime()));
        }
    }};
    // create a task to automatically update the particle filter
    // pros::Task pf_task = createLocalizationTask(&pf_model);
    printf("good on pf task\n");

    imu.set_heading(0);
    pf_motion_model.setPose({0_in,0_in,0_stRad});

    pf_motion_model.getPose();

    while (true) {
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
        // pros::lcd::print(0,
        //                  "%d %d %d",
        //                  (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
        //                  (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
        //                  (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >>
        //                    0); // Prints status of the emulated screen LCDs
        //
        // // Arcade control scheme
        int dir = master.get_analog(
          ANALOG_LEFT_Y); // Gets amount forward/backward from left joystick
        int turn = master.get_analog(
          ANALOG_RIGHT_X); // Gets the turn left/right from right joystick
        leftMotors.move(dir - turn); // Sets left motor voltage
        rightMotors.move(dir + turn); // Sets right motor voltage
        pros::delay(10); // Run for 20 ms then update
    }
}
