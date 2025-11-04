#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/apix.h"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "units/Angle.hpp"
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

struct CustomDistanceSensorConfiguration {
    // all floats without units are in meters
    static constexpr double exp_l = 1.5;
    static constexpr double std_deviation = (2_in).internal();
    static constexpr double map_deviation = (3_in).internal();

    // all these should add to one
    static constexpr double randomCoeff = 0.0;
    static constexpr double expCoeff = 0.15;
    static constexpr double normalCoeff = 0.6;
    static constexpr double mapCoeff = 0.25;

    static constexpr bool logging = general_logging;
    // static constexpr bool logging = false;
};

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

vexmaps::DistanceSensorModel<CustomDistanceSensorConfiguration>
  front_laser_model(&front_sensor,
                    { 5.25_in, 5.4375_in, 0_stDeg },
                    "front",
                    &map_reader);
vexmaps::DistanceSensorModel<CustomDistanceSensorConfiguration>
  left_laser_model(&left_sensor,
                   { 3_in, 5.25_in, 90_stDeg },
                   "left",
                   &map_reader);
vexmaps::DistanceSensorModel<CustomDistanceSensorConfiguration>
  back_laser_model(&back_sensor,
                   { -4_in, -1.84375_in, 180_stDeg },
                   "back",
                   &map_reader);
vexmaps::DistanceSensorModel<CustomDistanceSensorConfiguration>
  right_laser_model(&right_sensor,
                    { 4.25_in, -5.375_in, 270_stDeg },
                    "right",
                    &map_reader);

vexmaps::DistanceSensorModel<CustomDistanceSensorConfiguration>
  fake_distance_model(&fake_distance,
                      { 0_in, 0_in, 0_stDeg },
                      "fake",
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
    std::cout << "entered initialize" << std::endl;
    // pros::c::serctl(SERCTL_DISABLE_COBS, NULL);

    // reset the imu
    // imu.reset(true);

    // must read map before any distance sensor gets used

    auto start_time = pros::millis();
    map_reader.read_compressed("/usd/field.map.compressed");

	if(!map_reader.mapAvailable()){
		std::cout << "try to read uncompressed map" << std::endl;;
		map_reader.read("/usd/field.map");
	}

    auto end_time = pros::millis();
    std::cout << "read map in " << end_time - start_time << " milliseconds."
              << std::endl;

    // initialize all models and create their tasks
    // model_manager.init();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {

    pros::Task([] {
        std::cout << "entered opcontrol" << std::endl;

        float sum_of_dists = 0.0;

        float theta2 = 60;
        int theta = 0;

        // for (; theta <= 720; theta++) {
        // for (int x = -70; x <= 70; x++) {
        //     for (int y = -70; y <= 70; y++) {
        //     }
        // }
        // }

        int n = 5'000;

        std::uniform_real_distribution<float> xs(-70, 70);
        std::uniform_real_distribution<float> ys(-70, 70);
        std::uniform_real_distribution<float> thetas(0, 360);

        Xoshiro128plus rng(10);

        // std::vector<units::FPose> poses(n);
        std::vector<FLength> v_x(n + 100);
        std::vector<FLength> v_y(n + 100);
        // std::vector<units::FPose> poses(n);

        std::vector<float> curr_weights(n + 100);
        std::vector<float> curr_weights2(n + 100);
        std::vector<float> tmp_list(n + 100);

        for (int i = 0; i < n; i++) {
            v_x[i] = xs(rng) * in;
            v_y[i] = ys(rng) * in;
            // poses[i] = { v_x[i], v_y[i], thetas(rng) * deg };
        }

        fake_distance.set_length(30_Fin);
        fake_distance_model.update(60_FstDeg);

        v_x[0] = -20_in;
        v_y[0] = 20_in;

        auto wall_start_time = pros::micros();
        // for (auto& pose : poses) {
        //     FLength query1 = map_reader.query(pose.x, pose.y,
        //     pose.orientation); sum_of_dists += query1.internal();
        // }

        fake_distance_model.evaluate_wall_array(
          curr_weights.data(),
          reinterpret_cast<float*>(v_x.data()),
          reinterpret_cast<float*>(v_y.data()),
          tmp_list.data(),
          n);

        auto wall_end_time = pros::micros();
        // std::cout << "poses[0] is " << poses[0].x.convert(in) << " "
        //           << poses[0].y.convert(in) << " "
        //           << poses[0].orientation.convert(deg) << std::endl;

        auto old_start_time = pros::micros();
        for (int i = 0; i < n; i++) {
            curr_weights2[i] = fake_distance_model.evaluate(v_x[i], v_y[i]);
        }
        auto old_end_time = pros::micros();

        std::cout << "before all, curr_weights[0] = " << curr_weights[0]
                  << std::endl;

        // check if curr weights is valid
        for (int i = 0; i < n; i++) {
            if (auto diff = std::abs(curr_weights2[i] - curr_weights[i]);
                diff > 1e-3) {
                std::cout << "differ by: " << diff << std::endl;
            }
        }
        std::cout << "stopped checking!" << std::endl;

        std::cout << "new wall in  " << wall_end_time - wall_start_time
                  << " microseconds." << std::endl;
        std::cout << "old wall in  " << old_end_time - old_start_time
                  << " microseconds." << std::endl;

        auto all_start_time = pros::micros();
        fake_distance_model.evaluate_array(curr_weights.data(),
                                           v_x.data(),
                                           v_y.data(),
                                           tmp_list.data(),
                                           n);
        auto all_end_time = pros::micros();

        // make it so the above is not optimized away
        for (int i = 0; i < n; i++) {
            sum_of_dists += curr_weights[i];
        }

        std::cout << "all wall in  " << all_end_time - all_start_time
                  << " microseconds." << std::endl;

        std::cout << "result of computation was " << sum_of_dists << std::endl;

        auto query1 = map_reader.query(-20_Fin, 20_Fin, 60_FstDeg);
        std::cout << "custom queyr " << (query1).convert(in) << std::endl;

        std::cout << "curr weight[0] = " << curr_weights[0] << std::endl;
    });

    // std::cout << query1 * in.internal() << std::endl;

    // set the pose
    // model_manager.setPose({ 48_in, -48_in, 0_stDeg });
    //
    // // printf("doing more stuff\n");
    // bool manual_logging = true;
    //
    // while (true) {
    //     if (manual_logging) {
    //         int start_time = pros::millis();
    //         printf(
    //           "start generation\nstart distances\nend distances\nstart "
    //           "parti" "cles" "\n");
    //
    //         printf("%.1f %.1f %.1f\n",
    //                odom_model.getPose().x.convert(in),
    //                odom_model.getPose().y.convert(in),
    //                0.0);
    //         printf("%.1f %.1f %.1f\n",
    //                pf_model.getPose().x.convert(in),
    //                pf_model.getPose().y.convert(in),
    //                5.0);
    //         printf("%.1f %.1f %.1f\n",
    //                smoother_model.getPose().x.convert(in),
    //                smoother_model.getPose().y.convert(in),
    //                10.0);
    //
    //         printf(
    //           "end particles\ntotal weight: 0, time taken: 30000, "
    //           "timestamp:" " %d\n", start_time);
    //         printf("things done:1,1,0,%d\n", particle_count);
    //         printf("prediction:%.1f,%.1f,%.1f\n",
    //                smoother_model.getPose().x.convert(in),
    //                smoother_model.getPose().y.convert(in),
    //                smoother_model.getPose().orientation.convert(deg));
    //         printf("end generation\n");
    //     }
    //
    //     if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
    //         smoother_model.setPose({ 48_in, -48_in, 90_stDeg });
    //     }
    //
    //     // // Arcade control scheme
    //     int dir = master.get_analog(
    //       ANALOG_LEFT_Y); // Gets amount forward/backward from left joystick
    //     int turn = master.get_analog(
    //       ANALOG_RIGHT_X); // Gets the turn left/right from right joystick
    //     leftMotors.move(dir + turn); // Sets left motor voltage
    //     rightMotors.move(dir - turn); // Sets right motor voltage
    //     pros::delay(20); // Run for 20 ms then update
    // }
}
