#pragma once

#include "pros/abstract_motor.hpp"
#include "pros/apix.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include <climits>
#include <cmath>

namespace vexmaps {

enum trackingOrientation {
    vertical,
    horizontal
};

class TrackingWheel {
  public:
    TrackingWheel() {}

    virtual void init() = 0;
    virtual void update() = 0;
    virtual Length getDeltaDistance() = 0;
    virtual trackingOrientation getTrackingWheelType() = 0;
    virtual Length getOffset() = 0;
    virtual bool getAvailable() = 0;
    virtual ~TrackingWheel() = default;
};

inline double getGearingTicks(pros::MotorGears gearing) {
    double gearing_multiplier = 1;
    switch (gearing) {
        case pros::MotorGears::blue: gearing_multiplier = 300.0; break;
        case pros::MotorGears::green: gearing_multiplier = 900.0; break;
        case pros::MotorGears::red: gearing_multiplier = 1800.0; break;
        default: gearing_multiplier = 1; break;
    }
    return gearing_multiplier;
}

inline AngularVelocity getGearingRPM(pros::MotorGears gearing) {
    AngularVelocity gearing_multiplier = 200_rpm;
    switch (gearing) {
        case pros::MotorGears::blue: gearing_multiplier = 600_rpm; break;
        case pros::MotorGears::green: gearing_multiplier = 200_rpm; break;
        case pros::MotorGears::red: gearing_multiplier = 100_rpm; break;
        default: gearing_multiplier = 200_rpm; break;
    }
    return gearing_multiplier;
}

class MotorGroupTracking : public TrackingWheel {
  private:
    Length last_distance;
    Length delta_distance;

    Length diameter;
    AngularVelocity rpm;
    Length offset;

    pros::MotorGroup* motors;

  public:
    MotorGroupTracking(pros::MotorGroup* motors,
                       Length diameter,
                       AngularVelocity rpm,
                       Length offset)
        : motors(motors),
          diameter(diameter),
          rpm(rpm),
          offset(offset) {}

    Length calculateDistance() {
        Length distance = 0_in;

        if (motors == nullptr) {
            printf("odometry: motor group is a nullptr!\n");
            return 0_in;
        }

        int used_motor_count = 0;

        for (int i = 0; i < motors->size(); i++) {
            int port = abs(motors->get_port(i));
            // check if is installed
            auto plugged_device_type =
              (pros::DeviceType)pros::c::registry_get_plugged_type(port - 1);

            // only include if plugged in
            if (plugged_device_type == pros::DeviceType::motor) {
                pros::MotorGears gearing = motors->get_gearing(i);
                pros::MotorUnits encoder = motors->get_encoder_units(i);

                double rotation_multiplier =
                  1; // should convert position to # of rotations

                switch (encoder) {
                    case pros::MotorUnits::degrees:
                        rotation_multiplier = 1 / 360.0;
                        break;
                    case pros::MotorUnits::counts:
                        rotation_multiplier = 1 / getGearingTicks(gearing);
                        break;
                    case pros::MotorUnits::rotations:
                        rotation_multiplier = 1;
                        break;
                    default: rotation_multiplier = 1; break;
                }

                // number of rotations the wheel has traveled
                double rotations =
                  motors->get_position(i) * rotation_multiplier;

                const Length circumference = diameter * M_PI;

                // rotations * circumference of wheel
                distance += rotations * circumference *
                            (this->rpm / getGearingRPM(gearing));

                used_motor_count++;
            }
        }

        if (used_motor_count != 0)
            distance /= static_cast<double>(used_motor_count);
        return distance;
    }

    void init() override {
        last_distance = calculateDistance();
    }

    void update() override {
        Length current_distance = calculateDistance();

        delta_distance = current_distance - last_distance;

        last_distance = current_distance;
    }

    Length getDeltaDistance() override {
        return delta_distance;
    }

    Length getOffset() override {
        return offset;
    }

    bool getAvailable() override {
        return true;
    }

    trackingOrientation getTrackingWheelType() override {
        return vertical;
    }

    ~MotorGroupTracking() override = default;
};

template<trackingOrientation tracking_orientation>
class OdometryTracking : public TrackingWheel {
  private:
    Length last_distance;
    int32_t last_position;

    Length delta_distance;

    Length offset;

    Length diameter;
    double gear_ratio;

    bool available = true;

    // makes it so that when available changes to true, it is not immediately used
    bool available_changed = false;

    pros::Rotation* rotation_sensor;

  public:
    OdometryTracking(pros::Rotation* rotation_sensor,
                     FLength diameter,
                     double gear_ratio,
                     FLength offset)
        : rotation_sensor(rotation_sensor),
          diameter(diameter),
          gear_ratio(gear_ratio),
          offset(offset) {}

    void init() override {
        if (rotation_sensor == nullptr) {
            // done here so it only gets printed once once
            printf(
              "WARNING: ROTATION IS NULL - Check config for nullptr! - " "remov" "e " "from " "list " "of " "track" "ers " "if " "this " "track" "er " "is " "not " "used" "\n");
            available = false;
            return;
        }
        if (!rotation_sensor->is_installed()) {
            printf("WARNING: ROTATION NOT CONNECTED!\n");
            available = false;
            return;
        }

        // rotation_sensor exists and is plugged in
        rotation_sensor->set_data_rate(5);
        rotation_sensor->reset_position();
        last_position = rotation_sensor->get_position();
    }

    void update() override {
        if (rotation_sensor == nullptr) {
            // doesn't print always to let user see other possible messsages
            if(available){
                printf("WARNING: ROTATION IS NULL - Check config for nullptr! - "
                        "remove from list of trackers if this tracker is not used\n");
            }
            available = false;
            available_changed = false;
            return;
        }

        if (!rotation_sensor->is_installed()) {
            // doesn't print always to let user see other possible messsages
            if(available){
                printf("WARNING: ROTATION NOT CONNECTED!\n");
            }
            available = false;
            available_changed = false;
            return;
        }

        // if the sensor was not available the function would have returned by now
        // this means that the sensor is recoverable
        if (!available) {
            // we could be able to recover and still use the tracker
            rotation_sensor->set_data_rate(5);
            rotation_sensor->reset_position();
            last_position = rotation_sensor->get_position();

            // change available to not run through this again
            available = true;
            available_changed = true;

            // we cant really know the delta right now so we will returning with
            // available set to false but if its connected it will become true
            return;
        }

        // rotation_sensor exists and is plugged in

        available = true;
        available_changed = false;

        int32_t current_position = rotation_sensor->get_position();

        // kept track in int to avoid precision loss
        // number of degrees the rotation sensor has spun
        int32_t position_delta = current_position - last_position;

        const Length circumference = diameter * M_PI;

        delta_distance =
          ((static_cast<double>(position_delta) * circumference) / 36000.0) /
          gear_ratio;

        last_position = current_position;
    }

    Length getDeltaDistance() override {
        return delta_distance;
    }

    Length getOffset() override {
        return offset;
    }

    bool getAvailable() override {
        return available && !available_changed;
    }

    trackingOrientation getTrackingWheelType() override {
        return tracking_orientation;
    }

    ~OdometryTracking() override = default;
};

using VerticalOdometryTracker = OdometryTracking<vertical>;
using HorizontalOdometryTracker = OdometryTracking<horizontal>;
} // namespace vexmaps
