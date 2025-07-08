#pragma once

#include "pros/abstract_motor.hpp"
#include "pros/apix.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"
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
    virtual double getDeltaDistance() = 0;
    virtual trackingOrientation getTrackingWheelType() = 0;
    virtual double getOffset() = 0;
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

inline double getGearingRPM(pros::MotorGears gearing) {
    double gearing_multiplier = 1;
    switch (gearing) {
        case pros::MotorGears::blue: gearing_multiplier = 600.0; break;
        case pros::MotorGears::green: gearing_multiplier = 200.0; break;
        case pros::MotorGears::red: gearing_multiplier = 100.0; break;
        default: gearing_multiplier = 200.0; break;
    }
    return gearing_multiplier;
}

class MotorGroupTracking : public TrackingWheel {
  private:
    double last_distance;
    double delta_distance;

    double diameter;
    double rpm;
    double offset;

    pros::MotorGroup* motors;

  public:
    MotorGroupTracking(pros::MotorGroup* motors,
                       double diameter,
                       double rpm,
                       double offset)
        : motors(motors),
          diameter(diameter),
          rpm(rpm),
          offset(offset) {}

    MotorGroupTracking(pros::MotorGroup* motors,
                       Length diameter,
                       double rpm,
                       Length offset)
        : motors(motors),
          diameter(to_in(diameter)),
          rpm(rpm),
          offset(to_in(offset)) {}

    double calculateDistance() {
        double distance = 0.0;

        if(motors == nullptr){
            printf("odometry: motor group is a nullptr!\n");
            return 0.0;
        }

        for (int i = 0; i < motors->size(); i++) {
            auto port = motors->get_port(i);
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
                    default:
                        rotation_multiplier = 1;
                        break;
                }

                double position = motors->get_position(i);

                double gear_ratio = 1;

                distance += (position * rotation_multiplier) *
                            (diameter * M_PI) * (this->rpm / getGearingRPM(gearing));
            }
        }

        distance /= static_cast<double>(motors->size());
        return distance;
    }

    void init() override {
        last_distance = calculateDistance();
    }

    void update() override {
        double current_distance = calculateDistance();

        delta_distance = current_distance - last_distance;

        last_distance = current_distance;
    }

    double getDeltaDistance() override {
        return delta_distance;
    }

    double getOffset() override {
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
    double last_distance;
    int32_t last_position;

    double delta_distance;

    double offset;

    double diameter;
    double gear_ratio;

    bool available = true;

    pros::Rotation* rotation_sensor;

  public:
    OdometryTracking(pros::Rotation* rotation_sensor,
                     double diameter,
                     double gear_ratio,
                     double offset)
        : rotation_sensor(rotation_sensor),
          diameter(diameter),
          gear_ratio(gear_ratio),
          offset(offset) {}

    OdometryTracking(pros::Rotation* rotation_sensor,
                     Length diameter,
                     double gear_ratio,
                     Length offset)
        : rotation_sensor(rotation_sensor),
          diameter(to_in(diameter)),
          gear_ratio(gear_ratio),
          offset(to_in(offset)) {}

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
            // printf("WARNING: ROTATION IS NULL - Check config for nullptr! - "
            // "remove from list of trackers if this tracker is not used\n");
            available = false;
            return;
        }

        if (!rotation_sensor->is_installed()) {
            // doesn't print always to let user see other possible messsages
            // printf("WARNING: ROTATION NOT CONNECTED!\n");
            available = false;
            return;
        }

        if (!available) {
            // we could be able to recover and still use the tracker
            rotation_sensor->set_data_rate(5);
            rotation_sensor->reset_position();
            last_position = rotation_sensor->get_position();

            // we cant really know the delta right now so we will returning with
            // available set to false but if its connected it will become true
            return;
        }

        // rotation_sensor exists and is plugged in

        available = true;

        int32_t current_position = last_position;
        current_position = rotation_sensor->get_position();
        int32_t position_delta = 0;

        // kept track in int to avoid precision loss
        position_delta = current_position - last_position;

        delta_distance =
          ((static_cast<double>(position_delta) * diameter * M_PI) / 36000.0) /
          gear_ratio;

        last_position = current_position;
    }

    double getDeltaDistance() override {
        return delta_distance;
    }

    double getOffset() override {
        return offset;
    }

    bool getAvailable() override {
        return available;
    }

    trackingOrientation getTrackingWheelType() override {
        return tracking_orientation;
    }

    ~OdometryTracking() override = default;
};

using VerticalOdometryTracker = OdometryTracking<vertical>;
using HorizontalOdometryTracker = OdometryTracking<horizontal>;
} // namespace vexmaps
