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

class MotorGroupTracking : public TrackingWheel {
  private:
    std::optional<Length> last_distance = std::nullopt;
    Length delta_distance = 0_m;

    pros::MotorGroup* motors;

    Length diameter;
    AngularVelocity rpm;
    Length offset;

  public:
    MotorGroupTracking(pros::MotorGroup* motors,
                       Length diameter,
                       AngularVelocity rpm,
                       Length offset)
        : motors(motors),
          diameter(diameter),
          rpm(rpm),
          offset(offset) {}

    void init() override {}

    void update() override {
        Length res = 0_in;
        double count = 0;
        for (auto position : motors->get_raw_position_all(NULL)) {
            if (position == PROS_ERR) continue;

            Number rotations =
              (rpm * static_cast<double>(position)) / (3600_rpm * 50.0);

            res += rotations * (diameter * M_PI);
            count += 1.0;
        }

        Length current = INFINITY * m;

        if (count != 0) {
            current = res / count;
        }

        if (std::isfinite(current.internal()) && last_distance) {
            delta_distance = current - *last_distance;
        } else {
            delta_distance = 0_m;
        }

        last_distance = current;
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
    pros::Rotation* rotation_sensor;
    Length diameter;
    double gear_ratio;
    Length offset;

    int32_t last_position;
    Length delta_distance;

    bool available = true;

    // makes it so that when available changes to true, it is not immediately
    // used
    bool available_changed = false;

    bool m_print_on_error = false;

    bool m_disabled = false;

  public:
    OdometryTracking(pros::Rotation* rotation_sensor,
                     FLength diameter,
                     double gear_ratio,
                     FLength offset,
                     bool disabled = false,
                     bool print_on_error = false)
        : rotation_sensor(rotation_sensor),
          diameter(diameter),
          gear_ratio(gear_ratio),
          offset(offset),
          m_disabled(disabled),
          m_print_on_error(print_on_error) {}

    void init() override {
        if (rotation_sensor == nullptr) {
            if (m_print_on_error)
                // done here so it only gets printed once once
                printf(
                  "WARNING: ROTATION IS NULL - Check config for nullptr! - "
                  "remove from list of trackers if this tracker is not used\n");
            available = false;
            return;
        }
        if (!rotation_sensor->is_installed()) {
            if (m_print_on_error) printf("WARNING: ROTATION NOT CONNECTED!\n");
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
            if (available) {
                if (m_print_on_error)
                    printf("WARNING: ROTATION IS NULL - Check config for "
                           "nullptr! - remove from list of trackers if this "
                           "tracker is not used\n");
            }
            available = false;
            available_changed = false;
            return;
        }

        if (!rotation_sensor->is_installed()) {
            // doesn't print always to let user see other possible messsages
            if (available) {
                if (m_print_on_error)
                    printf("WARNING: ROTATION NOT CONNECTED!\n");
            }
            available = false;
            available_changed = false;
            return;
        }

        // if the sensor was not available the function would have returned by
        // now this means that the sensor is recoverable
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
        return (!m_disabled) && available && !available_changed;
    }

    trackingOrientation getTrackingWheelType() override {
        return tracking_orientation;
    }

    bool getDisabled() {
        return m_disabled;
    }

    void setDisabled(bool disabled) {
        m_disabled = disabled;
    }

    ~OdometryTracking() override = default;
};

using VerticalOdometryTracker = OdometryTracking<vertical>;
using HorizontalOdometryTracker = OdometryTracking<horizontal>;
} // namespace vexmaps
