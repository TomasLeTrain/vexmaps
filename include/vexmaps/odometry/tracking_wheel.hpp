#pragma once

#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "units/units.hpp"
#include <climits>
#include <cmath>

namespace vexmaps {

enum trackingOrientation {vertical, horizontal};

class TrackingWheel {
  public:
    TrackingWheel() {}

    virtual void init() = 0;
    virtual void update() = 0;
    virtual double getDeltaDistance() = 0;
    virtual trackingOrientation getTrackingWheelType() = 0;
    virtual double getOffset() = 0;
    virtual ~TrackingWheel() = default;
};

class MotorGroupTracking : public TrackingWheel {
  private:
    double last_distance;
    double delta_distance;

    double diameter;
    double gear_ratio;
    double offset;

    pros::MotorGroup* motors;

  public:
    MotorGroupTracking(pros::MotorGroup* motors, double diameter, double gear_ratio,double offset)
        : motors(motors), diameter(diameter), gear_ratio(gear_ratio), offset(offset)
    {}

    MotorGroupTracking(pros::MotorGroup* motors, Length diameter, double gear_ratio, Length offset)
        : motors(motors), diameter(to_in(diameter)), gear_ratio(gear_ratio), offset(to_in(offset))
    {}

    void init() override {
        last_distance = 0.0;

        for (double& position : motors->get_position_all()) {
            last_distance += position * (diameter * M_PI) / gear_ratio;
        }

        last_distance /= static_cast<double>(motors->size());
    }

    void update() override {
        double current_distance = 0.0;

        for (double& position : motors->get_position_all()) {
            current_distance += position * diameter / gear_ratio;
        }

        current_distance /= static_cast<double>(motors->size());

        delta_distance = current_distance - last_distance;

        last_distance = current_distance;
    }

    double getDeltaDistance() override {
        return delta_distance;
    }
    double getOffset() override {
        return offset;
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

    pros::Rotation * rotation_sensor;

    public:
    OdometryTracking(pros::Rotation * rotation_sensor, double diameter, double gear_ratio, double offset)
        : rotation_sensor(rotation_sensor), diameter(diameter), gear_ratio(gear_ratio), offset(offset)
    {}
    OdometryTracking(pros::Rotation * rotation_sensor, Length diameter, double gear_ratio, Length offset)
        : rotation_sensor(rotation_sensor), diameter(to_in(diameter)), gear_ratio(gear_ratio), offset(to_in(offset))
    {}

    void init() override {
        rotation_sensor->set_data_rate(5);
        rotation_sensor->reset_position();
        last_position = rotation_sensor->get_position();
    }
        
    void update() override {
        int32_t current_position = rotation_sensor->get_position();
        int32_t position_delta = 0; 

        position_delta = current_position - last_position;

        delta_distance = (static_cast<double>(position_delta) * diameter * M_PI / 36000.0) / gear_ratio;

        last_position = current_position;
    }

    double getDeltaDistance() override {
        return delta_distance;
    }

    double getOffset() override {
        return offset;
    }

    trackingOrientation getTrackingWheelType() override {
        return tracking_orientation;
    }
    ~OdometryTracking() override = default;
};
using VerticalOdometryTracker = OdometryTracking<vertical>;
using HorizontalOdometryTracker = OdometryTracking<horizontal>;
}

