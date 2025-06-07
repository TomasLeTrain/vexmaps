#pragma once

#include "pros/distance.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include "vexmaps/mcl/config.hpp"
#include "vexmaps/mcl/point.hpp"
#include "vexmaps/mcl/sensor.hpp"
#include "vexmaps/mcl/utils.hpp"
#include <cmath>

namespace vexmaps {
class DistanceSensorModel : public Sensor {
    Length measured_distance = 0_m;
    pros::Distance* distance_sensor;

    float std_deviation = 0;
    Angle angle = 0_stDeg;

    units::Pose offsets;

    units::Pose rotated_offsets = { 0_m, 0_m, 0_stDeg };

    // cached values
    double cosa;
    double sina;
    double secant;
    double cosecant;

    Length horizontal_wall_length = wall_length;
    Length vertical_wall_length = wall_length;

    std::string name;

    // determines whether or not readings from this sensor are considered
    bool exit = false;
    bool disabled = false;

  public:
    DistanceSensorModel(const units::Pose offset,
                        pros::Distance* distance_sensor,
                        std::string name)
        : offsets(offset),
          distance_sensor(std::move(distance_sensor)),
          name(name) {}

    void update(Angle angle) override {
        // first check if the distance sensor is available, and if its not then
        // fail non-destructively while still alerting user
        if (distance_sensor == nullptr || !distance_sensor->is_installed()) {
            // not available, just set exit to true
            exit = true;
            printf("ONE OF THE DISTANCE SENSORS ARE NOT CONNECTED CORRECTLY!!");
            return;
        }

        const auto measured_mm = distance_sensor->get();

        this->measured_distance = from_mm(measured_mm);

        // distance sensor doesn't measure anything
        exit = measured_mm == 9999 || disabled;

        this->std_deviation = (1.2_in).internal();

        this->angle = angle;
        const Angle offset_angle = this->angle + this->offsets.orientation;
        this->cosa = units::cos(offset_angle).internal();
        this->sina = units::sin(offset_angle).internal();
        this->secant = 1 / this->cosa;
        this->cosecant = 1 / this->sina;

        // keeps the angle the same
        this->rotated_offsets = rotatePose(this->offsets, this->angle);

        this->rotated_offsets.rotatedBy(this->angle);

        // we will always compare all particles to two walls
        // one vertical and one horizontal
        // since the walls we check are always the same for both we can cache
        // which wall we are checkign
        this->horizontal_wall_length = cosa > 0 ? wall_length : -wall_length;
        this->vertical_wall_length = sina > 0 ? wall_length : -wall_length;

        // simplifies the math even further
        this->horizontal_wall_length =
          horizontal_wall_length - this->rotated_offsets.x;
        this->vertical_wall_length =
          vertical_wall_length - this->rotated_offsets.y;

        if (localization_settings::logging) {
            // expected distance,confidence,std,exit
            std::cout << this->name << ":"
                      << this->measured_distance.convert(in) << ","
                      << this->distance_sensor->get_confidence() << ","
                      << this->std_deviation << ","
                      << (this->exit ? "true" : "false") << ","
                      << this->distance_sensor->get_object_size() << "\n";
        }
    }

    bool hasAvailableReading() override {
        return exit;
    }

    // returns x and y coordinates for which the distance sensor would match
    // measurements could be used to generate particles in case of total system
    // collapse
    Point getExpected() override {
        if (exit) {
            return { infinity() * m, infinity() * m };
        }
        // if we want to generate particles from the measurements themslves we
        // can easily rearrange to get the x and y values for which this
        // measurement would be plausible horizontal_wall_length -
        // measured_distance * this->cosa  = point.x vertical_wall_length -
        // measured_distance * this->sina  = point.y
        return { horizontal_wall_length - measured_distance * this->cosa,
                 vertical_wall_length - measured_distance * this->sina };
    }

    // assumes that its only getting called if exit is false
    // this assumption saves some conditionals improving performance
    float evaluate(const Point& point) override {
        const Length expected_distance =
          units::min((horizontal_wall_length - point.x) * this->secant,
                     (vertical_wall_length - point.y) * this->cosecant);

        const Length difference = expected_distance - measured_distance;

        const float normalCoeff = 1;
        const float randomCoeff = 0.01;
        const float randomUniform = 1 / (2 * wall_length.internal());

        return randomCoeff * randomUniform +
               normalCoeff *
                 newCheapNormalDistribution(difference.internal() /
                                         std_deviation) /
                 std_deviation;
    }

    ~DistanceSensorModel() override = default;
};
} // namespace vexmaps
