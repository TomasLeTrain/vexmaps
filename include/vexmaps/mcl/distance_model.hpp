#pragma once

#include "pros/distance.hpp"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include "vexmaps/mcl/config.hpp"
#include "vexmaps/mcl/sensor.hpp"
#include "vexmaps/mcl/utils.hpp"
#include <arm_neon.h>
#include <cmath>
#include <optional>

namespace vexmaps {

template<class DistanceSensorConfig>
    requires ValidDistanceConfig<DistanceSensorConfig>
class DistanceSensorModel : public Sensor {
    Length measured_distance = 0_m;
    pros::Distance* distance_sensor;

    Angle angle = 0_stDeg;

    units::Pose offsets;

    units::Pose rotated_offsets = { 0_m, 0_m, 0_stDeg };

    // 2.5 meters is more than what the distance sensor will ever be able to
    // sense
    static constexpr double randomUniformProbability = 1 / (2.54);

    static constexpr double randomFactor =
      DistanceSensorConfig::randomCoeff * randomUniformProbability;

    // precomputed values

    // doubles since they are not used directly
    double cosa;
    double sina;
    double secant;
    double cosecant;

    Length horizontal_wall_length = wall_length;
    Length vertical_wall_length = wall_length;

    // floats since they are used in evaluate
    float Vhor_wall_coeff;
    float Vver_wall_coeff;

    float x_coeff;
    float y_coeff;
    Length hor_wall_coeff;
    Length ver_wall_coeff;

    float expFactor;

    std::string name;

    // determines whether or not readings from this sensor are considered
    bool exit = false;

    bool enabled = true;

  public:
    DistanceSensorModel(pros::Distance* distance_sensor,
                        const units::Pose offset,
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
        exit = measured_mm == 9999 || (!enabled);

        this->angle = angle;
        const Angle offset_angle = this->angle + offsets.orientation;
        // keeps the angle the same
        rotated_offsets = rotatePose(offsets, this->angle);

        // precomputed values
        cosa = units::cos(offset_angle).internal();
        sina = units::sin(offset_angle).internal();

        // make sure they dont equal inf
        secant = 1.0 / (std::max(std::abs(cosa), 0.0001));
        cosecant = 1.0 / (std::max(std::abs(sina), 0.0001));

        secant *= cosa >= 0.0 ? 1.0 : -1.0; // give right sign
        cosecant *= sina >= 0.0 ? 1.0 : -1.0; // give right sign

        // we will always compare all particles to two walls
        // one vertical and one horizontal
        // since the walls we check are always the same for both we can cache
        // the x/y value of the wall for each axis
        horizontal_wall_length = wall_length;
        vertical_wall_length = wall_length;

        horizontal_wall_length *= cosa >= 0.0 ? 1.0 : -1.0;
        vertical_wall_length *= sina >= 0.0 ? 1.0 : -1.0;

        horizontal_wall_length -= rotated_offsets.x;
        vertical_wall_length -= rotated_offsets.y;

        hor_wall_coeff = horizontal_wall_length * secant - measured_distance;
        ver_wall_coeff = vertical_wall_length * cosecant - measured_distance;

        Vhor_wall_coeff = hor_wall_coeff.internal();
        Vver_wall_coeff = ver_wall_coeff.internal();

        x_coeff = -secant;
        y_coeff = -cosecant;

        float expVal = expDistribution<DistanceSensorConfig::exp_l>(
          measured_distance.internal());

        // should actually be applied per particle but would be really
        // computationally expensive
        // only computing it for measured distance should still work fine
        // float expNormFactor =
        //   expNormalizationFactor<DistanceSensorConfig::exp_l>(
        //     measured_distance.internal());

        float expNormFactor = 1;

        // constant in relation to all particles
        // (only depends on measured distance)
        expFactor =
          expNormFactor * expVal * DistanceSensorConfig::expCoeff +
          randomFactor;

        if (DistanceSensorConfig::logging) {
            // name:distance,confidence,std,exit,obj_size
            std::cout << name << ":" << measured_distance.convert(in) << ","
                      << distance_sensor->get_confidence() << ","
                      << DistanceSensorConfig::std_deviation << ","
                      << (exit ? "true" : "false") << ","
                      << distance_sensor->get_object_size() << "\n";
        }
    }

    bool hasAvailableReading() override {
        return !exit;
    }

    bool getVectorized() override {
        return true;
    }

    bool getVectorized2() override {
        return false;
    }

    inline float evaluate(const units::V2Position& point) override {
        return evaluate(point.x, point.y);
    }

    // assumes that its only getting called if exit is false
    // this assumption saves some conditionals improving performance
    inline float evaluate(Length x, Length y) override {
        const Length difference = units::min(hor_wall_coeff + x * x_coeff,
                                             ver_wall_coeff + y * y_coeff);

        float normal_dist =
          NormalDistributionApproximation<DistanceSensorConfig::std_deviation,
                                          DistanceSensorConfig::normalCoeff>(
            difference.internal());

        if (difference.internal() >= 0) {
            return normal_dist + expFactor;
        } else {
            return normal_dist + randomFactor;
        }
    }

    // assumes that its only getting called if exit is false
    // this assumption saves some conditionals improving performance
    inline float32x4_t Vevaluate(float32x4_t x, float32x4_t y) override {
        // clang-format off
        //
        // expected_distance =
        //   min( (horizontal_wall_length - point.x) * secant,
        //        (vertical_wall_length   - point.y) * cosecant );
        //
        // hor_wall = (horizontal_wall_length - x) * this->secant
        // hor_wall = (horizontal_wall_length * this->secant) + x * (-this->secant)
        // hor_wall = HC [all precomputed]                    + x * x_coeff [precomputed] 
        //
        // -- direct difference formulation --
        // difference = expected_distance - measured_distance
        // difference = min(HC,VC) - measured_distance  ==>  min(HC - measured_distance, VC - measured_distance)
        //
        // HC                            - measured_distance
        // (hor_wall_coeff - x * secant) - measured_distance
        // (hor_wall_coeff - measured_distance) + x * x_coeff
        // hor_wall_coeff [new coeff]           + x * x_coeff
        //
        //
        // in the end this results in:
        // hor_wall_coeff = horizontal_wall_length * secant   - measured_distance
        // ver_wall_coeff = vertical_wall_length   * cosecant - measured_distance
        // x_coeff = -secant
        // y_coeff = -cosecant
        //
        // clang-format on

        // HC = hor_wall_coeff + point.x * (-secant)
        // VC = ver_wall_coeff + point.y * (-cosecant)
        float32x4_t HC = vmlaq_n_f32(vdupq_n_f32(Vhor_wall_coeff), x, x_coeff);
        float32x4_t VC = vmlaq_n_f32(vdupq_n_f32(Vver_wall_coeff), y, y_coeff);

        // difference = min(HC,VC)
        float32x4_t difference = vminq_f32(HC, VC);

        uint32x4_t modMask = vcgeq_f32(difference, vdupq_n_f32(0.0));

        // constantFactor = measured <= expected ? expFactor : randomFactor
        float32x4_t VMaskedConstantFactor =
          vbslq_f32(modMask, vdupq_n_f32(expFactor), vdupq_n_f32(randomFactor));

        float32x4_t normal_dist =
          VNormalDistributionApproximation<DistanceSensorConfig::std_deviation,
                                           DistanceSensorConfig::normalCoeff>(
            difference);

        return vaddq_f32(normal_dist, VMaskedConstantFactor);
    }

    ~DistanceSensorModel() override = default;

    std::optional<units::V2Position> getExpected() override {
        return std::nullopt;
    }

    void disable() override {
        enabled = false;
    }

    void enable() override {
        enabled = true;
    }

    bool getEnabled() override {
        return enabled;
    }
};
} // namespace vexmaps
