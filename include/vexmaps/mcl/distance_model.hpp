#pragma once

#include "pros/distance.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include "vexmaps/mcl/config.hpp"
#include "vexmaps/mcl/point.hpp"
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
    static constexpr double normalFactor =
      DistanceSensorConfig::normalCoeff / DistanceSensorConfig::std_deviation;

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
    float constantFactor;

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

        // not done for logging
        // if(exit) return;

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

        // simplifies the math even further
        horizontal_wall_length -= rotated_offsets.x;
        vertical_wall_length -= rotated_offsets.y;

        hor_wall_coeff = horizontal_wall_length * secant - measured_distance;
        ver_wall_coeff = vertical_wall_length * cosecant - measured_distance;

        hor_wall_coeff /= DistanceSensorConfig::std_deviation;
        ver_wall_coeff /= DistanceSensorConfig::std_deviation;

        Vhor_wall_coeff = hor_wall_coeff.internal();
        Vver_wall_coeff = ver_wall_coeff.internal();

        x_coeff = secant / DistanceSensorConfig::std_deviation;
        y_coeff = cosecant / DistanceSensorConfig::std_deviation;

        float expNormalizationFactor =
          expNormalizationFactor<DistanceSensorConfig::exp_l>(
            measured_distance.internal());

        // constant in relation to all particles
        // (only depends on measured distance)
        expFactor = expNormalizationFactor * DistanceSensorConfig::expCoeff *
                    expDistribution<DistanceSensorConfig::exp_l>(
                      measured_distance.internal());

        constantFactor = randomFactor + expFactor;

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
        return true;
    }

    inline float evaluate(const Point& point) override {
        return evaluate(point.x, point.y);
    }

    // assumes that its only getting called if exit is false
    // this assumption saves some conditionals improving performance
    inline float evaluate(Length x, Length y) override {
        const Length mod_difference = units::min(hor_wall_coeff - x * x_coeff,
                                                 ver_wall_coeff - y * y_coeff);

        auto res = randomFactor;
        res +=
          NormalDistributionApproximation<static_cast<double>(normalFactor)>(
            mod_difference.internal());
        if (mod_difference.internal() >= 0) res += expFactor;
        return res;
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
        // -- expected distance formulation --
        // hor_wall = (horizontal_wall_length - point) * this->secant
        // hor_wall = (horizontal_wall_length * this->secant) - point * this->secant
        // hor_wall = HC [all precomputed]                    - point * this->secant
        //
        // -- direct difference formulation --
        // difference = expected_distance - measured_distance
        // difference = min(HC,VC) - measured_distance  ==>  min(HC - measured_distance, VC - measured_distance)
        //
        // HC                            - measured_distance
        // (hor_wall_coeff - x * secant) - measured_distance
        // (hor_wall_coeff - measured_distance) - x * secant
        // hor_wall_coeff [new coeff]           - x * secant
        //
        // -- precomputed std_deviation formulation --
        // normal_dist = normal_dist_pdf(difference / std_dev)
        //
        // Assuming std_dev is positive -->
        // mod_difference = difference / std_dev = min(HC,VC) / std_dev
        // -->
        // (hor_wall_coeff - x * secant) / std_dev
        // (hor_wall_coeff / std_dev) - x * (secant / std_dev)
        // hor_wall_coeff [new coeff] - x * x_coeff
        //
        // in the end this results in:
        // hor_wall_coeff = (horizontal_wall_length * secant   - measured_distance) / std_dev
        // ver_wall_coeff = (vertical_wall_length   * cosecant - measured_distance) / std_dev
        // x_coeff = secant   / std_dev
        // y_coeff = cosecant / std_dev
        //
        //
        // clang-format on

        float32x4_t HC = vdupq_n_f32(Vhor_wall_coeff);
        float32x4_t VC = vdupq_n_f32(Vver_wall_coeff);

        // HC = hor_wall_coeff - point.x * (secant * r_std_dev)
        // VC = ver_wall_coeff - point.y * (cosecant * r_std_dev)
        HC = vmlsq_n_f32(HC, x, x_coeff);
        VC = vmlsq_n_f32(VC, y, y_coeff);

        // difference = min(HC,VC)
        float32x4_t mod_difference = vminq_f32(HC, VC);

        uint32x4_t modMask = vcgeq_f32(mod_difference, vdupq_n_f32(0));

        float32x4_t VconstantFactor = vbslq_f32(modMask,
                                                vdupq_n_f32(constantFactor),
                                                vdupq_n_f32(randomFactor));

        float32x4_t normal_dist =
          VNormalDistributionApproximation<static_cast<double>(normalFactor)>(
            mod_difference);

        // res = randomFactor + normal_dist_pdf * normalFactor
        return vaddq_f32(VconstantFactor, normal_dist);
    }

    ~DistanceSensorModel() override = default;

    std::optional<Point> getExpected() override {
        return std::nullopt;
    }

    // in theory registers should get reused
    inline void Vevaluate2(float32x4_t x1,
                           float32x4_t y1,
                           float32x4_t x2,
                           float32x4_t y2,
                           float32x4_t* res1,
                           float32x4_t* res2) override {
        float32x4_t HC1 = vdupq_n_f32(Vhor_wall_coeff);
        float32x4_t VC1 = vdupq_n_f32(Vver_wall_coeff);

        float32x4_t HC2 = vdupq_n_f32(Vhor_wall_coeff);
        float32x4_t VC2 = vdupq_n_f32(Vver_wall_coeff);

        // HC = hor_wall_coeff - point.x * (secant * r_std_dev)
        // VC = ver_wall_coeff - point.y * (cosecant * r_std_dev)
        HC1 = vmlsq_n_f32(HC1, x1, x_coeff);
        VC1 = vmlsq_n_f32(VC1, y1, y_coeff);

        HC2 = vmlsq_n_f32(HC2, x2, x_coeff);
        VC2 = vmlsq_n_f32(VC2, y2, y_coeff);

        // difference = min(HC,VC)
        float32x4_t mod_difference1 = vminq_f32(HC1, VC1);
        float32x4_t mod_difference2 = vminq_f32(HC2, VC2);

        float32x4_t Vzero = vdupq_n_f32(0);
        uint32x4_t modMask1 = vcgeq_f32(mod_difference1, Vzero);
        uint32x4_t modMask2 = vcgeq_f32(mod_difference2, Vzero);

        float32x4_t VconstantFac = vdupq_n_f32(constantFactor);
        float32x4_t VrandomFactor = vdupq_n_f32(randomFactor);

        float32x4_t VconstantFactor1 =
          vbslq_f32(modMask1, VconstantFac, VrandomFactor);

        float32x4_t VconstantFactor2 =
          vbslq_f32(modMask2, VconstantFac, VrandomFactor);

        float32x4_t normal_dist1 =
          VNormalDistributionApproximation<static_cast<double>(normalFactor)>(
            mod_difference1);
        float32x4_t normal_dist2 =
          VNormalDistributionApproximation<static_cast<double>(normalFactor)>(
            mod_difference2);

        // res = randomFactor + normal_dist_pdf * normalFactor
        *res1 = vaddq_f32(VconstantFactor1, normal_dist1);
        *res2 = vaddq_f32(VconstantFactor2, normal_dist2);
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
