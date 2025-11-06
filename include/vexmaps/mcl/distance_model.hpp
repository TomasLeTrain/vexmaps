#pragma once

#include "pros/distance.hpp"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include "vexmaps/mcl/asm_functions.hpp"
#include "vexmaps/mcl/config.hpp"
#include "vexmaps/mcl/map_reader.hpp"
#include "vexmaps/mcl/sensor.hpp"
#include "vexmaps/mcl/utils.hpp"
#include <arm_neon.h>
#include <cmath>
#include <memory>
#include <optional>

namespace vexmaps {

template<class DistanceSensorConfig>
    requires ValidDistanceConfig<DistanceSensorConfig>
class DistanceSensorModel : public Sensor {

    pros::Distance* distance_sensor;
    units::Pose offsets;
    std::string name;

    // optional
    MapReader<>* map_reader;

    units::FPose rotated_offsets = { 0_m, 0_m, 0_stDeg };
    Length measured_distance = 0_m;

    // determines whether or not readings from this sensor are used - false when
    // there are no measurements
    bool exit = false;

    // when false, sensor is not used regardless of measurements
    bool enabled = true;

    // 2.5 meters is more than what the distance sensor will ever be able to
    // sense
    static constexpr double randomUniformProbability = 1 / (2.54);

    static constexpr double randomFactor =
      DistanceSensorConfig::randomCoeff * randomUniformProbability;

    // precomputed values

    // values used per particle evaluation - should all be floats
    float Vhor_wall_coeff;
    float Vver_wall_coeff;

    float fsina, fcosa;

    // held in degrees specifically for map lookup
    // defined as theta * 2 , constrained between [0,720]
    FAngle map_angle;

    float f_measured_distance = 0;

    float x_coeff;
    float y_coeff;
    FLength hor_wall_coeff;
    FLength ver_wall_coeff;
    float expFactor;

    // used for getExpected
    FLength horizontal_wall_length;
    FLength vertical_wall_length;

  public:
    DistanceSensorModel(pros::Distance* distance_sensor,
                        const units::Pose offset,
                        std::string name,
                        MapReader<>* map_reader = nullptr)
        : distance_sensor(std::move(distance_sensor)),
          offsets(offset),
          name(name),
          map_reader(map_reader) {}

    void update(Angle angle) override {
        // first check if the distance sensor is available, and if its not then
        // fail non-destructively while still alerting user
        if (distance_sensor == nullptr || !distance_sensor->is_installed()) {
            // not available, just set exit to true
            exit = true;
            printf(
              "ONE OF THE DISTANCE SENSORS ARE NOT CONNECTED CORRECTLY!!\n");
            return;
        }

        const int32_t measured_mm = distance_sensor->get();

        measured_distance = from_mm(measured_mm);
        f_measured_distance = measured_distance.internal();

        // distance sensor doesn't measure anything
        exit = measured_mm == 9999 || (!enabled);

        // rotates offset and angle
        rotated_offsets = FrotatePose(offsets, angle);

        map_angle = units::constrainAngle2pi(rotated_offsets.orientation);

        // precomputed values
        double cosa = units::cos(rotated_offsets.orientation);
        double sina = units::sin(rotated_offsets.orientation);

        fcosa = cosa;
        fsina = sina;

        double cos_sign = cosa >= 0.0 ? 1.0 : -1.0;
        double sin_sign = sina >= 0.0 ? 1.0 : -1.0;

        // avoid division by zero
        double secant = cos_sign / (std::max(std::abs(cosa), 0.0001));
        double cosecant = sin_sign / (std::max(std::abs(sina), 0.0001));

        // we will always compare all particles to two walls
        // one vertical and one horizontal
        // since the walls we check are always the same for both we can cache
        // the x/y value of the wall for each axis
        Length original_horizontal_wall_length = wall_length * cos_sign;
        Length original_vertical_wall_length = wall_length * sin_sign;

        horizontal_wall_length =
          original_horizontal_wall_length - rotated_offsets.x;
        vertical_wall_length =
          original_vertical_wall_length - rotated_offsets.y;

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

        // constant in relation to all particles
        // (only depends on measured distance)
        expFactor = expVal * DistanceSensorConfig::expCoeff + randomFactor;

        if (DistanceSensorConfig::logging) {
            // name:distance,confidence,std,exit,obj_size
            std::cout << name << ":" << measured_distance.convert(in) << ","
                      << distance_sensor->get_confidence() << ","
                      << DistanceSensorConfig::std_deviation << ","
                      << (exit ? "true" : "false") << ","
                      << distance_sensor->get_object_size() << "\n";
        }
    }

    // assumes that its only getting called if exit is false
    // this assumption saves some conditionals improving performance
    float evaluate(FLength x, FLength y) override {
        const FLength difference = units::min(hor_wall_coeff + x * x_coeff,
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
    float32x4_t Vevaluate(float32x4_t x, float32x4_t y) override {
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

        // each number is all UINT_MAX if (expected - measured) is >= 0, else
        // its 0
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

    void evaluate_wall_array(float* curr_weights,
                             float* x,
                             float* y,
                             float* tmp_array,
                             int len) {
        float32x4_t Vhor = vdupq_n_f32(Vhor_wall_coeff);
        float32x4_t Vver = vdupq_n_f32(Vver_wall_coeff);

        for (int i = 0; i < len; i += 16) {
            float32x4_t Vx1 = vld1q_f32(&x[i]);
            float32x4_t Vy1 = vld1q_f32(&y[i]);

            float32x4_t Vx2 = vld1q_f32(&x[i + 4]);
            float32x4_t Vy2 = vld1q_f32(&y[i + 4]);

            float32x4_t Vx3 = vld1q_f32(&x[i + 8]);
            float32x4_t Vy3 = vld1q_f32(&y[i + 8]);

            float32x4_t Vx4 = vld1q_f32(&x[i + 12]);
            float32x4_t Vy4 = vld1q_f32(&y[i + 12]);

            float32x4_t Vxx1 = vmulq_n_f32(Vx1, x_coeff);
            float32x4_t Vyy1 = vmulq_n_f32(Vy1, y_coeff);

            float32x4_t Vxx2 = vmulq_n_f32(Vx2, x_coeff);
            float32x4_t Vyy2 = vmulq_n_f32(Vy2, y_coeff);

            float32x4_t Vxx3 = vmulq_n_f32(Vx3, x_coeff);
            float32x4_t Vyy3 = vmulq_n_f32(Vy3, y_coeff);

            float32x4_t Vxx4 = vmulq_n_f32(Vx4, x_coeff);
            float32x4_t Vyy4 = vmulq_n_f32(Vy4, y_coeff);

            float32x4_t HC1 = vaddq_f32(Vhor, Vxx1);
            float32x4_t VC1 = vaddq_f32(Vver, Vyy1);

            float32x4_t HC2 = vaddq_f32(Vhor, Vxx2);
            float32x4_t VC2 = vaddq_f32(Vver, Vyy2);

            float32x4_t HC3 = vaddq_f32(Vhor, Vxx3);
            float32x4_t VC3 = vaddq_f32(Vver, Vyy3);

            float32x4_t HC4 = vaddq_f32(Vhor, Vxx4);
            float32x4_t VC4 = vaddq_f32(Vver, Vyy4);

            // difference = min(HC,VC)
            float32x4_t difference1 = vminq_f32(HC1, VC1);
            float32x4_t difference2 = vminq_f32(HC2, VC2);
            float32x4_t difference3 = vminq_f32(HC3, VC3);
            float32x4_t difference4 = vminq_f32(HC4, VC4);

            vst1q_f32(&tmp_array[i], difference1);
            vst1q_f32(&tmp_array[i + 4], difference2);
            vst1q_f32(&tmp_array[i + 8], difference3);
            vst1q_f32(&tmp_array[i + 12], difference4);
        }

        VNormalDistributionPDF(
          curr_weights,
          tmp_array,
          len,
          0, // difference already applied, mean is just zero
          DistanceSensorConfig::std_deviation,
          DistanceSensorConfig::normalCoeff);

        // gets vectorized?
        for (int i = 0; i < len; i++) {
            // measured <= expected ? expFactor : randomFactor
            // 0 <= (expected - measured) ? expFactor : randomFactor
            // 0 <= tmparray ? expFactor : randomFactor
            if (0 <= tmp_array[i]) {
                curr_weights[i] += expFactor;
            } else {
                curr_weights[i] += randomFactor;
            }
        }
    }

    void map_lookup_array(float* curr_weights,
                          FLength* x,
                          FLength* y,
                          float* tmp_array,
                          int len) {
        if (map_reader == nullptr || !map_reader->mapAvailable()) {
            // falls back to just adding a constant?
            // for (int i = 0; i < len; i++) {
            //     // curr_weights[i] += 0.1;
            // }
            return;
        }

        for (int i = 0; i < len; i++) {
            tmp_array[i] = map_reader
                             ->query(x[i] + rotated_offsets.x,
                                     y[i] + rotated_offsets.y,
                                     map_angle)
                             .internal();
        }

        VNormalDistributionPDF(curr_weights,
                               tmp_array,
                               len,
                               f_measured_distance,
                               DistanceSensorConfig::map_deviation,
                               DistanceSensorConfig::mapCoeff);
    }

    void evaluate_array(float* curr_weights,
                        FLength* x,
                        FLength* y,
                        float* tmp_array,
                        size_t len) override {

        // TODO: maybe multiply by some number <= 1.0 instead?
        if (exit) {
            // TODO: this should never get called?
            //
            // makes it as if sensor did not get processed
            std::fill(curr_weights, curr_weights + len, 1.0f);
            return;
        }

        // set all curr_weights equal to zero
        std::fill(curr_weights, curr_weights + len, 0.0f);

        evaluate_wall_array(curr_weights,
                            reinterpret_cast<float*>(x),
                            reinterpret_cast<float*>(y),
                            tmp_array,
                            len);
        map_lookup_array(curr_weights, x, y, tmp_array, len);
    }

    // returns x and y coordinates for which the distance sensor would match
    // measurements.
    // can be used for distance sensor resets
    std::optional<units::V2FPosition> getExpected() override {
        if (exit) {
            return std::nullopt;
        }

        // std::cout << std::format("offset is {}, {}, {}. hor/ver wall is {},{}, "
        //                          "measured is {}, fcos/sin {},{}",
        //                          rotated_offsets.x.convert(in),
        //                          rotated_offsets.y.convert(in),
        //                          rotated_offsets.orientation.convert(deg),
        //                          horizontal_wall_length.convert(in),
        //                          vertical_wall_length.convert(in),
        //                          measured_distance.convert(in),
        //                          fcosa,
        //                          fsina)
        //           << std::endl;

        return units::V2Position {
            horizontal_wall_length - measured_distance * fcosa,
            vertical_wall_length - measured_distance * fsina
        };
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

    bool hasAvailableReading() override {
        return !exit;
    }

    bool getVectorized() override {
        return true;
    }

    bool canProcessArray() override {
        return true;
    }
};
} // namespace vexmaps
