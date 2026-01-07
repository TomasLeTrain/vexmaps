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
#include <utility>

namespace vexmaps {

class DistanceSensorModel : public Sensor {

    pros::Distance* distance_sensor;
    units::Pose offsets;
    double m_distance_scale_factor;
    std::string name;

    DistanceSensorConfig config;

    // optional
    MapReader<>* map_reader;

    units::FPose rotated_offsets = { 0_m, 0_m, 0_stDeg };
    Length measured_distance = 0_m;

    // determines whether or not readings from this sensor are used - false when
    // there are no measurements
    bool exit = false;

    // used for certain procedures where using only absolutely new measurements
    // is not super important (i.e. distance sensor resets)
    bool exit_without_new_measurement = false;

    // when false, sensor is not used regardless of measurements
    bool enabled = true;

    // 2.5 meters is likely the most a distance sensor will ever be able to
    // sense
    const double randomUniformProbability = 1 / (2.54);
    double randomFactor;

    // precomputed values

    // values used per particle evaluation - should all be floats
    float Vhor_wall_coeff;
    float Vver_wall_coeff;

    float fsina, fcosa;
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

    bool new_measurement = false;

    std::optional<std::pair<int32_t, Time>> last_measurement = std::nullopt;

  public:
    DistanceSensorModel(pros::Distance* distance_sensor,
                        units::Pose offsets,
                        double distance_scale_factor,
                        std::string name,
                        DistanceSensorConfig config,
                        MapReader<>* map_reader = nullptr)
        : distance_sensor(distance_sensor),
          offsets(offsets),
          m_distance_scale_factor(distance_scale_factor),
          name(name),
          config(config),
          map_reader(map_reader) {
        randomFactor = config.randomCoeff * randomUniformProbability;
    }

    // returns nullopt if doesn't hit
    // otherwise returns distance to object
    std::optional<FLength>
    circleIntersection(units::V2Position position,
                       Angle angle,
                       units::V2FPosition circle_position,
                       FLength radius,
                       FLength actual_radius,
                       FLength wall_distance,
                       bool use_big_distance = false) {
        auto u = circle_position - position;

        // inside circle, activate smaller
        if (u.magnitude() < radius) {
            return u.magnitude();
        }

        auto unit_v = units::Vector2D<Number>::unitVector(angle);

        Length cross = units::abs(u.cross(unit_v));
        Length dot = u * unit_v;
        auto diff = units::square(radius) - units::square(cross);

        // circle is behind position, does not intersect
        if (dot.internal() < 0) return std::nullopt;

        // does not intersect circle
        if (diff.internal() < 0) return std::nullopt;

        // shortest intersection to circle
        auto circle_dist = dot - units::sqrt(diff);

        // circle not really intersected since wall distance was smaller
        if (wall_distance < circle_dist) return std::nullopt;

        auto actual_diff =
          units::max(units::square(actual_radius) - units::square(cross),
                     FArea(0));

        // more accurate distance to obstacle
        auto actual_dist = dot - units::sqrt(actual_diff);

        // distance is too big so it's unlikely object is close enough where it
        // matters
        // if (units::abs(wall_distance - actual_dist) > 10_in &&
        //     use_big_distance) {
        //     return std::nullopt;
        // }

        // object might be measured, returns expected distance to object
        return actual_dist;
    }

    void update(Angle angle, std::optional<units::FPose> pose) override {
        // first check if the distance sensor is available, and if its not then
        // fail non-destructively while still alerting user
        if (distance_sensor == nullptr || !distance_sensor->is_installed()) {
            // not available, just set exit to true
            exit = true;
            // printf(
            //   "ONE OF THE DISTANCE SENSORS ARE NOT CONNECTED CORRECTLY!!\n");
            return;
        }

        const int32_t measured_mm = distance_sensor->get();
        auto installed = distance_sensor->is_installed();

        Time now = from_msec(pros::millis());

        constexpr Time DIST_POLLING_RATE = 1.0 / 30_Hz;

        new_measurement = !installed ||
                          // either we don't have last measurement
                          !last_measurement ||
                          // or measurements differ
                          (last_measurement->first != measured_mm ||
                           // or we are guaranteed to have new measurements
                           now - last_measurement->second > DIST_POLLING_RATE);

        // updated only once to keep last_measurement_time accurate
        if (new_measurement) last_measurement = { measured_mm, now };

        measured_distance = from_mm(measured_mm);

        // only applies scale factor if distance sensor uses alternate algo for
        // determining distance (smaller than 200_mm probably does not need a
        // scaling factor)
        if (measured_distance > 200_mm) {
            measured_distance *= m_distance_scale_factor;
        }

        f_measured_distance = measured_distance.internal();

        exit_without_new_measurement =
          // not connected
          !installed ||
          // distance sensor doesn't measure anything
          measured_mm == 9999
          // or disabled
          || (!enabled);

        exit = exit_without_new_measurement
               // or didn't receieve new data
               || !new_measurement;

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
        Length original_horizontal_wall_length =
          global_hor_wall_length * cos_sign;
        Length original_vertical_wall_length =
          global_ver_wall_length * sin_sign;

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

        float expVal =
          expDistribution(measured_distance.internal(), config.exp_l);

        // should actually be applied per particle but would be really
        // computationally expensive
        // only computing it for measured distance should still work fine
        // float expNormFactor =
        //   expNormalizationFactor<DistanceSensorConfig::exp_l>(
        //     measured_distance.internal());

        // constant in relation to all particles
        // (only depends on measured distance)
        expFactor = expVal * config.expCoeff + randomFactor;

        bool make_shorter = false;

        if (pose) {
            FLength pose_distance_difference =
              getDistanceDifference(pose->x, pose->y);

            FLength expected_distance =
              pose_distance_difference + measured_distance;

            if (config.detect_obstacles) {
                FLength matchloader_x = 70_in;
                FLength matchloader_y = 46.7_in;
                FLength match_big_radius = 10_in;
                FLength matchloader_actual_radius = 2_in;

                FLength corner_x = 70_in;
                FLength corner_y = 70_in;
                FLength corner_radius = 10_in;

                // check if it would have intersection with a matchloader
                for (int i = -1; i <= 1; i += 2) {
                    for (int j = -1; j <= 1; j += 2) {
                        // check matchloader
                        make_shorter |=
                          circleIntersection(
                            *pose,
                            rotated_offsets.orientation,
                            { matchloader_x * i, matchloader_y * j },
                            match_big_radius,
                            matchloader_actual_radius,
                            expected_distance)
                            .has_value();

                        // check corner
                        make_shorter |=
                          circleIntersection(*pose,
                                             rotated_offsets.orientation,
                                             { corner_x * i, corner_y * j },
                                             corner_radius,
                                             corner_radius,
                                             expected_distance)
                            .has_value();
                    }
                }
            }

            FLength max_difference = config.maxDistanceDifference;
            FLength max_out_difference = config.maxOutDistanceDifference;

            if (make_shorter) {
                // make difference shorter if possibly noisy
                max_difference = 2.6_in;
                max_out_difference = 2.6_in;
            }

            // measured is smaller than expected
            if (units::sgn(pose_distance_difference) > 0.0 &&
                units::abs(pose_distance_difference) > max_difference) {
                exit = true;
            } else if
              // measured is greater than expected
              //
              // technically should never be a wrong measurement, however at
              // weird angles a measurement might be greater than it
              // actually should be. we can ignore measurements like these
              (units::sgn(pose_distance_difference) < 0.0 &&
               units::abs(pose_distance_difference) > max_out_difference) {
                exit = true;
            }
        }

        if (config.logging) {
            // name:distance,confidence,std,exit,obj_size
            std::cout << name << ":" << measured_distance.convert(in) << ","
                      << distance_sensor->get_confidence() << ","
                      << (exit_without_new_measurement ? 1.0 : 0.0) << ","
                      << (exit ? "true" : "false")
                      << ","
                      // << distance_sensor->get_object_size() << "\n";
                      << int(make_shorter) << "\n";
        }
    }

    // assumes that its only getting called if exit is false
    // this assumption saves some conditionals improving performance
    float evaluate(FLength x, FLength y) override {
        const FLength difference = units::min(hor_wall_coeff + x * x_coeff,
                                              ver_wall_coeff + y * y_coeff);

        float normal_dist =
          NormalDistributionApproximation(difference.internal(),
                                          config.std_deviation,
                                          config.normalCoeff);

        if (difference.internal() >= 0) {
            return normal_dist + expFactor;
        } else {
            return normal_dist + randomFactor;
        }
    }

    // returns calculated difference from measurement and expectation
    //
    // assumes that its only getting called if exit is false
    // this assumption saves some conditionals improving performance
    FLength getDistanceDifference(FLength x, FLength y) {
        const FLength difference = units::min(hor_wall_coeff + x * x_coeff,
                                              ver_wall_coeff + y * y_coeff);
        return difference;
    }

    // assumes that its only getting called if exit is false
    // this assumption saves some conditionals improving performance
    // float32x4_t Vevaluate(float32x4_t x, float32x4_t y) override {
    //     // clang-format off
    //     //
    //     // expected_distance =
    //     //   min( (horizontal_wall_length - point.x) * secant,
    //     //        (vertical_wall_length   - point.y) * cosecant );
    //     //
    //     // hor_wall = (horizontal_wall_length - x) * this->secant
    //     // hor_wall = (horizontal_wall_length * this->secant) + x *
    //     (-this->secant)
    //     // hor_wall = HC [all precomputed]                    + x *
    //     x_coeff [precomputed]
    //     //
    //     // -- direct difference formulation --
    //     // difference = expected_distance - measured_distance
    //     // difference = min(HC,VC) - measured_distance  ==>  min(HC -
    //     measured_distance, VC - measured_distance)
    //     //
    //     // HC                            - measured_distance
    //     // (hor_wall_coeff - x * secant) - measured_distance
    //     // (hor_wall_coeff - measured_distance) + x * x_coeff
    //     // hor_wall_coeff [new coeff]           + x * x_coeff
    //     //
    //     //
    //     // in the end this results in:
    //     // hor_wall_coeff = horizontal_wall_length * secant   -
    //     measured_distance
    //     // ver_wall_coeff = vertical_wall_length   * cosecant -
    //     measured_distance
    //     // x_coeff = -secant
    //     // y_coeff = -cosecant
    //     //
    //     // clang-format on
    //
    //     // HC = hor_wall_coeff + point.x * (-secant)
    //     // VC = ver_wall_coeff + point.y * (-cosecant)
    //     float32x4_t HC = vmlaq_n_f32(vdupq_n_f32(Vhor_wall_coeff), x,
    //     x_coeff); float32x4_t VC =
    //     vmlaq_n_f32(vdupq_n_f32(Vver_wall_coeff), y, y_coeff);
    //
    //     // difference = min(HC,VC)
    //     float32x4_t difference = vminq_f32(HC, VC);
    //
    //     // each number is all UINT_MAX if (expected - measured) is >= 0,
    //     else
    //     // its 0
    //     uint32x4_t modMask = vcgeq_f32(difference, vdupq_n_f32(0.0));
    //
    //     // constantFactor = measured <= expected ? expFactor :
    //     randomFactor float32x4_t VMaskedConstantFactor =
    //       vbslq_f32(modMask, vdupq_n_f32(expFactor),
    //       vdupq_n_f32(randomFactor));
    //
    //     float32x4_t normal_dist =
    //       VNormalDistributionApproximation<DistanceSensorConfig::std_deviation,
    //                                        DistanceSensorConfig::normalCoeff>(
    //         difference);
    //
    //     return vaddq_f32(normal_dist, VMaskedConstantFactor);
    // }

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
          config.std_deviation,
          config.normalCoeff);

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
                               config.map_deviation,
                               config.mapCoeff);
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
        if (exit_without_new_measurement) {
            return std::nullopt;
        }

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

    void setConfig(DistanceSensorConfig new_config) {
        config = new_config;
    }

    DistanceSensorConfig getConfig() {
        return config;
    }

    void setMaxDistanceDifference(FLength maxDistanceDifference) {
        config.maxDistanceDifference = maxDistanceDifference;
    }

    FLength getMaxDistanceDifference() {
        return config.maxDistanceDifference;
    }

    bool getEnabled() override {
        return enabled;
    }

    bool hasAvailableReading() override {
        return !exit;
    }

    bool getVectorized() override {
        // return true;
        return false;
    }

    bool canProcessArray() override {
        return true;
    }

    std::pair<bool, bool> getKnownCoords(units::Pose target_pose) override {
        const FLength hor_difference = hor_wall_coeff + target_pose.x * x_coeff;
        const FLength ver_difference = ver_wall_coeff + target_pose.y * y_coeff;

        // hitting horizontal wall, we know x coordinate
        if (hor_difference < ver_difference) {
            return { true, false };
        } else {
            return { false, true };
        }
    }
};
} // namespace vexmaps
