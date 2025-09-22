#pragma once

#include "pros/distance.hpp"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include "vexmaps/mcl/circle_intersection.hpp"
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

    pros::Distance* distance_sensor;

    Length measured_distance = 0_m;

    units::Pose offsets;

    units::Pose rotated_offsets = { 0_m, 0_m, 0_stDeg };

    std::string name;

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

    float f_measured_distance = 0;

    float x_coeff;
    float y_coeff;
    FLength hor_wall_coeff;
    FLength ver_wall_coeff;
    float expFactor;

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
        rotated_offsets = rotatePose(offsets, angle);

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
        Length horizontal_wall_length = wall_length * cos_sign;
        Length vertical_wall_length = wall_length * sin_sign;

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

    bool hasAvailableReading() override {
        return !exit;
    }

    bool getVectorized() override {
        return true;
    }

    inline float evaluate(const units::V2FPosition& point) override {
        return evaluate(point.x, point.y);
    }

    // assumes that its only getting called if exit is false
    // this assumption saves some conditionals improving performance
    inline float evaluate(FLength x, FLength y) override {
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

    inline void VdistanceList(float* res, float* x, float* y, uint32_t len) {
        // do some processing here
    }

    // calculates the distance sensor pdf for each (x,y) pair
    // requires various lists to save temporary results
    inline void VevaluateList(float* x,
                              float* y,
                              float* curr_weights,
                              float* tmp_weights,
                              float* tmp_weights2,
                              uint32_t len) {
        // likley gets optimized to memset
        for (int i = 0; i < len; i++) {
            curr_weights[i] = 0;
        }

        // build result on the curr_weights list
        // first do the distance calculations
        VdistanceList(tmp_weights, x, y);

        // here tmp_weights = distance to wall
        VNormalDistributionPDF<DistanceSensorConfig::std_deviation,
                               DistanceSensorConfig::normalCoeff>(tmp_weights,
                                                                  curr_weights,
                                                                  len,
																  f_measured_distance);
        // also apply the dist / random factors based on this distance

		float matchloader1x = 0;
		float matchloader1y = 0;

		float matchloader2x = 0;
		float matchloader2y = 0;

		float matchloader_radius = 0;

        // now calculate intersections with matchloaders - overrides whatever
        // was in tmp_weights
		circleIntersection(tmp_weights,
                           x,
                           y,
                           len,
                           this->fcosa,
                           this->fsina,
                           matchloader1x,
                           matchloader1y,
                           matchloader_radius);

        // apply the normal distribution for this matchloader
        VNormalDistributionPDF<DistanceSensorConfig::std_deviation,
                               DistanceSensorConfig::normalCoeff>(tmp_weights,
                                                                  curr_weights,
                                                                  len,
																  f_measured_distance);
        // now do the same for the other matchloader
        circleIntersection(tmp_weights,
                           x,
                           y,
                           len,
                           this->fcosa,
                           this->fsina,
                           matchloader2x,
                           matchloader2y,
                           matchloader_radius);

        // apply the normal distribution for this matchloader
        VNormalDistributionPDF<DistanceSensorConfig::std_deviation,
                               DistanceSensorConfig::normalCoeff>(tmp_weights,
                                                                  curr_weights,
                                                                  len,
																  f_measured_distance);

        // check top center goal - modeled as a line segment
        raySegmentDistance(tmp_weights,
                           x,
                           y,
                           len,
                           this->fcosa,
                           this->fsina,
                           0,  // x1
                           0,  // y1
                           0,  // x2
                           0); // y2
        // apply the normal distribution for this goal
        VNormalDistributionPDF<DistanceSensorConfig::std_deviation,
                               DistanceSensorConfig::normalCoeff>(tmp_weights,
                                                                  curr_weights,
                                                                  len,
																  f_measured_distance);
        // check bottom center goal - modeled as a line segment
        raySegmentDistance(tmp_weights,
                           x,
                           y,
                           len,
                           this->fcosa,
                           this->fsina,
                           0,  // x1
                           0,  // y1
                           0,  // x2
                           0); // y2
        
        // apply the normal distribution for this goal
        VNormalDistributionPDF<DistanceSensorConfig::std_deviation,
                               DistanceSensorConfig::normalCoeff>(tmp_weights,
                                                                  curr_weights,
                                                                  len,
																  f_measured_distance);
        // check -y top long goals - use two circles to model this
        // checks left
        circleIntersection(tmp_weights,
                           x,
                           y,
                           len,
                           this->fcosa,
                           this->fsina,
                           0,
                           0,
                           2);
        // checks right
        circleIntersection(tmp_weights2,
                           x,
                           y,
                           len,
                           this->fcosa,
                           this->fsina,
                           0,
                           0,
                           2);

        // merges both - takes the minimum of both
        for (int i = 0; i < len; i++) {
            tmp_weights[i] = std::min(tmp_weights[i], tmp_weights2[i]);
        }

        // apply the normal distribution for this goal
        VNormalDistributionPDF<DistanceSensorConfig::std_deviation,
                               DistanceSensorConfig::normalCoeff>(tmp_weights,
                                                                  curr_weights,
                                                                  len,
																  f_measured_distance);

        // check y+ long goal - use two circles to model this
        // checks left
        circleIntersection(tmp_weights,
                           x,
                           y,
                           len,
                           this->fcosa,
                           this->fsina,
                           0,
                           0,
                           2);
        // checks right
        circleIntersection(tmp_weights2,
                           x,
                           y,
                           len,
                           this->fcosa,
                           this->fsina,
                           0,
                           0,
                           2);

        // merges both - takes the minimum of both
        for (int i = 0; i < len; i++) {
            tmp_weights[i] = std::min(tmp_weights[i], tmp_weights2[i]);
        }

        // apply the normal distribution for this goal
        VNormalDistributionPDF<DistanceSensorConfig::std_deviation,
                               DistanceSensorConfig::normalCoeff>(tmp_weights,
                                                                  curr_weights,
                                                                  len,
																  f_measured_distance);
    }

    std::optional<units::V2FPosition> getExpected() override {
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
