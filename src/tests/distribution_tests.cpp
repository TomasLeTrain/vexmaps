#include "vexmaps/mcl/asm_functions.hpp"
#include "vexmaps/mcl/utils.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>
#include "tests/distribution_tests.hpp"

void testNormalDistPDF() {
    std::cout << "normal distribution test:" << std::endl;

	const int target_n = 1000;
	
    // makes n divisible by 24 and 16
    const int n = (target_n / 48) * 48;

    float x[n];
    float res[n];

    float start_value = -10;
    float value_delta = 0.1;

    x[0] = start_value;
    res[0] = 0;

    for (int i = 1; i < n; i++) {
        x[i] = x[i - 1] + value_delta;
        res[i] = 0;
    }

    float mean = 10, std_dev = 0.5, multiplier = 2;

    auto normalDist = [](double x,
                         double mean = 0,
                         double std_dev = 1,
                         double multiplier = 1) -> double {
        double xm = (x - mean) / std_dev;
        return multiplier *
               (exp(-0.5 * (xm * xm)) / (std_dev * sqrt(2 * M_PI)));
    };

    // run routine
    auto start_time = pros::c::micros();
    vexmaps::VNormalDistributionPDF(res, x, n, mean, std_dev, multiplier);
    auto end_time = pros::c::micros();

    std::cout << "time taken: " << end_time - start_time  << std::endl;

    float max_diff = 0.0;

    // check that x is intact:
    float check_val = start_value;

    for (int i = 0; i < n; i++) {
        assert(x[i] == check_val);
        check_val += value_delta;
    }

    for (int i = 0; i < n; i++) {
        float diff = res[i] - normalDist(x[i], mean, std_dev, multiplier);

        max_diff = std::max(max_diff, std::abs(diff));

        // std::cout << x[i] << ", " << diff << std::endl;
    }
    std::cout << "max diff: " << max_diff << " on range [" << start_value
              << ", " << (start_value + value_delta * (n-1)) << "]" << std::endl;
}

void testExpDistPDF() {
    std::cout << "exponential distribution test:" << std::endl;

	const int target_n = 1000;

    // makes n divisible by 24 and 16
    const int n = (target_n / 48) * 48;

    float x[n];
    float res[n];

    float start_value = 0;
    float value_delta = 0.05;

    x[0] = start_value;
    res[0] = 0;

    for (int i = 1; i < n; i++) {
        x[i] = x[i - 1] + value_delta;
        res[i] = 0;
    }

    float max_diff = 0.0;

    float lambda = 3;
    float exp_multiplier = 3;

    auto expDist =
      [](double x, double lambda = 1, double multiplier = 1) -> double {
        return lambda * exp(-x * lambda);
    };

    // run routine
    auto start_time = pros::c::micros();
    vexmaps::VNexpDistributionPDF(res, x, n, lambda, exp_multiplier);
    auto end_time = pros::c::micros();

    std::cout << "time taken: " << end_time - start_time  << std::endl;

    float check_val = start_value;

    for (int i = 0; i < n; i++) {
        assert(x[i] == check_val);
        check_val += value_delta;
    }

    for (int i = 0; i < n; i++) {
        float diff = res[i] - expDist(x[i], lambda, exp_multiplier);

        max_diff = std::max(max_diff, std::abs(diff));

        // std::cout << x[i] << ", " << diff << std::endl;
    }
    std::cout << "max diff: " << max_diff << " on range [" << start_value
              << ", " << (start_value + value_delta * (n-1)) << "]" << std::endl;

    std::cout << std::endl;
}


void testDistributions() {
	testNormalDistPDF();
	testExpDistPDF();
}

