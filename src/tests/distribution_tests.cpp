#include "vexmaps/mcl/circle_intersection.hpp"
#include <cmath>
#include <iostream>
#include <vector>

void testVNormalDistributionPDF() {
    // makes n divisible by 24 and 16
    const int n = (1000 / 48) * 48;

    float x[n];
    float res[n];

    for (int i = 0; i < n; i++) {
        x[i] = 0;
        res[i] = 0;
    }

    x[0] = 0.1;
    x[1] = 0.2;
    x[2] = 0.3;

    x[3] = -0.1;
    x[4] = -0.2;
    x[5] = -0.3;

    x[6] = 10;
    x[7] = -20;
    x[8] = 2;

    vexmaps::VNormalDistributionPDF(x, res, n, 0);

    auto normalDist = [](double x) -> double {
        return exp(-0.5 * (x * x)) / sqrt(2 * M_PI);
    };

    std::cout << "normal distribution test:" << std::endl;
    for (int i = 0; i < 30; i++) {
        std::cout << x[i] << " " << res[i] << " " << normalDist(x[i]) << " "
                  << res[i] - normalDist(x[i]) << " " << std::endl;
    }
    std::cout << std::endl;
}
