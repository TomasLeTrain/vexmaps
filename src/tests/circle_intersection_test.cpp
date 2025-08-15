#include "tests/circle_intersection_test.hpp"
#include "pros/rtos.h"
#include "vexmaps/mcl/circle_intersection.hpp"
#include <cmath>
#include <iostream>
#include <vector>

void testCircleIntersections() {
    // int n = (40000 / 16) * 16;
    int n = (100 / 16) * 16;

    std::vector<float> x(n);
    std::vector<float> y(n);
    std::vector<float> res(n);

    for(int i = 0; i < n;i++){
        // sets all vals to evaluate to 0
        x[i]  = 0;
        y[i]  = 2;
    }

    int offset = 0;

    // should be 2, instead returns big value since the approximation for sqrt(0) is a big value
    // effectively perfect intersections with the circle might be undefined behavior
    x[offset] = 0;
    y[offset] = 0;

    // should be 0
    x[offset+1] = 0;
    y[offset+1] = 2;

    // ray does not intersect, should be undefined
    x[offset+2] = -1;
    y[offset+2] = -1;

    // should also be undefined, since its inside the circle
    x[offset+3] = 1;
    y[offset+3] = 1;

    // should be 4
    x[offset+4] = -4;
    y[offset+4] = 2;

    float angle = 0;
    float vx = std::cos(angle);
    float vy = std::sin(angle);
    float ox = 2;
    float oy = 2;

    float r = 2;

    std::cout << "vx,vy: " << vx << " " << vy << std::endl;

    auto start_time = pros::c::micros();
    // vexmaps::circleIntersection(res.data(), x.data(), y.data(), n, vx, vy, ox, oy, r);
    vexmaps::circleCenterIntersection(res.data(), x.data(), y.data(), n, vx, vy, ox, oy, r);
    auto end_time = pros::c::micros();

    // running in the simulator gives 25.813 ms to run 40,000 points, meaning 1549.606 particles / ms
    std::cout << "time taken: " << end_time - start_time  << std::endl;

    for(auto e : res){
        std::cout << e << "\n";
    }
    std::cout << std::endl;
}
