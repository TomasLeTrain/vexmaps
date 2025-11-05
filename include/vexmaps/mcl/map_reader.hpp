#pragma once

#include "pros/rtos.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include <memory>

template<int theta_res = 721, int x_res = 101, int y_res = 101>
class MapReader {
  private:
    pros::Mutex m_mutex;

    std::unique_ptr<unsigned char[][x_res][y_res]> map;

    bool map_is_read;

    // works for (x, y) alredy in inches and theta in 2 * degrees
    float query_internal(float x, float y, float theta);

  public:
    void read(std::string filename);

    void read_compressed(std::string filename);

    int preprocess_angle(FAngle angle);

    // assumes theta is between [0,2pi]
    FLength query(FLength x, FLength y, FAngle theta);

    bool mapAvailable();
};
