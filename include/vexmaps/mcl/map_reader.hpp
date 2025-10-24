#pragma once

#include "units/Angle.hpp"
#include "units/units.hpp"

template<int theta_res = 721, int x_res = 71, int y_res = 71>
class MapReader {
    unsigned char map[theta_res][x_res][y_res];
    bool map_is_read;

  public:
    void read(std::string filename);

    int preprocess_angle(FAngle angle);

    // assumes theta is between [0,2pi]
    FLength query(FLength x, FLength y, FAngle theta);

    // works for (x, y) alredy in inches and theta in 2 * degrees
    float query(float x, float y, float theta);

    bool mapAvailable();
};
