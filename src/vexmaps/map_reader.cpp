#include "vexmaps/mcl/map_reader.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include <cassert>
#include <fstream>
#include <iostream>

template<int theta_res, int x_res, int y_res>
void MapReader<theta_res, x_res, y_res>::read(std::string filename) {
    std::ifstream field_file(filename, std::ios::binary);
    field_file.read(reinterpret_cast<char*>(map), sizeof map);
    std::streamsize read_size = field_file.gcount();
    assert((read_size == x_res * y_res * theta_res) &&
           "Map size doesn't match!");

    map_is_read = true;
}

// works for x,y alredy in inches and theta in 2 * degrees
template<int theta_res, int x_res, int y_res>
float MapReader<theta_res, x_res, y_res>::query(float x, float y, float theta) {
    bool x_sgn = x < 0;
    bool y_sgn = y < 0;

    if (x_sgn && y_sgn) {
        theta = theta - 2 * M_PI;
    } else if (x_sgn) {
        theta = 2 * M_PI - theta;
    } else if (y_sgn) {
        theta = 4 * M_PI - theta;
    }

    if (theta < 0) theta += 4 * M_PI;

    int ix = units::abs(std::round(x));
    int iy = units::abs(std::round(y));
    int itheta = std::round(theta);

    // std::cout << "indexes are " << ix << " " << iy << " " << itheta <<
    // std::endl;

    // assert(ix < x_res);
    // assert(iy < y_res);
    // assert(itheta < theta_res);

    unsigned char query = map[itheta][ix][iy];
    // std::cout << "query " << query << std::endl;

    FLength query_distance = 2.5_Fm * static_cast<float>(query) / 254.0;

    // what should we do?
    if (query == 255) query_distance = -2.5_Fm;

    return query_distance.internal();
}

// assumes theta is between [0,2pi]
template<int theta_res, int x_res, int y_res>
FLength
MapReader<theta_res, x_res, y_res>::query(FLength x, FLength y, FAngle theta) {
    // map into correct quadrant
    bool x_sgn = x.internal() < 0;
    bool y_sgn = y.internal() < 0;

    if (x_sgn && y_sgn) {
        theta = theta - Frot / 2;
    } else if (x_sgn) {
        theta = Frot / 2 - theta;
    } else if (y_sgn) {
        theta = Frot - theta;
    }

    if (theta.internal() < 0) theta += Frot;

    x = units::abs(x);
    y = units::abs(y);

    int ix = std::round(x.convert(in));
    int iy = std::round(y.convert(in));
    int itheta = std::round(theta.convert(deg) * 2);

    // std::cout << "indexes are " << ix << " " << iy << " " << itheta <<
    // std::endl;

    assert(ix < x_res);
    assert(iy < y_res);
    assert(itheta < theta_res);

    unsigned char query = map[itheta][ix][iy];
    // std::cout << "query " << query << std::endl;

    FLength query_distance = 2.5_Fm * static_cast<double>(query) / 254;

    if (query == 255) query_distance = -1.0_Fm;

    return query_distance;
};

template<int theta_res, int x_res, int y_res>
bool MapReader<theta_res, x_res, y_res>::mapAvailable() {
    return map_is_read;
}

// explicitly instantiate default
template class MapReader<>;
