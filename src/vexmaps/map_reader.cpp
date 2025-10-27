#include "vexmaps/mcl/map_reader.hpp"
#include "lz4/lz4.h"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include <cassert>
#include <fstream>
#include <iostream>
#include <vector>

template<int theta_res, int x_res, int y_res>
void MapReader<theta_res, x_res, y_res>::read(std::string filename) {
    std::ifstream map_stream(filename, std::ios::binary);

    // allocate memory
    map = std::make_unique<unsigned char[][x_res][y_res]>(theta_res);

    map_stream.read(reinterpret_cast<char*>(map.get()),
                    x_res * y_res * theta_res);

    if (!map_stream) {
        std::cerr << "Map reading was unsucessful!" << std::endl;
        return;
    }

    std::streamsize read_size = map_stream.gcount();
    if (read_size != x_res * y_res * theta_res) {
        std::cerr << "Map size doesn't match!" << std::endl;
        return;
    }

    map_is_read = true;
}

template<int theta_res, int x_res, int y_res>
void MapReader<theta_res, x_res, y_res>::read_compressed(std::string filename) {
    std::ifstream map_stream(filename, std::ios::binary | std::ios::ate);

    auto file_size = map_stream.tellg();
    std::vector<char> compressed_data(
      file_size); // construct string to stream size
    map_stream.seekg(0);
    map_stream.read(compressed_data.data(), file_size);

    if (!map_stream) {
        std::cerr << "Map reading was unsucessful!\n";
        return;
    }

    // allocate memory
    map = std::make_unique<unsigned char[][x_res][y_res]>(theta_res);

    // decompress data
    int decompressed_size =
      LZ4_decompress_safe(reinterpret_cast<char*>(compressed_data.data()),
                          reinterpret_cast<char*>(map.get()),
                          compressed_data.size(),
                          x_res * y_res * theta_res);
    if (decompressed_size == 0) {
        std::cerr << "Map decompression failed" << std::endl;
    }

    if (decompressed_size != x_res * y_res * theta_res) {
        std::cerr << "Map size doesn't match: got " << decompressed_size
                  << " and expected " << x_res * y_res * theta_res << std::endl;
        return;
    }

    map_is_read = true;
}

// works for x, y alredy in inches * factor and theta in degrees * factor
template<int theta_res, int x_res, int y_res>
float MapReader<theta_res, x_res, y_res>::query(float x, float y, float theta) {
    // std::cout << "called with " << x << " " << y << " " << theta <<
    // std::endl;

    constexpr float theta_factor = (theta_res - 1) / 360.0;
    // pi in deg
    constexpr float M_PI_deg = (rot / 2).convert(deg);

    bool x_sgn = x < 0;
    bool y_sgn = y < 0;

    // theta is doubled, so constants have to as well
    if (x_sgn && y_sgn) {
        theta = theta - theta_factor * M_PI_deg;
    } else if (x_sgn) {
        theta = theta_factor * M_PI_deg - theta;
    } else if (y_sgn) {
        theta = 2 * theta_factor * M_PI_deg - theta;
    }

    // std::cout << "theta before " << theta << std::endl;
    if (theta < 0) theta += 2 * theta_factor * M_PI_deg;
    // std::cout << "theta after " << theta << std::endl;

    int ix = std::round(std::abs(x));
    int iy = std::round(std::abs(y));
    int itheta = std::round(theta);

    // if (ix >= x_res) {
    //     std::cerr << "x out of range! -> " << ix << "," << iy << "," <<
    //     itheta
    //               << std::endl;
    //     return (-2.5_Fm).internal();
    // }
    // if (iy >= y_res) {
    //     std::cerr << "y out of range! -> " << ix << "," << iy << "," <<
    //     itheta
    //               << std::endl;
    //     return (-2.5_Fm).internal();
    // }
    // if (itheta >= theta_res) {
    //     std::cerr << "theta out of range! ->" << ix << "," << iy << ","
    //               << itheta << std::endl;
    //     return (-2.5_Fm).internal();
    // }

    // std::cout << "query at " << ix << " " << iy << " " << itheta <<
    // std::endl;

    unsigned char query = map[itheta][ix][iy];

    FLength query_distance = 2.5_Fm * static_cast<float>(query) / 254.0;

    // what should we do?
    if (query == 255) query_distance = -2.5_Fm;

    return query_distance.internal();
}

// assumes theta is between [0,2pi]
template<int theta_res, int x_res, int y_res>
FLength
MapReader<theta_res, x_res, y_res>::query(FLength x, FLength y, FAngle theta) {
    const FCurvature x_factor = (x_res - 1) / 70.0_in;
    const FCurvature y_factor = (y_res - 1) / 70.0_in;
    const Divided<Number, Angle> theta_factor = (theta_res - 1) / 360.0_stDeg;

    return FLength(query(x * x_factor, y * y_factor, theta * theta_factor));
};

template<int theta_res, int x_res, int y_res>
bool MapReader<theta_res, x_res, y_res>::mapAvailable() {
    return map_is_read;
}

// explicitly instantiate default
template class MapReader<>;
