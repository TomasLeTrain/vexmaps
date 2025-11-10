#include "vexmaps/mcl/map_reader.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include "zlib/zlib.h"
#include <cassert>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <sys/types.h>
#include <vector>

template<int theta_res, int x_res, int y_res>
void MapReader<theta_res, x_res, y_res>::read(std::string filename) {
    std::ifstream map_stream(filename, std::ios::binary);

    if (!map_stream) {
        std::cerr << "Map reading was unsucessful!" << std::endl;
        return;
    }

    // only allocate if we haven't already
    if (map.get() == nullptr)
        map = std::make_unique<unsigned char[][x_res][y_res]>(theta_res);

    map_stream.read(reinterpret_cast<char*>(map.get()),
                    x_res * y_res * theta_res);

    if (!map_stream) {
        std::cerr << "Map reading was unsucessful!" << std::endl;
        return;
    }

    std::streamsize read_size = map_stream.gcount();
    if (read_size != x_res * y_res * theta_res) {
        std::cerr << "Map size doesn't match: got " << read_size
                  << " and expected " << x_res * y_res * theta_res << std::endl;
        return;
    }

    {
        std::lock_guard lock(m_mutex);
        map_is_read = true;
    }
}

template<int theta_res, int x_res, int y_res>
void MapReader<theta_res, x_res, y_res>::read_compressed(std::string filename) {
    std::ifstream map_stream(filename, std::ios::binary | std::ios::ate);
    std::cout << "reading map!\n";

    // will equal -1 if file doesn't exit
    auto file_size = map_stream.tellg();
    std::cout << "file size is " << file_size << std::endl;

    // exit before vector is created
    if (!map_stream || file_size == -1) {
        std::cerr << "Map reading was unsucessful!\n";
        return;
    }

    // ensure vector is not stored on the stack
    std::unique_ptr<std::vector<char>> compressed_data =
      std::make_unique<std::vector<char>>(file_size);

    map_stream.seekg(0);

    std::cout << "reading compressed data" << std::endl;
    map_stream.read(compressed_data->data(), file_size);

    if (!map_stream) {
        std::cerr << "Map reading was unsucessful!\n";
        return;
    }

    std::cout << "allocating map" << std::endl;
    // allocate memory

    // only allocate if we haven't already
    if (map.get() == nullptr)
        map = std::make_unique<unsigned char[][x_res][y_res]>(theta_res);

    std::cout << "decompressing" << std::endl;

    // Allocate buffer for decompression
    uLong decompressedLen = theta_res * x_res * y_res;

    int result = uncompress(reinterpret_cast<Bytef*>(map.get()),
                            &decompressedLen,
                            reinterpret_cast<Bytef*>(compressed_data->data()),
                            compressed_data->size());

    if (result == Z_OK) {
        std::cout << "decompression worked! compressed/uncompressed: "
                  << compressed_data->size() << " " << decompressedLen
                  << std::endl;
    } else {
        std::cerr << std::format("there was an error with code: {}!", result)
                  << std::endl;
        return;
    }

    if (decompressedLen != theta_res * x_res * y_res) {
        std::cerr << std::format("sizes don't match! expected/got: {},{}",
                                 theta_res * x_res * y_res,
                                 decompressedLen)
                  << std::endl;
        return;
    }

    std::cout << "everything went good" << std::endl;

    {
        std::lock_guard lock(m_mutex);
        map_is_read = true;
    }
}

// works for x, y alredy in inches * factor and theta in degrees * factor
template<int theta_res, int x_res, int y_res>
float MapReader<theta_res, x_res, y_res>::query_internal(float x,
                                                         float y,
                                                         float theta) {
    // std::cout << "called with " << x << " " << y << " " << theta <<
    // std::endl;

    constexpr float theta_factor = (theta_res - 1) / 360.0;
    constexpr float M_PI_scaled = 180 * theta_factor;

    bool x_sgn = x < 0;
    bool y_sgn = y < 0;

    if (x_sgn && y_sgn) {
        theta = theta - M_PI_scaled;
    } else if (x_sgn) {
        theta = M_PI_scaled - theta;
    } else if (y_sgn) {
        theta = 2 * M_PI_scaled - theta;
    }

    if (theta < 0) theta += 2 * M_PI_scaled;

    int ix = std::round(std::abs(x));
    int iy = std::round(std::abs(y));
    int itheta = std::round(theta);

    if (ix >= x_res) {
        std::cerr << "x out of range! -> " << ix << "," << iy << "," <<
        itheta
                  << std::endl;
        return (-2.5_Fm).internal();
    }
    if (iy >= y_res) {
        std::cerr << "y out of range! -> " << ix << "," << iy << "," <<
        itheta
                  << std::endl;
        return (-2.5_Fm).internal();
    }
    if (itheta >= theta_res) {
        std::cerr << "theta out of range! ->" << ix << "," << iy << ","
                  << itheta << std::endl;
        return (-2.5_Fm).internal();
    }

    // std::cout << "query at " << ix << " " << iy << " " << itheta <<
    // std::endl;

    unsigned char query = map[itheta][ix][iy];

    constexpr float query_factor = (2.5_m / 254.0).internal();

    float query_distance = query_factor * static_cast<float>(query);

    // what should we do?
    if (query == 255) query_distance = (-2.5_Fm).internal();

    return query_distance;
}

// assumes theta is between [0,2pi]
template<int theta_res, int x_res, int y_res>
FLength
MapReader<theta_res, x_res, y_res>::query(FLength x, FLength y, FAngle theta) {
    const FCurvature x_factor = (x_res - 1) / 70.0_in;
    const FCurvature y_factor = (y_res - 1) / 70.0_in;
    const Divided<Number, Angle> theta_factor = (theta_res - 1) / 360.0_stDeg;

    return FLength(
      query_internal(x * x_factor, y * y_factor, theta * theta_factor));
};

template<int theta_res, int x_res, int y_res>
bool MapReader<theta_res, x_res, y_res>::mapAvailable() {
    std::lock_guard lock(m_mutex);
    return map_is_read;
}

// explicitly instantiate with default template params
template class MapReader<>;
