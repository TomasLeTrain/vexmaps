#pragma once

#include "pros/imu.h"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"
#include "units/Angle.hpp"
#include <cmath>
#include <cstdint>
#include <mutex>

namespace vexmaps {
class ScaledIMU : public pros::IMU {
  public:
    ScaledIMU(int port, double scalar = 1.0)
        : pros::IMU(port),
          m_port(port),
          m_scalar(scalar) {}

    ScaledIMU(const pros::IMU& other, double scalar = 1.0)
        : pros::IMU(other),
          m_port(other.get_port()),
          m_scalar(scalar) {}

    int32_t reset(bool blocking = false) {
        std::lock_guard lock(m_mutex);

        m_offset = 0;
        return pros::IMU::reset(blocking);
    }

    virtual double get_rotation() const {
        std::lock_guard lock(m_mutex);

        double raw = pros::c::imu_get_rotation(m_port);
        if(raw == INFINITY) return INFINITY;
        return raw * m_scalar + m_offset;
    }

    virtual int set_rotation(double new_rotation) {
        std::lock_guard lock(m_mutex);

        double curr_raw = this->get_rotation();
        if (curr_raw == INFINITY) return INT32_MAX;

        m_offset += new_rotation - curr_raw;
        return 0;
    }

  private:
    const double m_scalar;
    int m_port;

    mutable pros::Mutex m_mutex;

    double m_offset = 0;
};
} // namespace vexmaps
