#pragma once

#include "pros/error.h"
#include "pros/imu.h"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"
#include "units/Angle.hpp"
#include <cmath>
#include <cstdint>
#include <mutex>

namespace vexmaps {
class MultipleImu : public pros::IMU {
  protected:
    std::vector<pros::IMU*> m_imus;
    size_t m_available_imu = 0;

  public:
    // returns pointer to first valid imu from list
    virtual std::optional<pros::IMU*> getAvailableImu() {
        size_t curr_imu_ind = m_available_imu;

        for (; curr_imu_ind < m_imus.size(); curr_imu_ind++) {
            pros::IMU* current_imu = m_imus[curr_imu_ind];

            if (!current_imu->is_installed()) {
                // current is invalid
                continue;
            }
            // calibrated and heading is infinite meaning imu is bad
            if (!current_imu->is_calibrating() &&
                !std::isfinite(current_imu->get_heading())) {
                continue;
            }

            // current is valid, break
            break;
        }

        m_available_imu = curr_imu_ind;

        if (m_available_imu < m_imus.size()) {
            // current is valid
            return m_imus[m_available_imu];
        } else {
            // no good imus, return nullopt
            return std::nullopt;
        }
    }

    MultipleImu(std::vector<pros::IMU*> imus)
        : pros::IMU(PROS_ERR_BYTE),
          m_imus(imus) {
        // remove imus with nullptr references
        m_imus.erase(std::remove_if(m_imus.begin(),
                                    m_imus.end(),
                                    [](pros::Imu* curr_imu) {
                                        return curr_imu == nullptr;
                                    }),
                     m_imus.end());
    }

    int32_t reset(bool blocking = false) {
        std::lock_guard lock(m_mutex);

        bool all_good = false;

        for (pros::IMU* imu : m_imus) {
            int curr = imu->reset(false);
            // only have error if its plugged in
            if (curr == PROS_ERR && imu->is_installed()) all_good = false;
        }

        uint32_t start_time = pros::millis();

        if (blocking) {
            // waits for the first available imu to finish
            while (pros::millis() - start_time < 3000 &&
                   getAvailableImu().has_value() &&
                   getAvailableImu().value()->is_calibrating()) {
                pros::delay(10);
            }
        }

        return all_good ? 1 : PROS_ERR;
    }

    virtual double get_rotation() {
        std::lock_guard lock(m_mutex);

        auto curr_imu = getAvailableImu();

        // no good imu
        if (!curr_imu) return INFINITY;

        return curr_imu.value()->get_rotation();
    }

    virtual int set_rotation(double new_rotation) {
        std::lock_guard lock(m_mutex);

        double curr_raw = this->get_rotation();
        // no good imus
        if (curr_raw == INFINITY) return INT32_MAX;

        // at least one good imu
        for (pros::IMU* imu : m_imus) {
            imu->set_rotation(new_rotation);
        }

        return 0;
    }

    virtual std::int32_t set_data_rate(std::uint32_t rate) const {
        std::lock_guard lock(m_mutex);

        for (pros::IMU* imu : m_imus) {
            imu->set_data_rate(rate);
        }

        return 0;
    }

    virtual double get_heading() {
        std::lock_guard lock(m_mutex);

        auto curr_imu = getAvailableImu();

        // no good imu
        if (!curr_imu) return INFINITY;

        return curr_imu.value()->get_heading();
    }

    virtual pros::quaternion_s_t get_quaternion() {
        std::lock_guard lock(m_mutex);

        auto curr_imu = getAvailableImu();

        // no good imu
        if (!curr_imu)
            return { .x = PROS_ERR_F,
                     .y = PROS_ERR_F,
                     .z = PROS_ERR_F,
                     .w = PROS_ERR_F };

        return curr_imu.value()->get_quaternion();
    }

    virtual pros::euler_s_t get_euler() {
        std::lock_guard lock(m_mutex);

        auto curr_imu = getAvailableImu();

        // no good imu
        if (!curr_imu)
            return { .pitch = PROS_ERR_F,
                     .roll = PROS_ERR_F,
                     .yaw = PROS_ERR_F };

        return curr_imu.value()->get_euler();
    }

    virtual double get_pitch() {
        return get_euler().pitch;
    }

    virtual double get_roll() {
        return get_euler().roll;
    }

    virtual double get_yaw() {
        return get_euler().yaw;
    }

    virtual pros::imu_gyro_s_t get_gyro_rate() {
        std::lock_guard lock(m_mutex);

        auto curr_imu = getAvailableImu();

        // no good imu
        if (!curr_imu)
            return { .x = PROS_ERR_F, .y = PROS_ERR_F, .z = PROS_ERR_F };

        return curr_imu.value()->get_gyro_rate();
    }

    virtual pros::imu_accel_s_t get_accel() {
        std::lock_guard lock(m_mutex);

        auto curr_imu = getAvailableImu();

        // no good imu
        if (!curr_imu)
            return { .x = PROS_ERR_F, .y = PROS_ERR_F, .z = PROS_ERR_F };

        return curr_imu.value()->get_accel();
    }

    virtual pros::ImuStatus get_status() {
        std::lock_guard lock(m_mutex);

        auto curr_imu = getAvailableImu();

        // no good imu
        if (!curr_imu)
            return static_cast<pros::ImuStatus>(pros::E_IMU_STATUS_ERROR);

        return curr_imu.value()->get_status();
    }

    virtual bool is_calibrating() {
        pros::imu_status_e status =
          static_cast<pros::imu_status_e>(this->get_status());

        if (status == pros::E_IMU_STATUS_ERROR) {
            return false;
        }
        return status & pros::E_IMU_STATUS_CALIBRATING;
    }

    virtual std::int32_t tare_heading() {
        std::lock_guard lock(m_mutex);

        auto curr_imu = getAvailableImu();

        // no good imu
        if (!curr_imu) return PROS_ERR;

        return curr_imu.value()->tare_heading();
    }

    virtual std::int32_t tare_rotation() {
        std::lock_guard lock(m_mutex);

        auto curr_imu = getAvailableImu();

        // no good imu
        if (!curr_imu) return PROS_ERR;

        return curr_imu.value()->tare_rotation();
    }

    virtual std::int32_t tare_pitch() {
        std::lock_guard lock(m_mutex);

        auto curr_imu = getAvailableImu();

        // no good imu
        if (!curr_imu) return PROS_ERR;

        return curr_imu.value()->tare_pitch();
    }

    virtual std::int32_t tare_yaw() {
        std::lock_guard lock(m_mutex);

        auto curr_imu = getAvailableImu();

        // no good imu
        if (!curr_imu) return PROS_ERR;

        return curr_imu.value()->tare_yaw();
    }

    virtual std::int32_t tare_roll() {
        std::lock_guard lock(m_mutex);

        auto curr_imu = getAvailableImu();

        // no good imu
        if (!curr_imu) return PROS_ERR;

        return curr_imu.value()->tare_roll();
    }

    virtual std::int32_t tare_euler() {
        std::lock_guard lock(m_mutex);

        auto curr_imu = getAvailableImu();

        // no good imu
        if (!curr_imu) return PROS_ERR;

        return curr_imu.value()->tare_euler();
    }

    virtual std::int32_t set_heading(double target) {
        std::lock_guard lock(m_mutex);

        auto curr_imu = getAvailableImu();

        // no good imu
        if (!curr_imu) return PROS_ERR;

        return curr_imu.value()->set_heading(target);
    }

    virtual std::int32_t set_pitch(double target) {
        std::lock_guard lock(m_mutex);

        auto curr_imu = getAvailableImu();

        // no good imu
        if (!curr_imu) return PROS_ERR;

        return curr_imu.value()->set_pitch(target);
    }

    virtual std::int32_t set_yaw(double target) {
        std::lock_guard lock(m_mutex);

        auto curr_imu = getAvailableImu();

        // no good imu
        if (!curr_imu) return PROS_ERR;

        return curr_imu.value()->set_yaw(target);
    }

    virtual std::int32_t set_roll(double target) {
        std::lock_guard lock(m_mutex);

        auto curr_imu = getAvailableImu();

        // no good imu
        if (!curr_imu) return PROS_ERR;

        return curr_imu.value()->set_roll(target);
    }

    virtual std::int32_t set_euler(pros::euler_s_t target) {
        std::lock_guard lock(m_mutex);

        auto curr_imu = getAvailableImu();

        // no good imu
        if (!curr_imu) return PROS_ERR;

        return curr_imu.value()->set_euler(target);
    }

    virtual std::int32_t tare() {
        std::lock_guard lock(m_mutex);

        auto curr_imu = getAvailableImu();

        // no good imu
        if (!curr_imu) return PROS_ERR;

        return curr_imu.value()->tare();
    }

    virtual pros::imu_orientation_e_t get_physical_orientation() {
        std::lock_guard lock(m_mutex);

        auto curr_imu = getAvailableImu();

        // no good imu
        if (!curr_imu) return pros::E_IMU_ORIENTATION_ERROR;

        return curr_imu.value()->get_physical_orientation();
    }

  private:
    mutable pros::Mutex m_mutex;
};
} // namespace vexmaps
