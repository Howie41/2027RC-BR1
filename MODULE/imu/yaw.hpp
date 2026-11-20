#pragma once

#include "hwt901b.hpp"

#include <cstdint>

namespace imu {

/// @brief 世界航向估计：yaw = imu.yaw - 偏置，复位后以当前方向为基准。
class YawEstimator {
public:
    float yaw{0.0f};  // 世界航向 (rad)

    /// @brief 用 IMU 数据刷新航向。
    void predict(const imu::Data& data, std::uint32_t now_ms);

    /// @brief 复位：以当前方向为基准，下次 predict 重新捕获偏置。
    void reset();

private:
    float yaw_bias_{0.0f};
    bool first_{true};
};

inline void YawEstimator::predict(const imu::Data& data, std::uint32_t now_ms) {
    (void)now_ms;
    if (first_) {
        yaw_bias_ = data.yaw;
        first_ = false;
    }
    yaw = data.yaw - yaw_bias_;
}

inline void YawEstimator::reset() {
    yaw = 0.0f;
    first_ = true;
}

}  // namespace imu
