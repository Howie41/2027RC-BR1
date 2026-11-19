#pragma once

#include "hwt901b.hpp"

#include <cstdint>

namespace odometry {

/// @brief 世界坐标系状态：当前仅保留航向与角速度（位置由上位机雷达提供）。
struct State {
    float yaw{0.0f};    // 航向角 (rad)
    float omega{0.0f};  // 角速度 (rad/s)
};

/// @brief 航向估计器：世界 yaw = imu.yaw - 偏置。
class Estimator {
public:
    State state{};

    /// @brief 用 IMU 数据更新航向。
    void predict(const imu::Data& imu, std::uint32_t now_ms);

    /// @brief 复位坐标系：以当前方向为基准 y 轴。
    void reset();

private:
    float yaw_bias_{0.0f};  // 航向偏置：世界 yaw = imu.yaw - yaw_bias_
    bool first_{true};
};

inline void Estimator::predict(const imu::Data& imu, std::uint32_t now_ms) {
    (void)now_ms;
    if (first_) {
        // 首次预测：以当前 IMU yaw 为基准，世界 yaw 从 0 开始（IMU 可能掉电前就在跑）。
        yaw_bias_ = imu.yaw;
        first_ = false;
    }
    state.yaw = imu.yaw - yaw_bias_;
    state.omega = imu.wz;
}

inline void Estimator::reset() {
    state = {};
    first_ = true;  // 下次 predict 重新以当前 yaw 为基准（当前方向 = 基准 y 轴）
}

}  // namespace odometry
