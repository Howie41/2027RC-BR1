#pragma once

#include "hwt901b.hpp"

#include <cmath>
#include <cstdint>

namespace odometry {

/// @brief 世界坐标系状态：位姿、速度、加速度，供点跟踪接口使用。
struct State {
    float x{0.0f};      // 全局 x (m)
    float y{0.0f};      // 全局 y (m)
    float yaw{0.0f};    // 航向角 (rad)
    float vx{0.0f};     // 全局速度 x (m/s)
    float vy{0.0f};     // 全局速度 y (m/s)
    float omega{0.0f};  // 角速度 (rad/s)
    float ax{0.0f};     // 全局加速度 x (m/s²)
    float ay{0.0f};     // 全局加速度 y (m/s²)
    float alpha{0.0f};  // 角加速度 (rad/s²)
};

/// @brief 世界坐标系估计器：维护全局位姿，后续用卡尔曼滤波融合 IMU/编码器/雷达。
class Estimator {
public:
    State state{};

    /// @brief 用 IMU 原始数据做惯性预测（当前为纯积分，后续替换为卡尔曼预测）。
    void predict(const imu::Data& imu, std::uint32_t now_ms);

private:
    std::uint32_t last_update_ms_{0};
    float last_omega_{0.0f};
    bool first_{true};
};

inline void Estimator::predict(const imu::Data& imu, std::uint32_t now_ms) {
    const float dt =
        first_ ? 0.0f : static_cast<float>(now_ms - last_update_ms_) / 1000.0f;
    first_ = false;
    last_update_ms_ = now_ms;

    // 航向与角速度直接来自 IMU 原始数据。
    state.yaw = imu.yaw;
    state.omega = imu.wz;

    // 角加速度 = 角速度差分。
    if (dt > 0.0f) state.alpha = (state.omega - last_omega_) / dt;
    last_omega_ = state.omega;

    // 机体加速度 → 全局加速度（绕 z 旋转 yaw）。
    const float c = std::cos(state.yaw);
    const float s = std::sin(state.yaw);
    state.ax = imu.ax * c - imu.ay * s;
    state.ay = imu.ax * s + imu.ay * c;

    // 积分得到全局速度与位置（纯惯性推算会漂移，后续用卡尔曼滤波融合多源）。
    if (dt > 0.0f) {
        state.vx += state.ax * dt;
        state.vy += state.ay * dt;
        state.x += state.vx * dt;
        state.y += state.vy * dt;
    }
}

}  // namespace odometry
