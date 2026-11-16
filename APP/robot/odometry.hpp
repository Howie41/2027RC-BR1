#pragma once

#include "hwt901b.hpp"

#include <cmath>
#include <cstdint>

namespace robot {

/// @brief 全局坐标系里程计：维护位姿、速度与加速度，供点跟踪接口使用。
class Odometry {
public:
    float x{0.0f};      // 全局 x (m)
    float y{0.0f};      // 全局 y (m)
    float yaw{0.0f};    // 航向角 (rad)
    float vx{0.0f};     // 全局速度 x (m/s)
    float vy{0.0f};     // 全局速度 y (m/s)
    float omega{0.0f};  // 角速度 (rad/s)
    float ax{0.0f};     // 全局加速度 x (m/s²)
    float ay{0.0f};     // 全局加速度 y (m/s²)
    float alpha{0.0f};  // 角加速度 (rad/s²)

    /// @brief 用最新 IMU 数据更新里程计（在角度帧到达时调用）。
    void update(const imu::Data& data, std::uint32_t now_ms);

private:
    std::uint32_t last_update_ms_{0};
    float last_omega_{0.0f};
    bool first_{true};
};

inline void Odometry::update(const imu::Data& data, std::uint32_t now_ms) {
    const float dt =
        first_ ? 0.0f : static_cast<float>(now_ms - last_update_ms_) / 1000.0f;
    first_ = false;
    last_update_ms_ = now_ms;

    // 航向与角速度直接来自 IMU。
    yaw = data.yaw;
    omega = data.wz;

    // 角加速度 = 角速度差分。
    if (dt > 0.0f) alpha = (omega - last_omega_) / dt;
    last_omega_ = omega;

    // 机体加速度 → 全局加速度（绕 z 旋转 yaw）。
    const float c = std::cos(yaw);
    const float s = std::sin(yaw);
    ax = data.ax * c - data.ay * s;
    ay = data.ax * s + data.ay * c;

    // 积分得到全局速度与位置（IMU 纯惯性推算会漂移，后续融合轮速/点跟踪修正）。
    if (dt > 0.0f) {
        vx += ax * dt;
        vy += ay * dt;
        x += vx * dt;
        y += vy * dt;
    }
}

}  // namespace robot
