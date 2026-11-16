#pragma once

#include <algorithm>
#include <cmath>

namespace tracking {

/// @brief 点跟踪（当前仅角度跟踪）：维持目标航向，抵抗外部扰动。
class PointTracker {
public:
    /// @brief 更新跟踪器并输出修正后的归一化角速度。
    /// @param omega_cmd 遥控的归一化旋转指令 (-1..1)。
    /// @param yaw 当前航向 (rad)，来自世界坐标系。
    /// @param omega 当前角速度 (rad/s)，来自世界坐标系。
    /// @param dt 时间步长 (s)。
    float update(float omega_cmd, float yaw, float omega, float dt);

    /// @brief 目标航向 (rad)。
    float target_yaw() const { return target_yaw_; }

private:
    static constexpr float kMaxAngularSpeed = 1.5708f;  // 90°/s，与底盘解算一致
    static constexpr float kPi = 3.14159265f;
    static constexpr float kYawKp = 3.0f;  // 航向比例增益
    static constexpr float kYawKd = 0.5f;  // 航向阻尼（用角速度做 D 项）

    float target_yaw_{0.0f};
    bool first_{true};
};

inline float PointTracker::update(float omega_cmd, float yaw, float omega, float dt) {
    if (first_) {
        first_ = false;
        target_yaw_ = yaw;
        return omega_cmd;
    }

    // 目标航向 = 遥控旋转指令积分。
    target_yaw_ += omega_cmd * kMaxAngularSpeed * dt;

    // 航向误差归一化到 -π..π。
    float err = target_yaw_ - yaw;
    while (err > kPi) err -= 2.0f * kPi;
    while (err < -kPi) err += 2.0f * kPi;

    // PD：比例（误差）+ 微分（角速度阻尼）。
    return std::clamp(kYawKp * err - kYawKd * omega, -1.0f, 1.0f);
}

}  // namespace tracking
