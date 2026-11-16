#pragma once

#include <algorithm>
#include <cmath>

namespace tracking {

/// @brief 点跟踪（当前仅角度跟踪）：旋转时直接跟手，松开后用 PID 维持航向并抵抗扰动。
class PointTracker {
public:
    /// @brief 更新跟踪器并输出修正后的归一化角速度。
    /// @param omega_cmd 遥控的归一化旋转指令 (-1..1)。
    /// @param yaw 当前航向 (rad)，来自世界坐标系。
    /// @param omega 当前角速度 (rad/s)，来自世界坐标系。
    /// @param dt 时间步长 (s)，当前未使用。
    float update(float omega_cmd, float yaw, float omega, float dt);

    /// @brief 目标航向 (rad)。
    float target_yaw() const { return target_yaw_; }

private:
    static constexpr float kPi = 3.14159265f;
    static constexpr float kDeadband = 0.01f;  // 旋转指令死区
    static constexpr float kYawKp = 3.0f;      // 航向比例增益
    static constexpr float kYawKd = 0.5f;      // 航向阻尼（角速度做 D 项）

    float target_yaw_{0.0f};
    bool first_{true};
};

inline float PointTracker::update(float omega_cmd, float yaw, float omega, float dt) {
    (void)dt;
    if (first_) {
        first_ = false;
        target_yaw_ = yaw;
    }

    // 有旋转指令：直接输出角速度，目标航向跟随实际（松开后即锁定当前角度）。
    if (std::abs(omega_cmd) > kDeadband) {
        target_yaw_ = yaw;
        return omega_cmd;
    }

    // 无旋转指令：PID 维持目标航向，抵抗外部扰动（被踢后自动回正）。
    float err = target_yaw_ - yaw;
    while (err > kPi) err -= 2.0f * kPi;
    while (err < -kPi) err += 2.0f * kPi;

    return std::clamp(kYawKp * err - kYawKd * omega, -1.0f, 1.0f);
}

}  // namespace tracking
