#pragma once

#include <algorithm>
#include <cmath>

namespace tracking {

/// @brief 点跟踪（当前仅角度跟踪）：旋转时直接跟手，松开后锁定航向并抵抗扰动。
class PointTracker {
public:
    /// @brief 更新跟踪器并输出修正后的归一化角速度。
    /// @param omega_cmd 遥控的归一化旋转指令 (-1..1)。
    /// @param yaw 当前航向 (rad)，来自世界坐标系。
    /// @param omega 当前角速度 (rad/s)，当前未使用。
    /// @param dt 时间步长 (s)，当前未使用，预留给位置跟踪。
    float update(float omega_cmd, float yaw, float omega, float dt);

    /// @brief 目标航向 (rad)。
    float target_yaw() const { return target_yaw_; }

private:
    static constexpr float kPi = 3.14159265f;
    static constexpr float kDeadband = 0.01f;  // 旋转指令死区
    static constexpr float kDecel = 2.0f;      // 回正减速度（越大停得越快）

    float target_yaw_{0.0f};
    bool first_{true};
};

inline float PointTracker::update(float omega_cmd, float yaw, float omega, float dt) {
    (void)dt;
    (void)omega;
    if (first_) {
        first_ = false;
        target_yaw_ = yaw;
    }

    // 有旋转指令：直接输出角速度，目标航向跟随实际（松开后即锁定当前角度）。
    if (std::abs(omega_cmd) > kDeadband) {
        target_yaw_ = yaw;
        return omega_cmd;
    }

    // 无旋转指令：梯形速度曲线回正——误差大时全速，接近目标按 sqrt(2·a·|err|) 自动减速，
    // 到目标时速度恰为 0，避免超调；只依赖航向误差，不引入陀螺仪噪声。
    float err = target_yaw_ - yaw;
    while (err > kPi) err -= 2.0f * kPi;
    while (err < -kPi) err += 2.0f * kPi;

    float out = std::sqrt(2.0f * kDecel * std::abs(err));
    out = std::copysign(out, err);
    return std::clamp(out, -1.0f, 1.0f);
}

}  // namespace tracking
