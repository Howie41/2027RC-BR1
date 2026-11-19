#pragma once

#include <algorithm>
#include <cmath>

namespace tracking {

/// @brief 点跟踪（当前仅角度跟踪）：旋转时直接跟手，松开后用位置式 PID 维持航向。
class PointTracker {
public:
    /// @brief 更新跟踪器并输出修正后的归一化角速度。
    /// @param omega_cmd 遥控的归一化旋转指令 (-1..1)。
    /// @param yaw 当前航向 (rad)，来自世界坐标系。
    /// @param omega 当前角速度 (rad/s)，用于微分阻尼。
    /// @param dt 时间步长 (s)，用于积分。
    float update(float omega_cmd, float yaw, float omega, float dt);

    /// @brief 目标航向 (rad)。
    float target_yaw() const { return target_yaw_; }

    /// @brief 复位跟踪器：目标航向清零，下次更新重新锁定当前航向。
    void reset() {
        target_yaw_ = 0.0f;
        integral_ = 0.0f;
        first_ = true;
    }

private:
    static constexpr float kPi = 3.14159265f;
    static constexpr float kDeadband = 0.01f;  // 旋转指令死区
    static constexpr float kYawKp = 1.8f;      // 比例（航向误差 → 归一化角速度）
    static constexpr float kYawKi = 1.6f;      // 积分（小，消除稳态误差）
    static constexpr float kYawKd = 0.5f;      // 微分（陀螺角速度阻尼）
    static constexpr float kMaxAngularSpeed = 3.14159265f;  // 180°/s，角速度归一化基准
    static constexpr float kIntegralLimit = 0.6f;  // 积分限幅 (rad·s)，抗饱和

    float target_yaw_{0.0f};
    float integral_{0.0f};
    bool first_{true};
};

inline float PointTracker::update(float omega_cmd, float yaw, float omega, float dt) {
    if (first_) {
        first_ = false;
        target_yaw_ = yaw;
        integral_ = 0.0f;
    }

    // 有旋转指令：直接输出角速度，目标航向跟随实际；积分清零，松开时从零重新累积。
    if (std::abs(omega_cmd) > kDeadband) {
        target_yaw_ = yaw;
        integral_ = 0.0f;
        return omega_cmd;
    }

    // 无旋转指令：位置式 PID 维持目标航向。
    // 误差归一化到 -π..π，处理 ±180° 跳变。
    float err = target_yaw_ - yaw;
    while (err > kPi) err -= 2.0f * kPi;
    while (err < -kPi) err += 2.0f * kPi;

    // 积分（限幅抗饱和，只起小幅度稳态修偏）。
    integral_ += err * dt;
    integral_ = std::clamp(integral_, -kIntegralLimit, kIntegralLimit);

    // 微分用陀螺角速度（d(err)/dt = -omega），构成一阶阻尼、抑制超调。
    const float omega_norm = omega / kMaxAngularSpeed;
    const float out = kYawKp * err + kYawKi * integral_ - kYawKd * omega_norm;
    return std::clamp(out, -1.0f, 1.0f);
}

}  // namespace tracking
