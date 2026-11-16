#pragma once

#include <algorithm>
#include <cmath>

namespace tracking {

/// @brief 点跟踪（当前仅角度跟踪）：旋转时直接跟手，松开后用增量式 PID 维持航向。
class PointTracker {
public:
    /// @brief 更新跟踪器并输出修正后的归一化角速度。
    /// @param omega_cmd 遥控的归一化旋转指令 (-1..1)。
    /// @param yaw 当前航向 (rad)，来自世界坐标系。
    /// @param omega 当前角速度 (rad/s)，当前未使用（增量式 D 项用误差差分）。
    /// @param dt 时间步长 (s)，当前未使用。
    float update(float omega_cmd, float yaw, float omega, float dt);

    /// @brief 目标航向 (rad)。
    float target_yaw() const { return target_yaw_; }

    /// @brief 复位跟踪器：目标航向清零，下次更新重新锁定当前航向。
    void reset() {
        target_yaw_ = 0.0f;
        last_err_ = 0.0f;
        last_d_err_ = 0.0f;
        omega_out_ = 0.0f;
        first_ = true;
    }

private:
    static constexpr float kPi = 3.14159265f;
    static constexpr float kDeadband = 0.01f;  // 旋转指令死区
    static constexpr float kYawKp = 3.2f;      // 比例（误差增量）
    static constexpr float kYawKi = 0.0f;      // 积分（当前误差，0 表示关闭）
    static constexpr float kYawKd = 0.5f;      // 微分（误差增量的一阶差分）

    float target_yaw_{0.0f};
    float last_err_{0.0f};
    float last_d_err_{0.0f};
    float omega_out_{0.0f};  // 增量式累计输出
    bool first_{true};
};

inline float PointTracker::update(float omega_cmd, float yaw, float omega, float dt) {
    (void)omega;
    (void)dt;
    if (first_) {
        first_ = false;
        target_yaw_ = yaw;
        last_err_ = 0.0f;
        last_d_err_ = 0.0f;
        omega_out_ = 0.0f;
    }

    // 有旋转指令：直接输出角速度，目标航向跟随实际；增量状态复位，松开时从当前指令无跳变继续。
    if (std::abs(omega_cmd) > kDeadband) {
        target_yaw_ = yaw;
        omega_out_ = omega_cmd;
        last_err_ = 0.0f;
        last_d_err_ = 0.0f;
        return omega_cmd;
    }

    // 无旋转指令：增量式 PID 维持目标航向。
    const float err = target_yaw_ - yaw;

    // 误差增量（wrap 到 -π..π，处理穿越 ±π 的跳变）。
    float d_err = err - last_err_;
    while (d_err > kPi) d_err -= 2.0f * kPi;
    while (d_err < -kPi) d_err += 2.0f * kPi;

    // 误差增量的一阶差分（二阶差分）。
    const float d2_err = d_err - last_d_err_;

    omega_out_ += kYawKp * d_err + kYawKi * err + kYawKd * d2_err;
    omega_out_ = std::clamp(omega_out_, -1.0f, 1.0f);

    last_err_ = err;
    last_d_err_ = d_err;

    return omega_out_;
}

}  // namespace tracking
