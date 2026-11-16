#pragma once

#include "scurve_profile.hpp"

#include <algorithm>
#include <cmath>

namespace tracking {

/// @brief 点跟踪（当前仅角度跟踪）：旋转时直接跟手，松开后用 S 曲线平滑回正。
class PointTracker {
public:
    /// @brief 更新跟踪器并输出修正后的归一化角速度。
    /// @param omega_cmd 遥控的归一化旋转指令 (-1..1)。
    /// @param yaw 当前航向 (rad)，来自世界坐标系。
    /// @param omega 当前角速度 (rad/s)，当前未使用。
    /// @param dt 时间步长 (s)。
    float update(float omega_cmd, float yaw, float omega, float dt);

    /// @brief 目标航向 (rad)。
    float target_yaw() const { return target_yaw_; }

private:
    static constexpr float kPi = 3.14159265f;
    static constexpr float kMaxAngularSpeed = 1.5708f;  // 90°/s，与底盘解算一致
    static constexpr float kDeadband = 0.01f;           // 旋转指令死区
    static constexpr float kKickThreshold = 0.03f;      // 回正后再次被踢的判定阈值 (rad)

    static float wrap(float angle);
    static motion::SCurveProfile::Config config();

    motion::SCurveProfile profile_{};
    float target_yaw_{0.0f};
    bool need_plan_{false};
    bool first_{true};
};

inline float PointTracker::wrap(float angle) {
    while (angle > kPi) angle -= 2.0f * kPi;
    while (angle < -kPi) angle += 2.0f * kPi;
    return angle;
}

inline motion::SCurveProfile::Config PointTracker::config() {
    return {
        .max_velocity = kMaxAngularSpeed,  // 90°/s
        .max_acceleration = 4.0f,          // rad/s²
        .max_deceleration = 4.0f,          // rad/s²
        .max_jerk = 15.0f,                 // rad/s³
    };
}

inline float PointTracker::update(float omega_cmd, float yaw, float omega, float dt) {
    (void)omega;

    // 首次：目标设为当前航向，避免初始误差。
    if (first_) {
        first_ = false;
        target_yaw_ = yaw;
    }

    // 旋转中：直接输出角速度，目标跟随实际，下次松开时重新规划。
    if (std::abs(omega_cmd) > kDeadband) {
        target_yaw_ = yaw;
        need_plan_ = true;
        return omega_cmd;
    }

    // 松开：需要规划时，用当前航向误差规划 S 曲线回正。
    if (need_plan_) {
        profile_.plan(-wrap(target_yaw_ - yaw), config());
        need_plan_ = false;
    }

    // 已到目标：若又被踢（误差重新出现），重新规划；否则保持零速。
    if (profile_.done()) {
        const float err = wrap(target_yaw_ - yaw);
        if (std::abs(err) > kKickThreshold) {
            profile_.plan(-err, config());
        } else {
            return 0.0f;
        }
    }

    // 推进 S 曲线，输出归一化角速度。
    const float omega_rad_s = profile_.update(dt);
    return std::clamp(omega_rad_s / kMaxAngularSpeed, -1.0f, 1.0f);
}

}  // namespace tracking
