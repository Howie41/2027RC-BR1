#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace motion {

/// @brief 梯形速度曲线规划器（一维运动，状态机实现）。
/// 阶段：加速 → 匀速 → 减速，保证到目标位移时速度恰为零。
class TrapezoidProfile {
public:
    /// @brief 阶段。
    enum class Phase : std::uint8_t { Idle, Accel, Cruise, Decel, Done };

    /// @brief 配置。
    struct Config {
        float max_velocity{0.0f};      // 最大速度（绝对值，>0）
        float max_acceleration{0.0f};  // 最大加速度（绝对值，>0）
        float max_deceleration{0.0f};  // 最大减速度（绝对值，0 表示等于加速度）
    };

    /// @brief 开始规划：从当前位置到目标位移（正负表示方向）。
    void plan(float displacement, const Config& config);

    /// @brief 推进一个时间步，返回当前速度（带方向符号）。
    float update(float dt);

    /// @brief 当前阶段。
    Phase phase() const { return phase_; }

    /// @brief 是否已完成（到达目标且速度为零）。
    bool done() const { return phase_ == Phase::Done; }

    /// @brief 当前速度（带符号）。
    float velocity() const { return sign_ * velocity_; }

    /// @brief 累计位移（带符号）。
    float position() const { return sign_ * position_; }

private:
    Phase phase_{Phase::Idle};
    Config config_{};
    float sign_{1.0f};
    float displacement_{0.0f};
    float velocity_{0.0f};
    float position_{0.0f};
};

inline void TrapezoidProfile::plan(float displacement, const Config& config) {
    config_ = config;
    if (config_.max_deceleration <= 0.0f) {
        config_.max_deceleration = config_.max_acceleration;
    }
    sign_ = displacement >= 0.0f ? 1.0f : -1.0f;
    displacement_ = std::abs(displacement);
    velocity_ = 0.0f;
    position_ = 0.0f;
    phase_ = displacement_ > 0.0f ? Phase::Accel : Phase::Done;
}

inline float TrapezoidProfile::update(float dt) {
    if (phase_ == Phase::Idle || phase_ == Phase::Done) {
        return 0.0f;
    }
    if (dt <= 0.0f) {
        return sign_ * velocity_;
    }

    // 按阶段更新速度。
    switch (phase_) {
    case Phase::Accel:
        velocity_ = std::min(velocity_ + config_.max_acceleration * dt, config_.max_velocity);
        break;
    case Phase::Cruise:
        velocity_ = config_.max_velocity;
        break;
    case Phase::Decel:
        velocity_ = std::max(velocity_ - config_.max_deceleration * dt, 0.0f);
        break;
    default:
        break;
    }

    // 更新累计位移。
    position_ = std::min(position_ + velocity_ * dt, displacement_);

    // 状态转移。
    const float remaining = displacement_ - position_;
    const float decel_distance = velocity_ * velocity_ / (2.0f * config_.max_deceleration);
    switch (phase_) {
    case Phase::Accel:
        if (remaining <= decel_distance) {
            phase_ = Phase::Decel;  // 剩余距离已不足以继续加速，开始减速。
        } else if (velocity_ >= config_.max_velocity) {
            phase_ = Phase::Cruise;
        }
        break;
    case Phase::Cruise:
        if (remaining <= decel_distance) {
            phase_ = Phase::Decel;
        }
        break;
    case Phase::Decel:
        if (velocity_ <= 0.0f || position_ >= displacement_) {
            velocity_ = 0.0f;
            position_ = displacement_;
            phase_ = Phase::Done;
        }
        break;
    default:
        break;
    }

    return sign_ * velocity_;
}

}  // namespace motion
