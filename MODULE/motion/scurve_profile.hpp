#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace motion {

/// @brief 七段式 S 曲线速度规划器（一维运动，状态机实现）。
/// 段：加加速 → 匀加速 → 减加速 → 匀速 → 加减速 → 匀减速 → 减减速，
/// 保证到目标位移时速度与加速度都为零。
class SCurveProfile {
public:
    /// @brief 阶段。
    enum class Phase : std::uint8_t {
        Idle,
        AccelJerkUp,    // 加加速（加速度 0 → A）
        AccelConst,     // 匀加速
        AccelJerkDown,  // 减加速（加速度 A → 0）
        Cruise,         // 匀速
        DecelJerkUp,    // 加减速（加速度 0 → -D）
        DecelConst,     // 匀减速
        DecelJerkDown,  // 减减速（加速度 -D → 0）
        Done,
    };

    /// @brief 配置。
    struct Config {
        float max_velocity{0.0f};      // 最大速度（绝对值，>0）
        float max_acceleration{0.0f};  // 最大加速度（绝对值，>0）
        float max_deceleration{0.0f};  // 最大减速度（绝对值，0 表示等于加速度）
        float max_jerk{0.0f};          // 最大加加速度（绝对值，>0）
    };

    /// @brief 开始规划：从当前位置到目标位移（正负表示方向）。
    void plan(float displacement, const Config& config);

    /// @brief 推进一个时间步，返回当前速度（带方向符号）。
    float update(float dt);

    /// @brief 当前阶段。
    Phase phase() const { return phase_; }

    /// @brief 是否已完成（到达目标且速度、加速度都为零）。
    bool done() const { return phase_ == Phase::Done; }

    /// @brief 当前速度（带符号）。
    float velocity() const { return sign_ * velocity_; }

    /// @brief 当前加速度（带符号）。
    float acceleration() const { return sign_ * acceleration_; }

    /// @brief 累计位移（带符号）。
    float position() const { return sign_ * position_; }

private:
    /// @brief 从当前速度/加速度减速到静止所需的距离（S 曲线减速）。
    float stop_distance(float v, float a) const;

    Phase phase_{Phase::Idle};
    Config config_{};
    float sign_{1.0f};
    float displacement_{0.0f};
    float velocity_{0.0f};
    float acceleration_{0.0f};
    float position_{0.0f};
};

inline void SCurveProfile::plan(float displacement, const Config& config) {
    config_ = config;
    if (config_.max_deceleration <= 0.0f) {
        config_.max_deceleration = config_.max_acceleration;
    }
    sign_ = displacement >= 0.0f ? 1.0f : -1.0f;
    displacement_ = std::abs(displacement);
    velocity_ = 0.0f;
    acceleration_ = 0.0f;
    position_ = 0.0f;
    phase_ = displacement_ > 0.0f ? Phase::AccelJerkUp : Phase::Done;
}

inline float SCurveProfile::stop_distance(float v, float a) const {
    const float J = config_.max_jerk;
    const float D = config_.max_deceleration;

    // 仍在加速（a > 0）：先把加速度降到 0（减加速段），期间速度继续增加。
    if (a > 0.0f) {
        const float t = a / J;
        const float s0 = v * t + 0.5f * a * t * t - J * t * t * t / 6.0f;
        v = v + a * t - 0.5f * J * t * t;
        a = 0.0f;
        return s0 + stop_distance(v, a);
    }

    a = std::clamp(a, -D, 0.0f);
    if (v <= 0.0f) return 0.0f;

    float s = 0.0f;
    float vv = v;
    float aa = a;

    // 段 1：加加速度降到 -D。
    if (aa > -D) {
        const float t = (D + aa) / J;
        const float v_next = vv + aa * t - 0.5f * J * t * t;
        s += vv * t + 0.5f * aa * t * t - J * t * t * t / 6.0f;
        vv = std::max(v_next, 0.0f);
        aa = -D;
    }

    // 段 2：匀减速到 v_jerk（段 3 能恰好停住的速度）。
    const float v_jerk = D * D / (2.0f * J);
    if (vv > v_jerk) {
        const float t = (vv - v_jerk) / D;
        s += vv * t - 0.5f * D * t * t;
        vv = v_jerk;
    }

    // 段 3：减减速回到 0。
    if (vv >= v_jerk - 1e-6f) {
        s += D * D * D / (6.0f * J * J);
    } else {
        s += vv * vv / (2.0f * D);
    }

    return s;
}

inline float SCurveProfile::update(float dt) {
    if (phase_ == Phase::Idle || phase_ == Phase::Done) {
        return 0.0f;
    }
    if (dt <= 0.0f) {
        return sign_ * velocity_;
    }

    const float A = config_.max_acceleration;
    const float D = config_.max_deceleration;
    const float J = config_.max_jerk;
    const float V = config_.max_velocity;

    // 按阶段更新加速度。
    switch (phase_) {
    case Phase::AccelJerkUp:
        acceleration_ = std::min(acceleration_ + J * dt, A);
        break;
    case Phase::AccelConst:
        acceleration_ = A;
        break;
    case Phase::AccelJerkDown:
        acceleration_ = std::max(acceleration_ - J * dt, 0.0f);
        break;
    case Phase::Cruise:
        acceleration_ = 0.0f;
        velocity_ = V;
        break;
    case Phase::DecelJerkUp:
        acceleration_ = std::max(acceleration_ - J * dt, -D);
        break;
    case Phase::DecelConst:
        acceleration_ = -D;
        break;
    case Phase::DecelJerkDown:
        acceleration_ = std::min(acceleration_ + J * dt, 0.0f);
        break;
    default:
        break;
    }

    // 更新速度与位置。
    velocity_ = std::max(velocity_ + acceleration_ * dt, 0.0f);
    position_ = std::min(position_ + velocity_ * dt, displacement_);

    // 状态转移。
    const float remaining = displacement_ - position_;
    const float sd = stop_distance(velocity_, acceleration_);
    switch (phase_) {
    case Phase::AccelJerkUp:
        if (remaining <= sd) {
            phase_ = Phase::DecelJerkUp;
        } else if (acceleration_ >= A) {
            phase_ = Phase::AccelConst;
        }
        break;
    case Phase::AccelConst:
        if (remaining <= sd) {
            phase_ = Phase::DecelJerkUp;
        } else if (velocity_ >= V - A * A / (2.0f * J)) {
            phase_ = Phase::AccelJerkDown;
        }
        break;
    case Phase::AccelJerkDown:
        if (remaining <= sd) {
            phase_ = Phase::DecelJerkUp;
        } else if (acceleration_ <= 0.0f) {
            phase_ = Phase::Cruise;
        }
        break;
    case Phase::Cruise:
        if (remaining <= sd) {
            phase_ = Phase::DecelJerkUp;
        }
        break;
    case Phase::DecelJerkUp:
        if (acceleration_ <= -D) {
            phase_ = Phase::DecelConst;
        }
        break;
    case Phase::DecelConst:
        if (remaining <= sd) {
            phase_ = Phase::DecelJerkDown;
        }
        break;
    case Phase::DecelJerkDown:
        if (acceleration_ >= 0.0f && velocity_ <= 0.0f) {
            velocity_ = 0.0f;
            acceleration_ = 0.0f;
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
