#pragma once

#include <cmath>

namespace robot {

/// @brief 无头模式：SwA 2000 时摇杆平移相对固定世界方向，切换瞬间锁定当前航向为前方。
class HeadlessMode {
public:
    static constexpr float kPi = 3.14159265f;

    /// @brief 当前是否无头模式。
    bool active() const { return active_; }

    /// @brief 相对锁定前方的航向 (rad)。
    float relative_yaw() const { return yaw_; }

    /// @brief 更新模式并刷新相对航向；拨到无头的瞬间锁定当前航向为前方。
    void update(bool enabled, float world_yaw) {
        if (enabled && !active_) bias_ = world_yaw;
        active_ = enabled;
        yaw_ = world_yaw - bias_;
        while (yaw_ > kPi) yaw_ -= 2.0f * kPi;
        while (yaw_ < -kPi) yaw_ += 2.0f * kPi;
    }

    /// @brief 积分锁定系（世界系）线速度，避免速度随车身旋转。
    void integrateVelocity(float ax, float ay, float dt) {
        if (active_) {
            vx_ += ax * dt;
            vy_ += ay * dt;
        }
    }

    /// @brief 锁定系线速度 → 机体系线速度。
    void bodyVelocity(float& vx, float& vy) const {
        const float c = std::cos(yaw_);
        const float s = std::sin(yaw_);
        vx = c * vx_ + s * vy_;
        vy = -s * vx_ + c * vy_;
    }

    /// @brief 清零锁定系线速度。
    void resetVelocity() {
        vx_ = 0.0f;
        vy_ = 0.0f;
    }

    /// @brief 复位：回到有头模式，偏置与速度清零。
    void reset() {
        active_ = false;
        bias_ = 0.0f;
        yaw_ = 0.0f;
        vx_ = 0.0f;
        vy_ = 0.0f;
    }

private:
    bool active_{false};
    float bias_{0.0f};
    float yaw_{0.0f};
    float vx_{0.0f};  // 锁定系线速度 (m/s)
    float vy_{0.0f};  // 锁定系线速度 (m/s)
};

}  // namespace robot
