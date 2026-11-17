#pragma once

#include <cmath>

namespace robot {

/// @brief 无头模式：SwA 2000 时摇杆平移相对固定世界方向，切换瞬间锁定当前航向为前方。
class HeadlessMode {
public:
    static constexpr float k180 = 180.0f;
    static constexpr float kDegToRad = 3.14159265f / 180.0f;

    /// @brief 当前是否无头模式。
    bool active() const { return active_; }

    /// @brief 相对锁定前方的航向 (°)。
    float relative_yaw() const { return yaw_; }

    /// @brief 更新模式并刷新相对航向；拨到无头的瞬间锁定当前航向为前方。
    /// @param enabled SwA==2000。
    /// @param world_yaw 当前世界航向 (°)。
    void update(bool enabled, float world_yaw) {
        if (enabled && !active_) bias_ = world_yaw;
        active_ = enabled;
        yaw_ = world_yaw - bias_;
        while (yaw_ > k180) yaw_ -= 2.0f * k180;
        while (yaw_ < -k180) yaw_ += 2.0f * k180;
    }

    /// @brief 复位：回到有头模式，偏置清零。
    void reset() {
        active_ = false;
        bias_ = 0.0f;
        yaw_ = 0.0f;
    }

    /// @brief 摇杆平移 (sx,sy) → 机体系平移 (vx,vy)。有头模式直接透传。
    void apply(float sx, float sy, float& vx, float& vy) const {
        if (!active_) {
            vx = sx;
            vy = sy;
            return;
        }
        const float yaw_rad = yaw_ * kDegToRad;
        const float c = std::cos(yaw_rad);
        const float s = std::sin(yaw_rad);
        vx = c * sx + s * sy;
        vy = -s * sx + c * sy;
    }

private:
    bool active_{false};
    float bias_{0.0f};
    float yaw_{0.0f};
};

}  // namespace robot
