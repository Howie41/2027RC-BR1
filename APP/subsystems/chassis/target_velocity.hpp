#pragma once

namespace robot {

/// @brief 上位机下发的目标速度（世界系，自动模式下由话题对象发布）。
struct TargetVelocity {
    float vx{0.0f};    // 目标线速度 x (m/s)
    float vy{0.0f};    // 目标线速度 y (m/s)
    float omega{0.0f}; // 目标角速度 (rad/s)
};

}  // namespace robot
