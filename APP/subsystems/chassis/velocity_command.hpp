#pragma once

namespace robot {

/// @brief 上位机下发的速度指令（目标 + 实时，世界系）。
struct VelocityCommand {
    float target_vx{0.0f};    // 目标线速度 x (m/s)
    float target_vy{0.0f};    // 目标线速度 y (m/s)
    float target_omega{0.0f}; // 目标角速度 (rad/s)
    float actual_vx{0.0f};    // 实时线速度 x (m/s，雷达测量)
    float actual_vy{0.0f};    // 实时线速度 y (m/s，雷达测量)
    float actual_omega{0.0f}; // 实时角速度 (rad/s，雷达测量)
};

}  // namespace robot
