#include "robot.hpp"
#include "rtos_task.hpp"

#include <algorithm>

namespace {
// 摇杆满杆时的加速度（增量）。
constexpr float kLinearAccel = 5.0f;   // 左摇杆满杆线加速度 (m/s²)
constexpr float kAngularAccel = 3.0f;  // 右摇杆满杆角加速度 (rad/s²)
}  // namespace

// 底盘控制任务：摇杆给加速度（增量），积分成目标速度，由四轮转速 PID 做速度闭环。
rtos::Task<"chassis", [] {
    bool recovery_pressed = false;
    std::uint32_t last_tick = 0;
    bool first_tick = true;

    // 目标速度（摇杆加速度积分得到，掉线清零）。
    float vx_target = 0.0f;
    float vy_target = 0.0f;
    float omega_target = 0.0f;

    while (true) {
        const auto now = rtos::now_tick();
        const float dt = first_tick ? 0.0f : static_cast<float>(now - last_tick) / 1000.0f;
        first_tick = false;
        last_tick = now;

        if (robot::remote.online()) {
            // 车头 180° 取反；摇杆给加速度，无头模式旋转变换后积分成目标速度。
            const float sx = -robot::remote.left_x();
            const float sy = -robot::remote.left_y();
            robot::headless.update(robot::remote.switch_a() == 2000, robot::world_frame.state.yaw);
            float ax = 0.0f;
            float ay = 0.0f;
            robot::headless.apply(sx, sy, ax, ay);
            ax *= kLinearAccel;
            ay *= kLinearAccel;

            const float alpha = robot::remote.right_x() * kAngularAccel;

            vx_target = std::clamp(vx_target + ax * dt,
                -robot::Chassis::kMaxLinearSpeed, robot::Chassis::kMaxLinearSpeed);
            vy_target = std::clamp(vy_target + ay * dt,
                -robot::Chassis::kMaxLinearSpeed, robot::Chassis::kMaxLinearSpeed);
            omega_target = std::clamp(omega_target + alpha * dt,
                -robot::Chassis::kMaxAngularSpeed, robot::Chassis::kMaxAngularSpeed);

            // 通道 9（CH9，0 基下标 8）拨到 2000 = 请求故障恢复；上升沿触发一次。
            const bool recovery_now = robot::remote.channel(8) == 2000;
            if (recovery_now && !recovery_pressed) {
                robot::robot_context.recovery_requested.store(true);
            }
            recovery_pressed = recovery_now;

            // 通道 10（CH10，0 基下标 9）拨到 2000 = 强制掉线（按住生效）。
            robot::chassis.set_force_offline(robot::remote.channel(9) == 2000);
        } else {
            vx_target = 0.0f;
            vy_target = 0.0f;
            omega_target = 0.0f;
        }
        (void)robot::chassis.setVelocity(vx_target, vy_target, omega_target);
        robot::chassis.periodic();
        rtos::delay_until(now + 1);
    }
}, 1024 * 4, rtos::TaskPriority::High> chassis_task;
