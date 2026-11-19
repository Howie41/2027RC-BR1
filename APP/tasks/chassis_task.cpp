#include "robot.hpp"
#include "rtos_task.hpp"

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

    while (true) {
        const auto now = rtos::now_tick();
        const float dt = first_tick ? 0.0f : static_cast<float>(now - last_tick) / 1000.0f;
        first_tick = false;
        last_tick = now;

        if (robot::remote.online()) {
            // 车头 180° 取反；摇杆给加速度（增量）。
            const float ax = -robot::remote.left_x() * kLinearAccel;
            const float ay = -robot::remote.left_y() * kLinearAccel;
            const float alpha = robot::remote.right_x() * kAngularAccel;

            const bool headless = robot::remote.switch_a() == 2000;
            robot::headless.update(headless, robot::world_frame.state.yaw);

            if (headless) {
                // 无头模式：锁定系（世界系）线速度积分，转机体系；角速度单独积分。
                robot::headless.integrateVelocity(ax, ay, dt);
                robot::chassis.integrateOmega(alpha, dt);
                float vx = 0.0f;
                float vy = 0.0f;
                robot::headless.bodyVelocity(vx, vy);
                robot::chassis.setLinearVelocity(vx, vy);
            } else {
                // 有头模式：机体系速度积分（平移 + 角速度）。
                robot::chassis.integrateVelocity(ax, ay, alpha, dt);
            }

            // SwD（通道8，0 基下标 7）拨到 2000 = 速度强行归零。
            if (robot::remote.switch_d() == 2000) {
                robot::chassis.resetVelocity();
                robot::headless.resetVelocity();
            }

            // 通道 9（CH9，0 基下标 8）拨到 2000 = 请求故障恢复；上升沿触发一次。
            const bool recovery_now = robot::remote.channel(8) == 2000;
            if (recovery_now && !recovery_pressed) {
                robot::robot_context.recovery_requested.store(true);
            }
            recovery_pressed = recovery_now;

            // 通道 10（CH10，0 基下标 9）拨到 2000 = 强制掉线（按住生效）。
            robot::chassis.set_force_offline(robot::remote.channel(9) == 2000);
        } else {
            robot::chassis.resetVelocity();
            robot::headless.resetVelocity();
        }
        robot::chassis.periodic();
        rtos::delay_until(now + 1);
    }
}, 1024 * 4, rtos::TaskPriority::High> chassis_task;
