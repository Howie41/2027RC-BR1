#include "robot.hpp"
#include "rtos_task.hpp"

// 底盘控制任务：读取遥控，角度跟踪后写入运动目标并周期驱动底盘。
rtos::Task<"chassis", [] {
    bool recovery_pressed = false;
    std::uint32_t last_tick = 0;
    bool first_tick = true;
    while (true) {
        const auto now = rtos::now_tick();
        const float dt = first_tick ? 0.0f : static_cast<float>(now - last_tick) / 1000.0f;
        first_tick = false;
        last_tick = now;

        float vx = 0.0f;
        float vy = 0.0f;
        float omega = 0.0f;
        if (robot::remote.online()) {
            vx = robot::remote.left_x();
            vy = robot::remote.left_y();
            const float omega_cmd = -robot::remote.right_x();

            // 角度跟踪：遥控旋转指令积分成目标航向，PD 修正抵抗外部扰动。
            omega = robot::point_tracker.update(
                omega_cmd, robot::world_frame.state.yaw, robot::world_frame.state.omega, dt);

            // 通道 9（CH9，0 基下标 8）拨到 2000 = 请求故障恢复；上升沿触发一次。
            const bool recovery_now = robot::remote.channel(8) == 2000;
            if (recovery_now && !recovery_pressed) {
                robot::robot_context.recovery_requested.store(true);
            }
            recovery_pressed = recovery_now;

            // 通道 10（CH10，0 基下标 9）拨到 2000 = 强制掉线（按住生效）。
            robot::chassis.set_force_offline(robot::remote.channel(9) == 2000);
        }
        (void)robot::chassis.setVelocity(vx, vy, omega);
        robot::chassis.periodic();
        rtos::delay_until(now + 1);
    }
}, 1024 * 4, rtos::TaskPriority::High> chassis_task;
