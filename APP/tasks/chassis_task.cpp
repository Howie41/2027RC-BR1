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
            // 车头定义旋转 180°：摇杆前进/右平移取反，旋转（omega）不受影响。
            const float sx = -robot::remote.left_x();
            const float sy = -robot::remote.left_y();

            // 无头模式：SwA 2000 时摇杆平移相对固定世界方向，切换瞬间锁定当前航向为前方。
            robot::headless.update(robot::remote.switch_a() == 2000, robot::world_frame.state.yaw);
            robot::headless.apply(sx, sy, vx, vy);

            const float omega_cmd = -robot::remote.right_x();

            // 角度跟踪：位置式 PID 维持目标航向。
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
