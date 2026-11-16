#include "robot.hpp"
#include "rtos_task.hpp"

// 底盘控制任务：读取遥控，写入运动目标并周期驱动底盘。
rtos::Task<"chassis", [] {
    bool recovery_pressed = false;
    while (true) {
        const auto now = rtos::now_tick();
        float vx = 0.0f;
        float vy = 0.0f;
        float omega = 0.0f;
        if (robot::remote.online()) {
            vx = robot::remote.left_x();
            vy = -robot::remote.left_y();
            omega = robot::remote.right_x();

            // 通道 9（CH9，0 基下标 8）拨到 2000 = 请求故障恢复；上升沿触发一次。
            const bool recovery_now = robot::remote.channel(8) == 2000;
            if (recovery_now && !recovery_pressed) {
                robot::robot_context.recovery_requested.store(true);
            }
            recovery_pressed = recovery_now;
        }
        (void)robot::chassis.setVelocity(vx, vy, omega);
        robot::chassis.periodic();
        rtos::delay_until(now + 1);
    }
}, 1024 * 4, rtos::TaskPriority::High> chassis_task;
