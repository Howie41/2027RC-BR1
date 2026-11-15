#include "robot.hpp"
#include "rtos_task.hpp"

// 底盘控制任务：读取遥控，写入运动目标并周期驱动底盘。
rtos::Task<"chassis", [] {
    while (true) {
        const auto now = rtos::now_tick();
        float vx = 0.0f;
        float vy = 0.0f;
        float omega = 0.0f;
        if (robot::remote.online()) {
            vx = robot::remote.left_x();
            vy = -robot::remote.left_y();
            omega = robot::remote.right_x();
        }
        (void)robot::chassis.setVelocity(vx, vy, omega);
        robot::chassis.periodic();
        rtos::delay_until(now + 1);
    }
}, 1024 * 4, rtos::TaskPriority::High> chassis_task;
