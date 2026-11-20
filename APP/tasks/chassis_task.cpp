#include "robot.hpp"
#include "rtos_task.hpp"

// 底盘控制任务：周期驱动底盘子系统（准备/待机/运行控制逻辑在 Chassis::onRun 内）。
rtos::Task<"chassis", [] {
    while (true) {
        const auto now = rtos::now_tick();
        robot::chassis.periodic();
        rtos::delay_until(now + 1);
    }
}, 1024 * 4, rtos::TaskPriority::High> chassis_task;
