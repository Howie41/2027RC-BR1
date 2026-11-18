#include "asserter.hpp"
#include "robot.hpp"
#include "rtos_task.hpp"

rtos::Task<"robot_machine_task", [] {
    const auto result = robot::robot_machine.initialResult();

    RUNTIME_ASSERT(result == robot::M::TransitionResult::Changed);

    while (true) {
        robot::robot_machine.update();
        rtos::delay(20);
    }

// 【s34 扩容】MemManage 崩溃定位到任务栈区，静态分析无深调用链，翻倍排除溢出分支
}, 8192, rtos::TaskPriority::Normal> robot_machine_task;
