#include "robot.hpp"
#include "states.hpp"

void robot::TestRunning::entryGuard(robot::GuardControl& control) {
    for (const auto* subsystem : test_subsystems) {
        if (!subsystem->isRunning()) {
            control.cancelPendingTransitions();
            return;
        }
    }
}

void robot::TestRunning::enter() {
    robot_context.faults.fill(Robot_Context::FaultReason::None);
    robot_context.fault_details.fill(FaultInfo{});
    
    // EB: 这里初始化测试业务需要用的量 begin

    // EB: end
}

void robot::TestRunning::update(robot::Control& control) {
    for (const auto* subsystem : test_subsystems) {
        if (subsystem->hasFailed()) {
            control.changeTo<Fault>();
            return;
        }
    }

    // EB: 这里写测试业务逻辑

    // 底盘遥控器值更新
    chassis.updateRemoteValue(remote.online(),
        remote.left_x(), remote.left_y(), remote.right_x(),
        remote.switch_a() == 2000, remote.switch_d() == 2000);

    // EB: end
}

void robot::TestRunning::exit() {
}
