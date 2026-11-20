#include "robot.hpp"
#include "states.hpp"

void robot::UniversalManual::entryGuard(robot::GuardControl& control) {
    for (const auto* subsystem : subsystems) {
        if (!subsystem->isRunning()) {
            control.cancelPendingTransitions();
            return;
        }
    }
}

void robot::UniversalManual::enter() {
    robot_context.faults.fill(Robot_Context::FaultReason::None);
    robot_context.fault_details.fill(FaultInfo{});
}

void robot::UniversalManual::update(robot::Control& control) {
    for (const auto* subsystem : subsystems) {
        if (subsystem->hasFailed()) {
            control.changeTo<Fault>();
            return;
        }
    }

    
    // 底盘遥控器值更新
    chassis.updateRemoteValue(remote.online(),
        remote.left_x(), remote.left_y(), remote.right_x(),
        remote.switch_a() == 2000, remote.switch_d() == 2000,
        remote.channel(9) == 2000);  // CH10 = 速度/加速度模式切换


}

void robot::UniversalManual::exit() {}
