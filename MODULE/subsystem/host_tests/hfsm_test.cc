#include "robot.hpp"

#include <cassert>
#include <iostream>

using namespace robot;
using SubsystemState = Subsystem::State;
using Reason = Robot_Context::FaultReason;

void tickAll() {
    for (auto* subsystem : subsystems) subsystem->periodic();
}

template<class T> void reach() {
    for (unsigned i = 0; i < 20 && !robot_machine.isActive<T>(); ++i) {
        tickAll();
        robot_machine.update();
    }
    assert(robot_machine.isActive<T>());
}

int main() {
    // 初始为 Disable；业务与恢复入口均有保护。
    assert(robot_machine.isActive<Disable>());
    assert(robot_machine.immediateChangeTo<UniversalManual>() == M::TransitionResult::RejectedByEntryGuard);
    assert(robot_machine.immediateChangeTo<Recovering>() == M::TransitionResult::RejectedByEntryGuard);

    // 上电后自动进入正式启动流程：Disable → Enable → Starting。
    robot_machine.update();
    assert(robot_machine.isActive<Starting>());

    // 各子系统独立等待上线、准备并进入待机。
    first_subsystem.online = true;
    first_subsystem.ready = true;
    tickAll();
    robot_machine.update();
    assert(first_subsystem.isStandby() && first_subsystem.output);
    assert(second_subsystem.state() == SubsystemState::Preparing && !second_subsystem.output);

    second_subsystem.online = true;
    second_subsystem.ready = true;
    reach<UniversalManual>();
    assert(first_subsystem.isRunning() && second_subsystem.isRunning());
    assert(first_subsystem.prepares == 1 && second_subsystem.prepares == 1);
    std::cout << "PASS: independent preparation and formal run confirmation\n";

    // 子系统故障：隔离故障对象，健康对象待机，显式授权后恢复。
    first_subsystem.online = false;
    tickAll();
    robot_machine.update();
    assert(robot_machine.isActive<Fault>() && robot_context.faults[0] == Reason::SubsystemFailed);
    assert(robot_context.faults[1] == Reason::None);
    tickAll();
    assert(first_subsystem.state() == SubsystemState::Disabled && second_subsystem.isStandby());

    // 设备重新上线不会自动离开 Fault，需要显式恢复授权。
    first_subsystem.online = true;
    tickAll();
    robot_machine.update();
    assert(robot_machine.isActive<Fault>());
    robot_context.recovery_requested.store(true);
    robot_machine.update();
    assert(robot_machine.isActive<Recovering>() && !robot_context.recovery_requested.load());
    reach<UniversalManual>();
    assert(first_subsystem.isRunning() && second_subsystem.isRunning());
    std::cout << "PASS: selective fault isolation and explicit recovery\n";

    // 恢复期间再次失败需重新进入 Fault 并重新授权。
    first_subsystem.online = false;
    tickAll();
    robot_machine.update();
    tickAll();
    robot_context.recovery_requested.store(true);
    robot_machine.update();
    assert(robot_machine.isActive<Recovering>());
    first_subsystem.online = true;
    first_subsystem.run_success = false;
    tickAll();
    robot_machine.update();
    assert(robot_machine.isActive<Fault>() && !robot_context.recovery_requested.load());
    first_subsystem.run_success = true;
    robot_context.recovery_requested.store(true);
    robot_machine.update();
    reach<UniversalManual>();
    std::cout << "PASS: failed recovery re-enters Fault and requires fresh authorization\n";

    // 未知整机故障默认禁用全部子系统。
    robot_machine.immediateChangeTo<Fault>();
    tickAll();
    assert(first_subsystem.state() == SubsystemState::Disabled && second_subsystem.state() == SubsystemState::Disabled);
    assert(robot_context.faults[0] == Reason::Unspecified && robot_context.faults[1] == Reason::Unspecified);
    robot_context.recovery_requested.store(true);
    robot_machine.update();
    reach<UniversalManual>();
    std::cout << "PASS: unknown faults disable all and recover to manual\n";
}
