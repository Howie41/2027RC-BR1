#include "robot.hpp"
#include "states.hpp"

void robot::Recovering::entryGuard(robot::GuardControl& control) {
    // 仅在 Fault 中显式请求恢复时才允许离开 Fault，其他情况均拒绝离开。
    if (!control.isActive<Fault>() || !control.context().recovery_requested.load()) {
        control.cancelPendingTransitions();
    }
}

void robot::Recovering::enter() {
    // 复位坐标系与无头模式：机器人恢复后位姿重新标定。
    world_frame.reset();
    headless.reset();

    phase_ = Phase::Preparing;
    for (auto* subsystem : subsystems) {
        if (subsystem->state() == Subsystem::State::Disabled) {
            subsystem->requestPrepare();
        } else {
            (void)subsystem->requestStandby();
        }
    }
}

void robot::Recovering::update(robot::Control& control) {
    auto& context = control.context();
    bool all_standby = true;
    bool all_running = true;
    for (std::size_t i = 0; i < subsystems.size(); ++i) {
        const auto state = subsystems[i]->state();
        if (state == Subsystem::State::Failed) {
            control.changeTo<Fault>();
            return;
        }
        all_standby &= state == Subsystem::State::Standby;
        all_running &= state == Subsystem::State::Running;
    }

    if (phase_ == Phase::Preparing) {
        if (!all_standby) return;
        for (auto* subsystem : subsystems) {
            if (!subsystem->requestRun()) {
                context.pending_fault_source = subsystem;
                context.pending_fault_reason = Robot_Context::FaultReason::RunRejected;
                // 故障恢复期间，子系统拒绝运行请求。
                RECORD_FAULT(context.pending_fault_details, "Subsystem rejected run during recovery");
                control.changeTo<Fault>();
                return;
            }
        }
        phase_ = Phase::WaitingRunning;
        return;
    }
    if (all_running) control.changeTo<UniversalManual>();
}

void robot::Recovering::exit() {}
