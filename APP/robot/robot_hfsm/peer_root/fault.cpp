#include "robot.hpp"
#include "states.hpp"

void robot::Fault::enter() {
    reentering_ = false;
    auto& context = robot_context;
    context.recovery_requested.store(false);
    // 本轮恢复成功前保留已有故障，重入 Fault 不能使先前禁用的机构自动待机。
    bool identified = false;
    // 必须先保存故障来源，再禁用；禁用完成后子系统不再处于 Failed。
    for (std::size_t i = 0; i < subsystems.size(); ++i) {
        // 定位故障
        if (subsystems[i] == context.pending_fault_source) {
            context.faults[i] = context.pending_fault_reason;
            context.fault_details[i] = context.pending_fault_details;
            // 子系统请求被拒绝，但未提供具体诊断信息时使用此兜底说明。
            RECORD_FAULT(context.fault_details[i], "Subsystem request rejected without diagnostic details");
        } else if (subsystems[i]->hasFailed()) {
            context.faults[i] = Robot_Context::FaultReason::SubsystemFailed;
            context.fault_details[i] = subsystems[i]->faultInfo();
            // 子系统已失败，但未提供具体诊断信息时使用此兜底说明。
            RECORD_FAULT(context.fault_details[i], "Subsystem failed without diagnostic details");
        }
        identified |= context.faults[i] != Robot_Context::FaultReason::None;
    }
    context.pending_fault_source = nullptr;
    context.pending_fault_reason = Robot_Context::FaultReason::None;
    context.pending_fault_details = {};
    
    // 没有明确来源的故障采用禁用所有子系统的策略
    if (!identified) {
        context.faults.fill(Robot_Context::FaultReason::Unspecified);
        for (auto& info : context.fault_details) {
            // 整机进入故障状态，但未能识别故障来源。
            RECORD_FAULT(info, "Robot entered Fault without an identified source");
        }
    }

    for (std::size_t i = 0; i < subsystems.size(); ++i) {
        if (context.faults[i] != Robot_Context::FaultReason::None) {
            subsystems[i]->requestDisable();
        } else if (!subsystems[i]->requestStandby()) {
            context.faults[i] = Robot_Context::FaultReason::SubsystemFailed;
            context.fault_details[i] = subsystems[i]->faultInfo();
            // 故障处置期间，子系统拒绝待机请求。
            RECORD_FAULT(context.fault_details[i], "Subsystem rejected standby during fault handling");
            subsystems[i]->requestDisable();
        }
    }
}

void robot::Fault::update(robot::Control& control) {
    auto& context = control.context();
    for (std::size_t i = 0; i < subsystems.size(); ++i) {
        if (context.faults[i] == Robot_Context::FaultReason::None &&
            subsystems[i]->hasFailed()) {
            // 待机机构出现新故障时重新执行 Fault::enter() 的处置策略。
            reentering_ = true;
            control.restart<Fault>();
            return;
        }
        if (context.faults[i] != Robot_Context::FaultReason::None &&
            subsystems[i]->state() != Subsystem::State::Disabled) return;
    }
    // 恢复输入预留：只有显式置位才开始恢复，设备上线本身不会使整机离开 Fault。
    if (context.recovery_requested.load()) control.changeTo<Recovering>();
}

void robot::Fault::exitGuard(robot::GuardControl& control) {
    // 故障扩大时仅重入当前 Fault，不属于离开故障状态，无需恢复授权。
    if (reentering_) return;
    const auto& context = control.context();
    if (!context.recovery_requested.load()) {
        control.cancelPendingTransitions();
        return;
    }
    for (std::size_t i = 0; i < subsystems.size(); ++i) {
        // 如果仍有fault未被处理，拒绝退出
        if (subsystems[i]->hasFailed() ||
            (context.faults[i] != Robot_Context::FaultReason::None &&
             subsystems[i]->state() != Subsystem::State::Disabled)) {
            control.cancelPendingTransitions();
            return;
        }
    }
}

void robot::Fault::exit() {
    robot_context.recovery_requested.store(false);
}
