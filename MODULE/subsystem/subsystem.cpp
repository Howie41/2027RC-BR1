#include "subsystem.hpp"

#include "rtos_utility.hpp"

void Subsystem::requestPrepare() {
    taskENTER_CRITICAL();
    ++latest_request_.generation;
    latest_request_.mode = Mode::Prepare;
    latest_request_.reset_preparation = true;
    onRequestReset();
    state_.store(State::Preparing);
    taskEXIT_CRITICAL();
}

bool Subsystem::requestStandby() {
    taskENTER_CRITICAL();
    const bool accepted = !hasFailed();
    if (accepted && (isStandby() ||
                     (latest_request_.mode != Mode::Standby && latest_request_.mode != Mode::Prepare))) {
        latest_request_.reset_preparation = latest_request_.mode == Mode::Disabled;
        ++latest_request_.generation;
        latest_request_.mode = Mode::Standby;
        state_.store(State::Preparing);
    }
    taskEXIT_CRITICAL();
    return accepted;
    // 
}

bool Subsystem::requestRun() {
    taskENTER_CRITICAL();
    const bool accepted = (latest_request_.mode == Mode::Prepare || latest_request_.mode == Mode::Standby) &&
                          isStandby();
    if (accepted) latest_request_.mode = Mode::Run;
    taskEXIT_CRITICAL();
    return accepted;
}

void Subsystem::requestDisable() {
    taskENTER_CRITICAL();
    ++latest_request_.generation;
    latest_request_.mode = Mode::Disabled;
    latest_request_.reset_preparation = true;
    onRequestReset();
    taskEXIT_CRITICAL();
}

Subsystem::State Subsystem::state() const noexcept { return state_.load(); }
bool Subsystem::isStandby() const noexcept { return state() == State::Standby; }
bool Subsystem::isRunning() const noexcept { return state() == State::Running; }
bool Subsystem::hasFailed() const noexcept { return state() == State::Failed; }
FaultInfo Subsystem::faultInfo() const noexcept {
    taskENTER_CRITICAL();
    const auto info = fault_info_;
    taskEXIT_CRITICAL();
    return info;
}
// 判断现在是否接受业务目标。
bool Subsystem::acceptsCommands() const noexcept {
    const auto current_state = state();
    if (latest_request_.mode == Mode::Disabled || current_state == State::Failed) return false;
    if (current_state == State::Standby || current_state == State::Running) return true;

    // 仅放行本子系统的待机回调；其他任务和已被新请求替代的回调不能借用授权。
    return current_state == State::Preparing && standby_callback_task_ != nullptr &&
           standby_callback_task_ == xTaskGetCurrentTaskHandle() &&
           latest_request_.generation == active_generation_ &&
           (latest_request_.mode == Mode::Prepare || latest_request_.mode == Mode::Standby);
}
// 默认无需额外待机步骤，控制输出由 periodic() 统一执行。
Subsystem::Readiness Subsystem::onStandby(bool) {
    return Readiness::Ready;
}

void Subsystem::periodic() {
    current_fault_ = {};
    // 请求的快照
    Request request;
    taskENTER_CRITICAL();
    request = latest_request_;
    taskEXIT_CRITICAL();
    if (request.generation != active_generation_) {
        active_generation_ = request.generation;
        preparation_started_ = false;
        standby_started_ = false;
        if (request.reset_preparation) {
            prepared_ = false;
            (void)onRun(false);
            current_fault_ = {}; // 关闭输出不报告新的生命周期失败。
        }
        phase_ = request.mode == Mode::Disabled ? State::Disabled : State::Preparing;
    }

    const bool was_failed = phase_ == State::Failed;
    FaultInfo offline_fault{};
    if (request.mode == Mode::Disabled || phase_ == State::Failed) {
        (void)onRun(false);
    } 
    else if (!isOnline(offline_fault)) {
        (void)onRun(false);
        current_fault_ = {};
        // 仅在已建立运行条件或请求运行时才报告掉线；否则允许等待上线。
        if (request.mode == Mode::Run || prepared_) {
            current_fault_ = offline_fault;
            // 已建立运行条件后，进入/保持待机时掉线同样需要显式恢复。
            RECORD_FAULT(current_fault_, "Subsystem went offline after preparation");
            phase_ = State::Failed;
        } else {
            // 尚未建立运行条件时允许等待上线。
            phase_ = State::Preparing;
            prepared_ = false;
            preparation_started_ = false;
            standby_started_ = false;
        }
    } else {
        if (!preparation_started_) {
            preparation_started_ = true;
            if (!prepared_) onPrepare();
        }

        const auto readiness = onCheckReadiness();
        if (readiness == Readiness::Failed) {
            // 子系统运行条件检查失败。
            RECORD_FAULT(current_fault_, "Subsystem readiness check failed");
            phase_ = State::Failed;
        } else if (prepared_ && readiness != Readiness::Ready) {
            // 子系统此前已就绪，但已建立的运行条件失效。
            RECORD_FAULT(current_fault_, "Subsystem lost established readiness");
            phase_ = State::Failed;
        } else if (readiness == Readiness::Ready) {
            prepared_ = true;
            if (request.mode == Mode::Run) {
                phase_ = onRun(true) ? State::Running : State::Failed;
                if (phase_ == State::Failed) {
                    // 子系统在业务运行期间控制输出失败。
                    RECORD_FAULT(current_fault_, "Subsystem control failed while running");
                }
            } else {
                // 进这说明mode是Prepare或Standby，且已准备好，这里进入待机
                taskENTER_CRITICAL();
                standby_callback_task_ = xTaskGetCurrentTaskHandle();
                taskEXIT_CRITICAL();
                const auto standby = onStandby(!standby_started_);
                taskENTER_CRITICAL();
                standby_callback_task_ = nullptr;
                taskEXIT_CRITICAL();
                standby_started_ = true;
                if (standby == Readiness::Failed) {
                    // 子系统进入或保持待机的回调报告失败。
                    RECORD_FAULT(current_fault_, "Subsystem standby callback failed");
                    phase_ = State::Failed;
                } else if (!onRun(true)) {
                    // 子系统在进入或保持待机期间控制输出失败。
                    RECORD_FAULT(current_fault_, "Subsystem control failed while entering or holding standby");
                    phase_ = State::Failed;
                } else {
                    phase_ = standby == Readiness::Ready ? State::Standby : State::Preparing;
                }
            }
        }

        if (phase_ == State::Failed) (void)onRun(false);
    }

    // 计算期间的新请求在后续周期执行；本轮可能产生一次旧输出，但不能覆盖新状态。
    taskENTER_CRITICAL();
    if (latest_request_.generation == request.generation && latest_request_.mode == request.mode) {
        if (!was_failed && phase_ == State::Failed && current_fault_.reason != nullptr) {
            fault_info_ = current_fault_;
        }
        state_.store(phase_);
    }
    taskEXIT_CRITICAL();
}
