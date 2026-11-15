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
        }
        phase_ = request.mode == Mode::Disabled ? State::Disabled : State::Preparing;
    }

    if (request.mode == Mode::Disabled || phase_ == State::Failed) {
        (void)onRun(false);
    } 
    else if (!isOnline()) {
        (void)onRun(false);
        if (request.mode == Mode::Run || prepared_) {
            // 已建立运行条件后，进入/保持待机时掉线同样需要显式恢复。
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
        if (readiness == Readiness::Failed ||
            (prepared_ && readiness != Readiness::Ready)) {
            phase_ = State::Failed;
        } else if (readiness == Readiness::Ready) {
            prepared_ = true;
            if (request.mode == Mode::Run) {
                phase_ = onRun(true) ? State::Running : State::Failed;
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
                if (standby == Readiness::Failed || !onRun(true)) phase_ = State::Failed;
                else phase_ = standby == Readiness::Ready ? State::Standby : State::Preparing;
            }
        }

        if (phase_ == State::Failed) (void)onRun(false);
    }

    // 计算期间的新请求在后续周期执行；本轮可能产生一次旧输出，但不能覆盖新状态。
    taskENTER_CRITICAL();
    if (latest_request_.generation == request.generation && latest_request_.mode == request.mode) {
        state_.store(phase_);
    }
    taskEXIT_CRITICAL();
}
