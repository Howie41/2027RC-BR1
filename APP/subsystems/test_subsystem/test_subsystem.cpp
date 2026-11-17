#include "test_subsystem.hpp"

#include "rtos_utility.hpp"

#include <cmath>

namespace robot {

TestSubsystem::TestSubsystem(C620Motor& motor)
    : motor_(motor),
      speed_pid_("test_motor_speed", {
          .kp = 400.0f,
          .ki = 3000.0f,
          .kd = 0.0f,
          .max_out = 3000.0f,
          .improvement = {1U << PidController::Improvement::increment_of_out}
      }) {}

bool TestSubsystem::isOnline(FaultInfo& online_info) const {
    taskENTER_CRITICAL();
    const bool online = motor_.is_online();
    taskEXIT_CRITICAL();
    if (!online) {
        RECORD_FAULT(online_info, "Test motor offline");
    }
    return online;
}

bool TestSubsystem::setTargetSpeed(float speed) {
    if (!std::isfinite(speed)) return false;

    // 为子系统业务功能要用到的数据添加临界区保护
    taskENTER_CRITICAL();
    const bool accepted = acceptsCommands();
    if (accepted) {
        target_speed_ = speed;
    }
    taskEXIT_CRITICAL();
    return accepted;
}

void TestSubsystem::onRequestReset() noexcept {
    target_speed_ = 0.0f;
}

void TestSubsystem::onPrepare() {
    speed_pid_.init();
}

Subsystem::Readiness TestSubsystem::onStandby(bool entering) {
    if (entering && !setTargetSpeed(0.0f)) {
        // 测试电机拒绝待机所需的零速度目标。
        RECORD_FAULT(current_fault_, "Test motor rejected standby zero-speed target");
        return Readiness::Failed;
    }
    return Readiness::Ready;
}

Subsystem::Readiness TestSubsystem::onCheckReadiness() {
    bool online;
    // FDCAN 接收中断优先级为 5，在此临界区内不会改写反馈。
    taskENTER_CRITICAL();
    online = motor_.is_online();
    
    taskEXIT_CRITICAL();
    return online ? Readiness::Ready : Readiness::Pending;
}

// 子系统统一处理业务计算、输出应用和关闭。
bool TestSubsystem::onRun(bool enabled) {
    float current_ma = 0.0f;
    bool success = true;
    if (enabled) {
        float target_speed_snapshot;
        taskENTER_CRITICAL();
        target_speed_snapshot = target_speed_;
        measured_speed_ = motor_.get_current_speed();
        taskEXIT_CRITICAL();
        current_ma = speed_pid_.calculate(measured_speed_, target_speed_snapshot);
        success = std::isfinite(current_ma);
        if (!success) {
            // 测试电机 PID 输出为 NaN 或无穷大。
            RECORD_FAULT(current_fault_, "Test motor PID output is not finite");
        }
    }

    // 电机命令也由 CAN 通信任务读取；这里只同步命令
    taskENTER_CRITICAL();
    if (enabled && success) {
        success = motor_.set_output_command({.current_ma = current_ma});
        if (success) {
            motor_.enable();
        } else {
            // 测试电机拒绝输出电流命令。
            RECORD_FAULT(current_fault_, "Test motor rejected output current command");
            motor_.disable();
        }
    } else {
        motor_.disable();
    }
    taskEXIT_CRITICAL();
    return success;
}

} // namespace robot
