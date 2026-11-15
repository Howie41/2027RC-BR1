#include "chassis.hpp"

#include "rtos_utility.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace robot {

namespace {
constexpr float kMaxSpeedRadS = 10.4719f;  // 600 deg/s
constexpr float kMaxOmegaRadS = 4.3633f;   // 250 deg/s
}  // namespace

Chassis::Chassis(C620Motor& fr, C620Motor& fl, C620Motor& rl, C620Motor& rr)
    : motors_{&fr, &fl, &rl, &rr},
      fr_pid_("chassis_fr_speed"),
      fl_pid_("chassis_fl_speed"),
      rl_pid_("chassis_rl_speed"),
      rr_pid_("chassis_rr_speed"),
      pids_{&fr_pid_, &fl_pid_, &rl_pid_, &rr_pid_} {
    const PidController::Config config{
        .kp = 20.0f,
        .ki = 2.0f,
        .kd = 0.0f,
        .max_out = 20000.0f,
        .integral_limit = 5000.0f,
        .improvement = {1U << PidController::integral_limit},
    };
    for (auto* pid : pids_) pid->configure(config);
}

bool Chassis::all_online() const {
    taskENTER_CRITICAL();
    bool online = true;
    for (const auto* motor : motors_) online &= motor->is_online();
    taskEXIT_CRITICAL();
    return online;
}

bool Chassis::isOnline() const {
    return all_online();
}

bool Chassis::setVelocity(float vx, float vy, float omega) {
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(omega)) return false;

    taskENTER_CRITICAL();
    const bool running = acceptsCommands();
    if (running) {
        vx_ = std::clamp(vx, -1.0f, 1.0f);
        vy_ = std::clamp(vy, -1.0f, 1.0f);
        omega_ = std::clamp(omega, -1.0f, 1.0f);
    }
    taskEXIT_CRITICAL();
    return running;
}

void Chassis::onRequestReset() noexcept {
    vx_ = 0.0f;
    vy_ = 0.0f;
    omega_ = 0.0f;
}

void Chassis::onPrepare() {
    for (auto* pid : pids_) pid->init();
}

Subsystem::Readiness Chassis::onCheckReadiness() {
    return all_online() ? Readiness::Ready : Readiness::Pending;
}

Subsystem::Readiness Chassis::onStandby(bool) {
    return onRun(true) ? Readiness::Ready : Readiness::Failed;
}

bool Chassis::onRun(bool enabled) {
    float current_ma[4]{};
    bool success = true;

    if (enabled) {
        float vx;
        float vy;
        float omega;
        taskENTER_CRITICAL();
        vx = vx_;
        vy = vy_;
        omega = omega_;
        for (std::size_t i = 0; i < 4; ++i) {
            measured_speed_[i] = motors_[i]->get_current_speed();
        }
        taskEXIT_CRITICAL();

        // 全向轮 X 布局解算，FR/FL/RL/RR，旋转四轮同号。
        const float v = -vx * kMaxSpeedRadS;
        const float s = vy * kMaxSpeedRadS;
        const float w = -omega * kMaxOmegaRadS;
        const float target[4] = {
             v - s - w,
             v + s - w,
            -v + s - w,
            -v - s - w,
        };
        for (std::size_t i = 0; i < 4; ++i) {
            current_ma[i] = pids_[i]->calculate(measured_speed_[i], target[i]);
            success &= std::isfinite(current_ma[i]);
        }
    }

    taskENTER_CRITICAL();
    if (enabled && success) {
        for (std::size_t i = 0; i < 4; ++i) {
            if (motors_[i]->set_output_command({.current_ma = current_ma[i]})) {
                motors_[i]->enable();
            } else {
                motors_[i]->disable();
                success = false;
            }
        }
    } else {
        for (auto* motor : motors_) motor->disable();
    }
    taskEXIT_CRITICAL();
    return success;
}

}  // namespace robot
