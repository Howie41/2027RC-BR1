#include "chassis.hpp"

#include "robot.hpp"
#include "rtos_utility.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace robot {

namespace {
// 底盘几何尺寸（米）
constexpr float kWheelRadius = 0.06475f;   // 轮子半径
constexpr float kHalfLength = 0.273125f;   // 半长（前-后，546.25/2）
constexpr float kHalfWidth = 0.223065f;    // 半宽（左-右，446.13/2）

// 物理速度 → 车轮转速 (rad/s)：45° X 布局含 √2 因子。
constexpr float kSqrt2 = 1.41421356f;
constexpr float kLinearToWheel = 1.0f / (kSqrt2 * kWheelRadius);
constexpr float kRotationToWheel = (kHalfLength + kHalfWidth) / (kSqrt2 * kWheelRadius);

// 旧框架 PID 增益按 deg/s 标定，换算到 rad/s
constexpr float kDegPerRad = 57.29578f;
}  // namespace

Chassis::Chassis(remote::Remote& remote, odometry::Estimator& odometry,
                 rtos::Subscriber<VelocityCommand, 4>& velocity_target,
                 C620Motor& fr, C620Motor& fl, C620Motor& rl, C620Motor& rr)
    : remote_{remote},
      odometry_{odometry},
      velocity_target_{velocity_target},
      motors_{&fr, &fl, &rl, &rr},
      fr_pid_("chassis_fr_speed"),
      fl_pid_("chassis_fl_speed"),
      rl_pid_("chassis_rl_speed"),
      rr_pid_("chassis_rr_speed"),
      pids_{&fr_pid_, &fl_pid_, &rl_pid_, &rr_pid_} {
    const PidController::Config config{
        .kp = 20.0f * kDegPerRad,   // 旧 20 mA/(deg/s)，换算到 rad/s
        .ki = 1.2f * kDegPerRad,    // 旧 2 mA/(deg/s)，换算到 rad/s
        .kd = 0.0f,
        .max_out = 20000.0f,
        .integral_limit = 5000.0f,
        .improvement = {1U << PidController::integral_limit},
    };
    for (auto* pid : pids_) pid->configure(config);

    // 外环线速度 PID（增量式，输出命令速度）。
    const PidController::Config outer_linear{
        .kp = 1.0f,
        .ki = 0.5f,
        .kd = 0.0f,
        .max_out = kMaxLinearSpeed,
        .improvement = {1U << PidController::increment_of_out},
    };
    vx_pid_.configure(outer_linear);
    vy_pid_.configure(outer_linear);

    const PidController::Config outer_angular{
        .kp = 1.0f,
        .ki = 0.5f,
        .kd = 0.0f,
        .max_out = kMaxAngularSpeed,
        .improvement = {1U << PidController::increment_of_out},
    };
    omega_pid_.configure(outer_angular);
}

bool Chassis::all_online() const {
    if (force_offline_) return false;
    taskENTER_CRITICAL();
    bool online = true;
    for (const auto* motor : motors_) online &= motor->is_online();
    taskEXIT_CRITICAL();
    return online;
}

bool Chassis::isOnline(FaultInfo& online_info) const {
    const bool online = all_online();
    if (!online) {
        RECORD_FAULT(online_info, "Chassis motor offline");
    }
    return online;
}

bool Chassis::setVelocity(float vx, float vy, float omega) {
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(omega)) return false;

    taskENTER_CRITICAL();
    const bool running = acceptsCommands();
    if (running) {
        vx_ = std::clamp(vx, -kMaxLinearSpeed, kMaxLinearSpeed);
        vy_ = std::clamp(vy, -kMaxLinearSpeed, kMaxLinearSpeed);
        omega_ = std::clamp(omega, -kMaxAngularSpeed, kMaxAngularSpeed);
    }
    taskEXIT_CRITICAL();
    return running;
}

void Chassis::integrateVelocity(float ax, float ay, float alpha, float dt) {
    taskENTER_CRITICAL();
    if (acceptsCommands()) {
        vx_ = std::clamp(vx_ + ax * dt, -kMaxLinearSpeed, kMaxLinearSpeed);
        vy_ = std::clamp(vy_ + ay * dt, -kMaxLinearSpeed, kMaxLinearSpeed);
        omega_ = std::clamp(omega_ + alpha * dt, -kMaxAngularSpeed, kMaxAngularSpeed);
    }
    taskEXIT_CRITICAL();
}

void Chassis::integrateOmega(float alpha, float dt) {
    taskENTER_CRITICAL();
    if (acceptsCommands()) {
        omega_ = std::clamp(omega_ + alpha * dt, -kMaxAngularSpeed, kMaxAngularSpeed);
    }
    taskEXIT_CRITICAL();
}

void Chassis::setLinearVelocity(float vx, float vy) {
    taskENTER_CRITICAL();
    if (acceptsCommands()) {
        vx_ = std::clamp(vx, -kMaxLinearSpeed, kMaxLinearSpeed);
        vy_ = std::clamp(vy, -kMaxLinearSpeed, kMaxLinearSpeed);
    }
    taskEXIT_CRITICAL();
}

void Chassis::resetVelocity() noexcept {
    taskENTER_CRITICAL();
    vx_ = 0.0f;
    vy_ = 0.0f;
    omega_ = 0.0f;
    taskEXIT_CRITICAL();
}

void Chassis::setAutoMode(bool auto_mode) {
    auto_mode_ = auto_mode;
    remote_.set_enabled(!auto_mode);  // 全自动模式禁用遥控器。
    if (auto_mode) {
        odometry_.reset();  // 同步基准航向（雷达坐标系），复用里程计 yaw 偏置。
    }
    // 切换模式时复位外环线速度 PID。
    vx_pid_.init();
    vy_pid_.init();
    omega_pid_.init();
}

void Chassis::onRequestReset() noexcept {
    resetVelocity();
}

void Chassis::receiveVelocityCommand() {
    const auto msg = velocity_target_.receive(0);  // 非阻塞取最新指令。
    if (msg) {
        target_vx_ = msg->target_vx;
        target_vy_ = msg->target_vy;
        target_omega_ = msg->target_omega;
        actual_vx_ = msg->actual_vx;
        actual_vy_ = msg->actual_vy;
        actual_omega_ = msg->actual_omega;
    }
}

void Chassis::onPrepare() {
    for (auto* pid : pids_) pid->init();
    vx_pid_.init();
    vy_pid_.init();
    omega_pid_.init();
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

        if (auto_mode_) {
            // 全自动模式：接收速度指令（目标 + 实时）→ 外环 PID → 命令速度。
            receiveVelocityCommand();
            vx = vx_pid_.calculate(actual_vx_, target_vx_);
            vy = vy_pid_.calculate(actual_vy_, target_vy_);
            omega = omega_pid_.calculate(actual_omega_, target_omega_);
        }

        // 世界坐标系：vx 向右(+x)、vy 向前(+y)、omega 逆时针(+)；X 布局 FR/FL/RL/RR。
        const float v = vx * kLinearToWheel;    // 右 (rad/s)
        const float s = vy * kLinearToWheel;    // 前 (rad/s)
        const float w = omega * kRotationToWheel;  // 旋转 (rad/s)
        const float target[4] = {
            -v + s + w,   // FR：(a,b)，方向(-1,1)
            -v - s + w,   // FL：(-a,b)，方向(-1,-1)
             v - s + w,   // RL：(-a,-b)，方向(1,-1)
             v + s + w,   // RR：(a,-b)，方向(1,1)
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
