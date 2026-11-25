#include "chassis.hpp"

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

// 摇杆满杆时的加速度（增量）。
constexpr float kLinearAccel = 6.0f;   // 左摇杆满杆线加速度 (m/s²)
constexpr float kAngularAccel = 3.14159265f;  // 右摇杆满杆角加速度 (rad/s²)
}  // namespace

Chassis::Chassis(imu::Data& imu, rtos::Subscriber<VelocityCommand, 4>& velocity_command_msg,
                 C6x0Motor& fr, C6x0Motor& fl, C6x0Motor& rl, C6x0Motor& rr)
    : motors_{&fr, &fl, &rl, &rr},
      pids_{&fr_pid_, &fl_pid_, &rl_pid_, &rr_pid_},
      imu_{imu},
      velocity_target_{velocity_command_msg} {
}

bool Chassis::isOnline(FaultInfo& online_info) const {
    taskENTER_CRITICAL();
    bool online = true;
    for (const auto* motor : motors_) {
        online &= motor->is_online();
        if (!online) {
            if (motor == motors_[0]) {
                RECORD_FAULT(online_info, "Chassis front-right motor offline");
            } else if (motor == motors_[1]) {
                RECORD_FAULT(online_info, "Chassis front-left motor offline");
            } else if (motor == motors_[2]) {
                RECORD_FAULT(online_info, "Chassis rear-left motor offline");
            } else if (motor == motors_[3]) {
                RECORD_FAULT(online_info, "Chassis rear-right motor offline");
            }
        }
    }
    online &= imu_.is_online;
    if (!online) {
        RECORD_FAULT(online_info, "Chassis IMU offline");
    }
    taskEXIT_CRITICAL();
    return online;
}

void Chassis::resetYawBias() noexcept {
    yaw_bias_ = imu_.yaw_imu;
}

void Chassis::setAutoMode(bool auto_mode) {
    auto_mode_ = auto_mode;
    remote_enabled_ = !auto_mode;  // 全自动模式禁用遥控器。
    if (auto_mode) {
        resetYawBias();  // 以当前航向同步世界系基准。
    }
    // 切换模式时复位外环 PID。
    stopVelocity();
}

void Chassis::updateRemoteValue(bool online, float left_x, float left_y, float right_x, bool switch_a,
                                bool switch_d, bool channel10) {
    taskENTER_CRITICAL();
    remote_online_ = online;
    remote_left_x_ = left_x;
    remote_left_y_ = left_y;
    remote_right_x_ = right_x;
    remote_switch_a_ = switch_a;
    remote_switch_d_ = switch_d;
    // CH10 切换开关：上升沿翻转速度/加速度模式，进入速度模式时加速度归零。
    if (channel10 && !channel10_prev_) {
        velocity_mode_ = !velocity_mode_;
        if (velocity_mode_) {
            stopVelocity();
        }
    }
    channel10_prev_ = channel10;
    taskEXIT_CRITICAL();
}

void Chassis::onRequestReset() noexcept {
    resetYawBias();
    taskENTER_CRITICAL();
    headless_mode_ = false;
    taskEXIT_CRITICAL();
    stopVelocity();
}

void Chassis::receiveVelocityCommand() {
    const auto msg = velocity_target_.receive(0);  // 非阻塞取最新指令。
    if (msg) {
        vx_world_ = msg->target_vx;
        vy_world_ = msg->target_vy;
        omega_target_ = msg->target_omega;
        vx_world_cur_ = msg->actual_vx;
        vy_world_cur_ = msg->actual_vy;
        omega_cur_ = msg->actual_omega;
    }
}

void Chassis::forwardKinematics() {
    // 正运动学：轮速 → 机体系线速度（假设不打滑）。
    const float u_fr = measured_speed_[0];
    const float u_fl = measured_speed_[1];
    const float u_rl = measured_speed_[2];
    const float u_rr = measured_speed_[3];
    const float k = kWheelRadius / (2.0f * kSqrt2);
    vx_body_cur_ = (u_fr + u_fl - u_rl - u_rr) * k;
    vy_body_cur_ = (u_fl + u_rl - u_fr - u_rr) * k;
}

void Chassis::bodyToWorld(float body_vx, float body_vy, float& world_vx, float& world_vy) const {
    const float c = std::cos(yaw_);
    const float s = std::sin(yaw_);
    world_vx = c * body_vx - s * body_vy;
    world_vy = s * body_vx + c * body_vy;
}

void Chassis::worldToBody(float world_vx, float world_vy, float& body_vx, float& body_vy) const {
    const float c = std::cos(yaw_);
    const float s = std::sin(yaw_);
    body_vx = c * world_vx + s * world_vy;
    body_vy = -s * world_vx + c * world_vy;
}

void Chassis::stopVelocity() {
    vx_world_ = 0.0f;
    vy_world_ = 0.0f;
    vx_body_ = 0.0f;
    vy_body_ = 0.0f;
    omega_target_ = 0.0f;
    vx_pid_.init();
    vy_pid_.init();
    omega_pid_.init();
}

void Chassis::manualControl(float dt) {
    if (!remote_online_) {
        stopVelocity();
        return;
    }

    // 左摇杆（机体系）；右摇杆（角速度，仅在 omega 映射处取反一次）。
    const float stick_x = remote_left_x_;
    const float stick_y = remote_left_y_;
    const float stick_omega = -remote_right_x_;

    // 无头模式（SwA）：由有头转无头时重定世界系基准。
    const bool headless = remote_switch_a_;
    if (headless && !headless_mode_) {
        resetYawBias();
    }
    headless_mode_ = headless;

    // SwD（通道8，0 基下标 7）拨到 2000 = 速度强行归零（清空外环 PID）。
    if (remote_switch_d_) {
        stopVelocity();
        return;
    }

    // 角速度：速度模式直接设定，加速度模式积分（不分头）。
    if (velocity_mode_) {
        omega_target_ = stick_omega * kMaxAngularSpeed;
    } else {
        omega_target_ = std::clamp(omega_target_ + stick_omega * kAngularAccel * dt,
                                   -kMaxAngularSpeed, kMaxAngularSpeed);
    }

    if (headless_mode_) {
        // 无头：摇杆直接映射到世界系。
        if (velocity_mode_) {
            vx_world_ = stick_x * kMaxLinearSpeed;
            vy_world_ = stick_y * kMaxLinearSpeed;
        } else {
            vx_world_ = std::clamp(vx_world_ + stick_x * kLinearAccel * dt, -kMaxLinearSpeed,
                                   kMaxLinearSpeed);
            vy_world_ = std::clamp(vy_world_ + stick_y * kLinearAccel * dt, -kMaxLinearSpeed,
                                   kMaxLinearSpeed);
        }
    } else {
        // 有头：机体系目标速度，每周期用当前航向转世界系（速度天然跟车头）。
        if (velocity_mode_) {
            vx_body_ = stick_x * kMaxLinearSpeed;
            vy_body_ = stick_y * kMaxLinearSpeed;
        } else {
            vx_body_ = std::clamp(vx_body_ + stick_x * kLinearAccel * dt, -kMaxLinearSpeed,
                                  kMaxLinearSpeed);
            vy_body_ = std::clamp(vy_body_ + stick_y * kLinearAccel * dt, -kMaxLinearSpeed,
                                  kMaxLinearSpeed);
        }
        bodyToWorld(vx_body_, vy_body_, vx_world_, vy_world_);
    }
}

float Chassis::tickDelta() {
    const auto now = rtos::now_tick();
    const float dt = first_tick_ ? 0.0f : static_cast<float>(now - last_tick_) / 1000.0f;
    first_tick_ = false;
    last_tick_ = now;
    return dt;
}

void Chassis::onPrepare() {
    for (auto* pid : pids_) pid->init();
    stopVelocity();
    tickDelta();  // 初始化计时器。
}

Subsystem::Readiness Chassis::onCheckReadiness() {
    return Readiness::Ready;
}

Subsystem::Readiness Chassis::onStandby(bool) {
    stopVelocity();
    return Readiness::Ready;
}

bool Chassis::onRun(bool enabled) {
    float current_ma[4]{};
    bool success = true;

    if (enabled) {
        const float dt = tickDelta();

        // 逻辑航向 = imu 航向 - 基准。
        yaw_ = imu_.yaw_imu - yaw_bias_;

        // 读取轮速（内环反馈 + 手动正运动学）。
        taskENTER_CRITICAL();
        for (std::size_t i = 0; i < 4; ++i) {
            measured_speed_[i] = motors_[i]->get_current_speed();
        }
        taskEXIT_CRITICAL();

        if (auto_mode_) {
            // 全自动模式：目标/实时速度均从话题队列获取（世界系）。
            receiveVelocityCommand();
        } else if (remote_enabled_) {
            // 手动模式：摇杆积分 → 目标；正运动学 → 实时。
            manualControl(dt);
            forwardKinematics();
            bodyToWorld(vx_body_cur_, vy_body_cur_, vx_world_cur_, vy_world_cur_);
            omega_cur_ = imu_.omega_imu;
        }

        // 外环 PID（增量式，世界系）。
        output_vx_world_ = vx_pid_.calculate(vx_world_cur_, vx_world_);
        output_vy_world_ = vy_pid_.calculate(vy_world_cur_, vy_world_);
        output_omega_ = omega_pid_.calculate(omega_cur_, omega_target_);

        // 世界系 → 机体系。
        worldToBody(output_vx_world_, output_vy_world_, output_vx_body_, output_vy_body_);

        // 逆运动学：机体系速度 → 各轮转速。
        const float v = output_vx_body_ * kLinearToWheel;    // 右 (rad/s)
        const float s = output_vy_body_ * kLinearToWheel;    // 前 (rad/s)
        const float w = output_omega_ * kRotationToWheel;    // 旋转 (rad/s)
        const float target[4] = {
            v - s - w,   // FR：(a,b)，方向(-1,1)
            v + s - w,   // FL：(-a,b)，方向(-1,-1)
            -v + s - w,  // RL：(-a,-b)，方向(1,-1)
            -v - s - w,  // RR：(a,-b)，方向(1,1)
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
