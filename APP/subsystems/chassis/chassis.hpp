#pragma once

#include "c620.hpp"
#include "pid_controller.hpp"
#include "pub_sub.hpp"
#include "remote.hpp"
#include "subsystem.hpp"
#include "yaw.hpp"

#include <cstddef>

namespace robot {

/// @brief 上位机下发的速度指令（目标 + 实时，世界系）。
struct VelocityCommand {
    float target_vx{0.0f};    // 目标线速度 x (m/s)
    float target_vy{0.0f};    // 目标线速度 y (m/s)
    float target_omega{0.0f}; // 目标角速度 (rad/s)
    float actual_vx{0.0f};    // 实时线速度 x (m/s，雷达测量)
    float actual_vy{0.0f};    // 实时线速度 y (m/s，雷达测量)
    float actual_omega{0.0f}; // 实时角速度 (rad/s，雷达测量)
};

/// @brief 全向轮底盘：四轮速度闭环，接收物理速度并解算各轮目标。
class Chassis final : public Subsystem {
public:
    static constexpr float kMaxLinearSpeed = 3.0f;         // 最大线速度 (m/s)
    static constexpr float kMaxAngularSpeed = 2 * 3.14159265f; // 最大角速度 (rad/s，360°/s)

    /// @brief remote 用于全自动模式禁用遥控器；velocity_command_msg 用于接收上位机速度指令；
    ///        fr/fl/rl/rr 依次为前右/前左/后左/后右电机。
    Chassis(remote::Remote& remote, rtos::Subscriber<VelocityCommand, 4>& velocity_command_msg,
            C620Motor& fr, C620Motor& fl, C620Motor& rl, C620Motor& rr);

    /// @brief 设置运动目标：vx 向右(+x, m/s) / vy 向前(+y, m/s) / omega 逆时针(rad/s)。
    /// @return 仅在 Running 且目标有限时接受并返回 true。
    [[nodiscard]] bool setVelocity(float vx, float vy, float omega);

    /// @brief 积分速度增量（加速度 × dt），限幅到最大速度；仅在 Running 时生效。
    void integrateVelocity(float ax, float ay, float alpha, float dt);

    /// @brief 积分角速度（角加速度 × dt），限幅；仅在 Running 时生效。
    void integrateOmega(float alpha, float dt);

    /// @brief 设置线速度目标（机体系，直接，限幅）；仅在 Running 时生效。
    void setLinearVelocity(float vx, float vy);

    /// @brief 清零速度目标。
    void resetVelocity() noexcept;

    /// @brief 用 IMU 数据刷新世界航向（imu_task 每帧调用）。
    void updateYaw(const imu::Data& data, std::uint32_t now_ms);

    /// @brief 复位世界航向基准：以当前方向为基准。
    void resetYaw() noexcept;

    /// @brief 设置全自动模式：进入时禁用遥控、复位世界航向基准（同步雷达）、启用线速度 PID；
    ///        退出时恢复遥控、复位 PID。
    void setAutoMode(bool auto_mode);
    bool autoMode() const { return auto_mode_; }

    /// @brief 设置无头模式：拨到无头瞬间锁定当前航向为前方，每周期刷新相对航向。
    void setHeadless(bool enabled);

    /// @brief 积分锁定系（世界系）线速度，避免速度随车身旋转。
    void integrateWorldVelocity(float ax, float ay, float dt);

    /// @brief 锁定系线速度 → 机体系线速度。
    void bodyVelocity(float& vx, float& vy) const;

    /// @brief 清零锁定系线速度。
    void resetHeadlessVelocity() noexcept;

    /// @brief 复位无头模式：回到有头，偏置与锁定系速度清零。
    void resetHeadless() noexcept;

private:
    [[nodiscard]] bool isOnline(FaultInfo& online_info) const override;
    void onRequestReset() noexcept override;
    void onPrepare() override;
    [[nodiscard]] Readiness onCheckReadiness() override;
    [[nodiscard]] Readiness onStandby(bool entering) override;
    [[nodiscard]] bool onRun(bool enabled) override;

    bool all_online() const;

    /// @brief 自动模式：从话题队列接收速度指令（目标 + 实时）。
    void receiveVelocityCommand();
    /// @brief 手动模式：轮速正运动学 → 实际速度（假设不打滑）。
    void updateActualVelocity();
    /// @brief 手动模式：摇杆加速度 → 积分速度（含无头解算、掉线/清零）。
    void manualControl(float dt);
    /// @brief 计算控制周期时间步长 (s)。
    float tickDelta();

    // 底盘电机及其pid
    C620Motor* motors_[4]{};
    PidController fr_pid_;
    PidController fl_pid_;
    PidController rl_pid_;
    PidController rr_pid_;
    PidController* pids_[4]{};
    // 机体系速度（右x+、前y+、逆时针+，m/s / rad/s）。
    float vx_{0.0f};
    float vy_{0.0f};
    float omega_{0.0f};
    float measured_speed_[4]{};
    // 世界系 目标速度（锁定系，无头模式，m/s）。
    float target_vx_{0.0f};
    float target_vy_{0.0f};
    float target_omega_{0.0f};
    // 世界系 实际速度（锁定系，无头模式，m/s）。
    float actual_vx_{0.0f};
    float actual_vy_{0.0f};
    float actual_omega_{0.0f};
    // 外环线速度 PID（增量式）。
    PidController vx_pid_;
    PidController vy_pid_;
    PidController omega_pid_;
    // 无头模式：锁定系（世界系）速度 + 相对航向。
    bool headless_active_{false};
    float headless_bias_{0.0f};
    float headless_yaw_{0.0f};
    float world_vx_{0.0f};
    float world_vy_{0.0f};
    // 其他
    std::uint32_t last_tick_{0};  // 上次控制周期 tick（ms），用于计算 dt。
    bool first_tick_{true};
    remote::Remote& remote_;
    imu::YawEstimator world_frame_{};
    bool auto_mode_{false};
    rtos::Subscriber<VelocityCommand, 4>& velocity_target_;
};

}  // namespace robot
