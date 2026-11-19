#pragma once

#include "c620.hpp"
#include "pid_controller.hpp"
#include "subsystem.hpp"

#include <cstddef>

namespace robot {

/// @brief 全向轮底盘：四轮速度闭环，接收物理速度并解算各轮目标。
class Chassis final : public Subsystem {
public:
    static constexpr float kMaxLinearSpeed = 3.0f;         // 最大线速度 (m/s)
    static constexpr float kMaxAngularSpeed = 3.14159265f; // 最大角速度 (rad/s，180°/s)

    /// @brief fr/fl/rl/rr 依次为前右/前左/后左/后右电机。
    Chassis(C620Motor& fr, C620Motor& fl, C620Motor& rl, C620Motor& rr);

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

    /// @brief 设置全自动模式（始终使用无头模式，速度来自上位机）。
    void setAutoMode(bool auto_mode) { auto_mode_ = auto_mode; }
    bool autoMode() const { return auto_mode_; }

    /// @brief 设置基准航向（同步雷达坐标系，供世界系速度 → 机体系变换）。
    void setReferenceYaw(float yaw) { reference_yaw_ = yaw; }
    float referenceYaw() const { return reference_yaw_; }

    /// @brief 强制掉线（测试用）：置位后视为电机离线，用于演练 Fault 流程。
    void set_force_offline(bool offline) { force_offline_ = offline; }

private:
    [[nodiscard]] bool isOnline(FaultInfo& online_info) const override;
    void onRequestReset() noexcept override;
    void onPrepare() override;
    [[nodiscard]] Readiness onCheckReadiness() override;
    [[nodiscard]] Readiness onStandby(bool entering) override;
    [[nodiscard]] bool onRun(bool enabled) override;

    bool all_online() const;

    C620Motor* motors_[4]{};
    PidController fr_pid_;
    PidController fl_pid_;
    PidController rl_pid_;
    PidController rr_pid_;
    PidController* pids_[4]{};

    float vx_{0.0f};
    float vy_{0.0f};
    float omega_{0.0f};
    float measured_speed_[4]{};
    bool force_offline_{false};
    bool auto_mode_{false};
    float reference_yaw_{0.0f};
};

}  // namespace robot
