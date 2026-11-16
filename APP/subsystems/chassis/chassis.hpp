#pragma once

#include "c620.hpp"
#include "pid_controller.hpp"
#include "subsystem.hpp"

#include <cstddef>

namespace robot {

/// @brief 全向轮底盘：四轮速度闭环，接收归一化速度并解算各轮目标。
class Chassis final : public Subsystem {
public:
    /// @brief fr/fl/rl/rr 依次为前右/前左/后左/后右电机。
    Chassis(C620Motor& fr, C620Motor& fl, C620Motor& rl, C620Motor& rr);

    /// @brief 设置归一化运动目标：vx 向右(+x) / vy 向前(+y) / omega 逆时针，各 -1.0~1.0。
    /// @return 仅在 Running 且目标有限时接受并返回 true。
    [[nodiscard]] bool setVelocity(float vx, float vy, float omega);

    /// @brief 强制掉线（测试用）：置位后视为电机离线，用于演练 Fault 流程。
    void set_force_offline(bool offline) { force_offline_ = offline; }

private:
    [[nodiscard]] bool isOnline() const override;
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
};

}  // namespace robot
