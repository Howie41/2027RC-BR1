#pragma once

#include "c620.hpp"
#include "hwt901b.hpp"
#include "pid_controller.hpp"
#include "pub_sub.hpp"
#include "remote.hpp"
#include "subsystem.hpp"

#include <cstddef>
#include <cstdint>

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

/// @brief 全向轮底盘：外环线/角速度 PID（世界系）+ 内环四轮转速 PID。
class Chassis final : public Subsystem {
public:
    static constexpr float kMaxLinearSpeed = 3.0f;         // 最大线速度 (m/s)
    static constexpr float kMaxAngularSpeed = 2 * 3.14159265f; // 最大角速度 (rad/s，360°/s)

    /// @brief remote 用于读取摇杆/禁用遥控；imu 提供 yaw_imu/omega_imu；velocity_command_msg 接收上位机指令；
    ///        fr/fl/rl/rr 依次为前右/前左/后左/后右电机。
    Chassis(remote::Remote& remote, imu::Data& imu,
            rtos::Subscriber<VelocityCommand, 4>& velocity_command_msg,
            C620Motor& fr, C620Motor& fl, C620Motor& rl, C620Motor& rr);

    /// @brief 以当前 imu 航向为世界系基准（yaw_bias = yaw_imu）。
    void resetYawBias() noexcept;

    /// @brief 设置全自动模式：自动模式禁用遥控，速度指令从话题队列获取。
    void setAutoMode(bool auto_mode);
    bool autoMode() const { return auto_mode_; }

private:
    [[nodiscard]] bool isOnline(FaultInfo& online_info) const override;
    void onRequestReset() noexcept override;
    void onPrepare() override;
    [[nodiscard]] Readiness onCheckReadiness() override;
    [[nodiscard]] Readiness onStandby(bool entering) override;
    [[nodiscard]] bool onRun(bool enabled) override;

    bool all_online() const;
    /// @brief 自动模式：从话题队列接收目标/实时速度（世界系）。
    void receiveVelocityCommand();
    /// @brief 手动模式：轮速正运动学 → 机体系实时线速度。
    void forwardKinematics();
    /// @brief 手动模式：摇杆 → 世界系目标速度（含无头映射、积分、限幅）。
    void manualControl(float dt);
    /// @brief 急停：清零目标速度与外环 PID 状态（内环目标归零）。
    void stopVelocity();
    /// @brief 机体系速度 → 世界系速度（按逻辑航向旋转）。
    void bodyToWorld(float body_vx, float body_vy, float& world_vx, float& world_vy) const;
    /// @brief 世界系速度 → 机体系速度。
    void worldToBody(float world_vx, float world_vy, float& body_vx, float& body_vy) const;
    /// @brief 计算控制周期时间步长 (s)。
    float tickDelta();

    // 底盘电机及其内环转速 PID。
    C620Motor* motors_[4]{};
    PidController fr_pid_;
    PidController fl_pid_;
    PidController rl_pid_;
    PidController rr_pid_;
    PidController* pids_[4]{};

    // 外部引用。
    remote::Remote& remote_;
    imu::Data& imu_;
    rtos::Subscriber<VelocityCommand, 4>& velocity_target_;

    // 轮速实测。
    float measured_speed_[4]{};

    // 外环线/角速度 PID（增量式）。
    PidController vx_pid_;
    PidController vy_pid_;
    PidController omega_pid_;

    // 世界系：基准航向 + 逻辑航向。
    float yaw_bias_{0.0f};
    float yaw_{0.0f};

    // 世界系目标/实时线速度（外环 PID 目标值/当前值）。
    float vx_world_{0.0f};
    float vy_world_{0.0f};
    float vx_world_cur_{0.0f};
    float vy_world_cur_{0.0f};

    // 机体系实时线速度（正运动学）。
    float vx_body_cur_{0.0f};
    float vy_body_cur_{0.0f};

    // 外环 PID 输出（世界系 → 机体系）。
    float output_vx_world_{0.0f};
    float output_vy_world_{0.0f};
    float output_vx_body_{0.0f};
    float output_vy_body_{0.0f};

    // 角速度：目标 / 实时 / 输出。
    float omega_target_{0.0f};
    float omega_cur_{0.0f};
    float output_omega_{0.0f};

    // 模式与计时。
    bool headless_mode_{false};
    bool auto_mode_{false};
    std::uint32_t last_tick_{0};
    bool first_tick_{true};
};

}  // namespace robot
