// =====================================================================
//  【落位说明，s34，2026-11-17】
//  本文件的阀测试序列由归档 sessions/s33-阀测试迁移/阀测试-test_running修改件-s33.cpp
//  迁入，落位位置：APP/robot/robot_hfsm/peer_root/test/test_running.cpp（HFSM Test::TestRunning）。
//  原 test_motor 速度测试序列已随移除（原序列可从 git 历史找回）。
//  前置条件（必须全部满足，否则进不到本状态或测试无意义）：
//    1. robot.hpp 中 catch_ground_cube 已注册进 subsystems 与 test_subsystems 数组
//       【截至 s34 尚未注册——见 robot.hpp注册修改说明-s33.md，迁移落位前需先改】；
//    2. 有周期任务调用 robot::catch_ground_cube.periodic()（已满足：APP/tasks/subsystems.cpp
//       的 catch_cube 任务，1kHz）；
//    3. 双电机在线（CAN2 有反馈）——TestStarting 会等 test_subsystems 全部
//       Standby/Running 才放行；电机未接时本测试不会运行（此时用 debug_task 临时标定，
//       电机在线后 debug_task 中的临时阀翻转段应删除）。
//  观察要点：
//    阶段1（进入后 20s）：阀1+阀2 同相、1s 翻转一次——两爪同时动作，
//                        用于确认阀号↔夹爪映射与得电电平方向（回填 CGC_VALVE_CLAMPED）；
//    阶段2（20s 之后）  ：阀1/阀2 反相、2s 翻转一次——两爪交替动作，验证独立控制。
//  setValve(id, open)：open=true = MOS 导通 = 阀得电（不等于"夹紧"，
//  夹紧方向由 catch_ground_cube.hpp 的 CGC_VALVE_CLAMPED 约定）。
// =====================================================================

#include "robot.hpp"
#include "states.hpp"
#include "rtos_utility.hpp"

namespace {
// 阀测试文件级状态（不进 states.hpp，避免多改一个文件；enter() 复位）
bool          valve_out{false};         // 当前输出：true=阀得电
std::uint32_t valve_last_toggle_ms{0};  // 上次翻转时刻
std::uint32_t valve_enter_ms{0};        // 进入 test_running 时刻
} // namespace

void robot::TestRunning::entryGuard(robot::GuardControl& control) {
    for (const auto* subsystem : test_subsystems) {
        if (!subsystem->isRunning()) {
            control.cancelPendingTransitions();
            return;
        }
    }
}

void robot::TestRunning::enter() {
    robot_context.faults.fill(Robot_Context::FaultReason::None);
    robot_context.fault_details.fill(FaultInfo{});

    // ---- 阀测试初始化：复位输出为不得电，记录起点 ----
    valve_out            = false;
    valve_enter_ms       = rtos::now_ms();
    valve_last_toggle_ms = valve_enter_ms;
    robot::catch_ground_cube.setValve(1, false);
    robot::catch_ground_cube.setValve(2, false);
}

void robot::TestRunning::update(robot::Control& control) {
    for (const auto* subsystem : test_subsystems) {
        if (subsystem->hasFailed()) {
            control.changeTo<Fault>();
            return;
        }
    }

    // ---- 阀测试序列（s33 迁入，s34 落位）----
    constexpr std::uint32_t kPhase1DurationMs = 20000; // 阶段1时长：同相标号
    constexpr std::uint32_t kPhase1PeriodMs   = 1000;  // 阶段1翻转周期
    constexpr std::uint32_t kPhase2PeriodMs   = 2000;  // 阶段2翻转周期（反相）
    const auto now = rtos::now_ms();
    const bool anti_phase =
        static_cast<std::uint32_t>(now - valve_enter_ms) >= kPhase1DurationMs;
    const auto period = anti_phase ? kPhase2PeriodMs : kPhase1PeriodMs;
    if (static_cast<std::uint32_t>(now - valve_last_toggle_ms) >= period) {
        valve_last_toggle_ms = now;
        valve_out            = !valve_out;
        robot::catch_ground_cube.setValve(1, valve_out);
        robot::catch_ground_cube.setValve(2, anti_phase ? !valve_out : valve_out);
    }
}

void robot::TestRunning::exit() {
    // 退出测试：两阀恢复到不得电，避免残留得电状态
    robot::catch_ground_cube.setValve(1, false);
    robot::catch_ground_cube.setValve(2, false);
}
