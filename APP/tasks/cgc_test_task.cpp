// =====================================================================
//  【落位说明，s34，11-22】CGC（catch_ground_cube）bring-up 测试独立任务。
//  按用户要求与 robot_machine（HFSM Test 路径）和 debug_task（PidViewer）
//  隔离：本任务自跑阀测试序列，不依赖 HFSM 状态、不依赖电机在线。
//  历史：阀测试序列源自 s33 修改件，曾落位于 test_running.cpp
//  （11-17 迁入 → 11-19 迁回 → 11-20 按 chassis 风格重构 → 11-22 拆分为独立任务）。
//  观察要点（标定后回填）：
//    阶段1（启动后 20s）：阀1+阀2 同相、1s 翻转一次——两爪同时动作，
//                        确认阀号↔夹爪映射与得电电平方向（回填 CGC_VALVE_CLAMPED）；
//    阶段2（20s 之后）  ：阀1/阀2 反相、2s 翻转一次——两爪交替动作，验证独立控制。
//  setValve(id, open)：open=true = MOS 导通 = 阀得电（不等于"夹紧"，
//  夹紧方向由 catch_ground_cube.hpp 的 CGC_VALVE_CLAMPED 约定；电平模式已定稿）。
//  接 CAN2 双电机（ID 5/6）后：升降/抓取测试可扩展 ValveTest 或
//  走 HFSM Test 路径（归档 sessions/s33-阀测试迁移/阀测试-test_running修改件-s33.cpp）。
// =====================================================================

#include "robot.hpp"
#include "rtos_task.hpp"
#include "rtos_utility.hpp"

#include <cstdint>

namespace {

// 阀测试时序常量
constexpr std::uint32_t kPhase1DurationMs = 20000; // 阶段1时长：同相标号
constexpr std::uint32_t kPhase1PeriodMs   = 1000;  // 阶段1翻转周期
constexpr std::uint32_t kPhase2PeriodMs   = 2000;  // 阶段2翻转周期（反相）

/// @brief 双阀 bring-up 测试序列：状态进成员（参照 Chassis 结构）。
class ValveTest {
public:
    /// @brief 进入测试：两阀置不得电，记录起点
    void reset() {
        out_            = false;
        enter_ms_       = rtos::now_ms();
        last_toggle_ms_ = enter_ms_;
        robot::catch_ground_cube.setValve(1, false);
        robot::catch_ground_cube.setValve(2, false);
    }

    /// @brief 周期步进：阶段1 双阀同相 1s 翻转（标定）→ 阶段2 反相 2s 翻转（独立控制）
    void update() {
        const auto now = rtos::now_ms();
        const bool anti_phase =
            static_cast<std::uint32_t>(now - enter_ms_) >= kPhase1DurationMs;
        const auto period = anti_phase ? kPhase2PeriodMs : kPhase1PeriodMs;
        if (static_cast<std::uint32_t>(now - last_toggle_ms_) >= period) {
            last_toggle_ms_ = now;
            out_            = !out_;
            robot::catch_ground_cube.setValve(1, out_);
            robot::catch_ground_cube.setValve(2, anti_phase ? !out_ : out_);
        }
    }

private:
    bool          out_{false};        // 当前输出：true=阀得电
    std::uint32_t last_toggle_ms_{0};   // 上次翻转时刻
    std::uint32_t enter_ms_{0};       // 进入测试时刻
};

inline ValveTest valve_test{};   // 文件级实例（inline，C++17 单例语义）

} // namespace

// CGC 阀测试任务：10ms 节拍驱动 ValveTest，与 HFSM / debug_task 互不干扰。
rtos::Task<"cgc_test", [] {
    valve_test.reset();
    while (true) {
        valve_test.update();
        robot::chassis.periodic();
        rtos::delay(10);
    }
}, 4096, rtos::TaskPriority::Normal> cgc_test_task;
