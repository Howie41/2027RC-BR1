#pragma once

#include "asserter.hpp"
#include "main.h"
#include "memory_map.h"
#include "robot_hfsm/config.hpp"
#include "rtos_utility.hpp"
#include "hfsm.hpp"
#include "robot_hfsm/states.hpp"
#include "usart.h"
#include "can_bus.hpp"
#include "bsp_dwt.hpp"
#include "rs485_bus.hpp"
#include "rtos_task.hpp"
#include "uart_port.hpp"
#include "c620.hpp"
#include "chassis.hpp"
#include "remote.hpp"
#include "hwt901b.hpp"
#include "odometry.hpp"
#include "point_tracker.hpp"
#include "headless_mode.hpp"
#include "subsystem.hpp"

#include <array>
#include <atomic>

//调试
#include "catch_ground_cube_subsystem.hpp"
#include "catch_ground_cube.hpp"


namespace robot {

inline bool initialized{false};

inline CanBus can1_bus{hfdcan1};
inline CanBus can2_bus{hfdcan2};
inline CanBus can3_bus{hfdcan3};

inline auto can_communication_entry{([] {
    while (true) {
        (void) robot::can1_bus.periodic();
        (void) robot::can2_bus.periodic();
        (void) robot::can3_bus.periodic();
        rtos::delay(1);
    }
})};
RAM_D1_BSS inline rtos::Task<
    "can_communication",
    can_communication_entry,
    2048 * 4,
    rtos::TaskPriority::Realtime
> can_communication_task;

// EB: 这里声明需要用到的外设和子系统（机构）
// 底盘（全向轮），CAN1 电机 1~4 = FR/FL/RL/RR。
inline C620 can1_c620{can1_bus};
inline C620Motor chassis_fr{can1_c620, 1};
inline C620Motor chassis_fl{can1_c620, 2};
inline C620Motor chassis_rl{can1_c620, 3};
inline C620Motor chassis_rr{can1_c620, 4};
inline Chassis chassis{chassis_fr, chassis_fl, chassis_rl, chassis_rr};

//调试
// EB: 抓地立方体机构（双3508同步升降 + 双气缸）【待确认 CAN 分配/电调编号】
inline C620 can2_c620{can2_bus};                          // CAN2 如已有 C620 则复用此行删除
inline C620Motor catch_motor_left{can2_c620, 5};          // 电调编号5 = 反馈ID 0x205
inline C620Motor catch_motor_right{can2_c620, 6};         // 电调编号6 = 反馈ID 0x206
inline CatchGroundCubeSubsystem catch_ground_cube{catch_motor_left, catch_motor_right};



// EB: 这里声明整车的外设和子系统（机构）
// 全部子系统的注册表，也是正式启动集合；Fault 按故障记录选择禁用对象。
inline const std::array<Subsystem*, 2> subsystems{&chassis, &catch_ground_cube};  // 【s34 注册】s33 修改 1（faults 数组按 size() 自动适配）


// EB: 这里声明你测试时需要用到的外设和子系统（机构）
// 必须是 subsystems 的子集；测试只等待并准备此集合，其余子系统保持待机。
// 【s34 无电机绕行，11-19】电机未接，暂空：TestStarting/entryGuard 的电机检测
// 只遍历本数组，清空即全链放行，阀测试（setValve 无守门）无需电机在线；
// 接 CAN2 双电机后恢复 {&catch_ground_cube}（s33 修改 2）。
inline const std::array<Subsystem*, 0> test_subsystems{};

// i-Bus 遥控接收机（UART4，115200 + DMA，仅接收）。
DMA_BUFFER_BSS inline UartPortDma<0, 64, 128> uart4_port{huart4, {}};
inline Remote remote{};

// IMU（HWT901B，UART8，9600 + DMA，仅接收）。
DMA_BUFFER_BSS inline UartPortDma<0, 64, 128> uart8_port{huart8, {}};
inline imu::Data imu_data{};
inline odometry::Estimator world_frame{};
inline tracking::PointTracker point_tracker{};
inline HeadlessMode headless{};

DMA_BUFFER_BSS inline UartPortDma<32, 32, 32 * 2> uart2_port{huart2, {}};
inline Rs485Bus rs4851_bus{uart2_port};

/**
 * @brief 初始化 DWT 并启动三路 CAN；CAN 设备已在构造时自动注册。
 * @note 由 MX_FREERTOS_Init() 在 rtos_resource_init() 完成后、调度器启动前调用；
 *       此时外设、队列、流缓冲区和任务已创建，业务任务尚未开始执行。
 * @note 不得调用任务延时或阻塞等待等依赖调度器运行的操作。
 * @note 初始化完成后设置 initialized，重复调用直接返回。
 */
inline void init() {
    if (initialized) return;

    // CYCCNT 使用 CPU 内核时钟，不能使用二分频
    // 后的 HCLK。
    Dwt::instance().init(SystemCoreClock / 1'000'000U);

    // 各总线依次建立映射、配置过滤器并启动，保留构造阶段注册的设备。
    for (auto *bus : {&can1_bus, &can2_bus, &can3_bus}) {
        const bool ready = bus->init();
        RUNTIME_ASSERT(ready);
    }

    initialized = true;
}

struct Robot_Context {
    enum class RecoveryMode { Manual, Test };
    enum class FaultReason { None, SubsystemFailed, RunRejected, Unspecified };

    RecoveryMode recovery_mode{RecoveryMode::Manual};
    // 进入 Fault 前记录请求拒绝等不会直接改变子系统状态的故障。
    Subsystem* pending_fault_source{nullptr};
    FaultReason pending_fault_reason{FaultReason::None};
    FaultInfo pending_fault_details{};
    std::array<FaultReason, subsystems.size()> faults{};
    // 与 faults / subsystems 下标对应；进入 Fault 时复制具体原因、文件名及行号。
    std::array<FaultInfo, subsystems.size()> fault_details{};
    // 预留给操作员/上位机恢复命令。唯一可由其他任务直接修改的上下文字段。
    // 每次进入 Fault 清零；显式置 true 后才允许离开 Fault，离开时消费。
    std::atomic<bool> recovery_requested{false};
};

using Tree = M::PeerRoot<
    M::Leaf< Disable >,
    M::Composite< Enable,
        M::Composite< Manual,
            M::Leaf< ManualStarting >,
            M::Leaf< UniversalManual >
        >,
        M::Composite< Auto,
            M::Leaf< AutoStarting >,
            M::Leaf< WaitUpperComputer >,
            M::Leaf< DevelopingAuto >,
            M::Leaf< Paused >
        >
    >,
    M::Composite< Test,
        M::Leaf< TestStarting >,
        M::Leaf< TestRunning >
    >,
    M::Leaf< Fault >,
    M::Leaf< Recovering >
>;

inline Robot_Context robot_context{};
using Machine = M::Instance<Tree>;
inline Machine robot_machine(robot_context, M::Initial<Disable>());
}; //namespace Robot
