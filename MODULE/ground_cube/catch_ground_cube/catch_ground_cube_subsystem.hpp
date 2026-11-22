/**
 * @file catch_ground_cube_subsystem.hpp
 * @brief 抓地立方体机构的 Subsystem 适配层 —— 把核心类挂进框架生命周期
 * @attention
 *  1. 本层只做生命周期映射和业务接口守门，不含控制逻辑（逻辑全在 CatchGroundCube）。
 *  2. 升降类业务接口用 acceptsCommands() 守门（仅"请求运行且已运行"时写入目标）；
 *     气缸接口不设守门——调试/待机时也需要能手动捅阀。
 *  3. s32 新增：内部持有 PutGroundCube 放置流程层，onRun 在 core_.update() 后顺带
 *     步进 put_ground_cube_.update()，robot.hpp / subsystems.cpp 不需要任何新增接线。
 *  4. 掉线由基类统一处理：就绪后掉线自动 Failed 并 stopOutput()，
 *     故障恢复请调 requestPrepare()（一般来自 HFSM，调试可在 debug_task 直接发请求）。
 */

#pragma once

#include "subsystem.hpp"
#include "c620.hpp"
#include "catch_ground_cube.hpp"
#include "put_ground_cube.hpp"

/// 状态快照（调试读取用，字段只读拷贝）
struct CubeStatus {
  float height_mm;         // 当前高度(mm)
  float target_height_mm;  // 当前目标高度(mm)
  float sync_error_mm;     // 同步误差(mm)
  bool  sync_fault;        // true=同步误差超限封波中
  bool  valve1_open;       // 气缸1 记账状态（开/关）
  bool  valve2_open;       // 气缸2 记账状态（开/关）
  uint8_t grab_state;      // 抓取流程状态（0=IDLE 1=CLOSING 2=LIFTING 3=DONE，见 GrabState）
};

class CatchGroundCubeSubsystem final : public Subsystem {
public:
  explicit CatchGroundCubeSubsystem(C620Motor& left, C620Motor& right);

  // ---- 业务接口（外部唯一入口；未运行时升降类调用返回 false 且不生效） ----
  [[nodiscard]] bool moveToHeight(float target_mm);  // 定高走位；false=未运行或超行程已钳制
  [[nodiscard]] bool jog(float delta_mm);            // 点动增量(mm)，正升负降
  void setValve(int valve_id, bool open);            // 气缸开关，id=1/2（不设守门）
  void valvePulse(int valve_id);                     // 气缸强制翻转/发脉冲
  void resetValveState(bool valve1_open, bool valve2_open);  // 上电校准气缸记账
  [[nodiscard]] CubeStatus getStatus() const;        // 状态快照

  [[nodiscard]] CatchGroundCube::Config& config() noexcept { return core_.config; }  // 参数入口（接线时改配置用）

  // ---- 放置流程（s32 新增：本体为 put_ground_cube_，硬件仍归 core_ 独占） ----
  // 功能1 贴地放两个   = startPut(地面高度, PUT_RELEASE_BOTH)
  // 功能2 抬升后放两个 = startPut(目标高度, PUT_RELEASE_BOTH)
  // 功能3 抬升后放一个 = startPut(目标高度, PUT_RELEASE_VALVE1_ONLY / _VALVE2_ONLY)
  [[nodiscard]] bool startPut(float height_mm, PutRelease release);  // false=未运行/同步故障/参数非法
  [[nodiscard]] bool startPut(const PutStep* steps, size_t count);   // 多步队列（预留扩展）
  void  cancelPut();                  // 中断放置流程（随时可调用）
  PutGroundCube& put() noexcept { return put_ground_cube_; }  // 流程本体：查状态/改流程参数

  // ---- 抓取流程（s32 新增：合爪→锁定→自动抬升防摩擦；非阻塞） ----
  [[nodiscard]] bool startGrab(CatchGroundCube::GrabSelection sel = CatchGroundCube::GrabSelection::BOTH);
  void  cancelGrab();

protected:
  [[nodiscard]] bool isOnline(FaultInfo& online_info) const override;  // 双电机在线检查（临界区内读反馈）【s32/s34 修订：与框架基类签名对齐】
  [[nodiscard]] bool motorsOnline() const;             // 双电机在线检查（isOnline/onCheckReadiness 共用）
  void onRequestReset() noexcept override;             // 冻结升降目标（临界区内，禁阻塞/硬件操作）
  void onPrepare() override;                           // 每次上线一次：PID 配置 + 阀 IO 初始态
  [[nodiscard]] Readiness onCheckReadiness() override; // 在线即就绪（编码器首帧自初始化）
  // onStandby 用基类默认（= onRun(true)，保持当前位置待机）
  [[nodiscard]] bool onRun(bool enabled) override;     // true=跑核心 update()；false=关动力输出

private:
  CatchGroundCube core_;
  PutGroundCube   put_ground_cube_;   // 放置流程层（s32）：构造时绑定 core_，不拥有硬件
};
