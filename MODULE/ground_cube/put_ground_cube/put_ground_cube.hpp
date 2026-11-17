/**
 * @file put_ground_cube.hpp
 * @brief 放地立方体机构（新框架版）—— 基于 CatchGroundCube 核心的非阻塞放置流程层
 * @attention
 *  1. 本类不拥有任何硬件：电机/阀全部通过 CatchGroundCube 已有接口操作，
 *     同一套机构（双3508 + 双气缸）抓取与放置共用，不建第二个 Subsystem。
 *  2. 非阻塞状态机：start() 立即返回，update() 由 CatchGroundCubeSubsystem::onRun
 *     在 1kHz 下顺带步进，放置期间 chassis/HFSM 该干嘛干嘛。
 *  3. 功能映射（对应 s32 需求）：
 *     功能1 贴地放两个   = start(地面高度, PUT_RELEASE_BOTH)
 *     功能2 抬升后放两个 = start(目标高度, PUT_RELEASE_BOTH)   ← 与功能1 同一入口，仅高度不同
 *     功能3 接力放一个   = start(目标高度, PUT_RELEASE_VALVE1_ONLY / _VALVE2_ONLY)
 *            场景：对手塔基已就位但尚未放中段时，我方抢先放一个中段上去。
 *            合规依据：L1 公共区/L2 允许两队共建塔，中段得分归放置方（规则 8.1.3/8.4）。
 *     功能4 移除对方错误项（实验性功能，不一定能用，使用前必须对照规则原文确认）：
 *            夹取高空方块不是本层的职责，用核心层接口组合实现：
 *            CatchGroundCube::startGrab(单爪) 在高度夹住对方未正确放置的方块 →
 *            moveToHeightMm 抬升 → 底盘转移 → 本层 start 释放。
 *            【规则风险】规则 7.2：故意移除/移动/干扰"已正确放置"的对方塔基/中段 =
 *            取消资格。规则 V2 3.5.5 仅允许移动"未正确放置"的，判据以裁判认定为准。
 *  4. 可扩展性：start(steps, count) 接受最多 4 步的放置队列（如"先放一个到h1、
 *     再放一个到h2"），功能1/2/3 都是单步队列的特例。
 * @versioninfo s32: 新建。阀释放电平取 CGC_VALVE_CLAMPED 的反面（见 catch_ground_cube.hpp）。
 *             s32修订: 功能3 语义更正为接力（原误写截断）；功能4 以注释形式登记为实验性。
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "catch_ground_cube.hpp"

/// 释放选择：两个都放 / 只放阀1对应的爪 / 只放阀2对应的爪
/// 【待确认】阀号与上/下爪的对应关系，装车后按实际机械改调用处即可
enum PutRelease : uint8_t {
  PUT_RELEASE_BOTH = 0,
  PUT_RELEASE_VALVE1_ONLY,
  PUT_RELEASE_VALVE2_ONLY,
};

/// 单个放置步骤（队列元素）
struct PutStep {
  float       height_mm;  // 本步释放高度(mm，行程坐标；贴地放置传地面高度即 ground_clearance_mm)
  PutRelease  release;    // 本步释放哪些爪
};

class PutGroundCube {
public:
  // ---------- 可调参数（运行时可改，下次 start 生效；进行中修改立即生效） ----------
  struct Config {
    float    arrive_tolerance_mm = 2.0f;  // 到位判定容差(mm)【待填】
    uint16_t stable_time_ms      = 100;   // 到位后稳定保持时间(ms)，消抖【待填】
    uint16_t release_settle_ms   = 150;   // 开爪后等待方块脱离(ms)【待填】
    uint32_t lift_timeout_ms     = 3000;  // 单步升降超时(ms)，超时→FAILED【待填】
  } config;

  /// 流程状态机（外部查询用）
  enum class State : uint8_t {
    IDLE = 0,    // 空闲（或 cancel 后）
    LIFTING,     // 正在升降到本步高度
    STABLE,      // 已到高度，稳定保持中
    RELEASING,   // 已开爪，等待方块脱离
    DONE,        // 全部步骤完成（结果态，reset/start 后离开）
    FAILED,      // 失败：同步故障或升降超时（结果态）
  };

  explicit PutGroundCube(CatchGroundCube& mech) : mech_(mech) {}

  // ---------- 流程控制 ----------
  bool  start(float height_mm, PutRelease release);  // 单步放置（功能1/2/3 统一入口）；false=参数非法或同步故障
  bool  start(const PutStep* steps, size_t count);   // 多步队列（<=4 步）；false=参数非法或同步故障
  void  cancel();   // 中断流程：保持当前高度与阀状态，回到 IDLE（随时可调用）
  void  reset();    // 清除 DONE/FAILED 结果态，回到 IDLE
  void  update();   // 1kHz 步进状态机（仅由 Subsystem 适配层调用）

  // ---------- 状态查询 ----------
  State  state() const   { return state_; }
  bool   busy() const    { return state_ == State::LIFTING || state_ == State::STABLE || state_ == State::RELEASING; }
  bool   isDone() const  { return state_ == State::DONE; }
  bool   isFailed() const{ return state_ == State::FAILED; }
  size_t stepIndex() const { return step_index_; }  // 当前执行到第几步（0 起）

private:
  static constexpr size_t kMaxSteps = 4;  // 固定容量小队列，不用堆分配

  CatchGroundCube& mech_;              // 机构核心（引用，不拥有）
  State    state_           = State::IDLE;
  PutStep  steps_[kMaxSteps];
  size_t   step_count_      = 0;
  size_t   step_index_      = 0;
  uint16_t stable_timer_ms_ = 0;   // STABLE 倒计时
  uint16_t release_timer_ms_= 0;   // RELEASING 倒计时
  uint32_t step_timer_ms_   = 0;   // LIFTING 超时计时
};
