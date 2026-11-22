/**
 * @file catch_ground_cube_subsystem.cpp
 * @brief 抓地立方体机构 Subsystem 适配层实现
 */

#include "catch_ground_cube_subsystem.hpp"

#include "rtos_utility.hpp"

#include <cmath>

CatchGroundCubeSubsystem::CatchGroundCubeSubsystem(C620Motor& left, C620Motor& right)
    : core_(left, right),
      put_ground_cube_(core_) {}   // 放置流程层绑定核心（引用，构造顺序：core_ 先声明先构造）

// =====================================================================
//  业务接口：升降类用 acceptsCommands() 守门（检查与写入在同一临界区）
// =====================================================================
bool CatchGroundCubeSubsystem::moveToHeight(float target_mm) {
  if (!std::isfinite(target_mm)) return false;
  bool accepted = false;
  taskENTER_CRITICAL();
  if (acceptsCommands()) {
    accepted = core_.moveToHeightMm(target_mm);   // false=超行程，已钳制执行
  }
  taskEXIT_CRITICAL();
  return accepted;
}

bool CatchGroundCubeSubsystem::jog(float delta_mm) {
  if (!std::isfinite(delta_mm) || delta_mm == 0.0f) return false;
  bool accepted = false;
  taskENTER_CRITICAL();
  if (acceptsCommands()) {
    core_.addLiftDelta(delta_mm);
    accepted = true;
  }
  taskEXIT_CRITICAL();
  return accepted;
}

void CatchGroundCubeSubsystem::setValve(int valve_id, bool open) {
  if (valve_id == 1)      core_.setValve1(open);
  else if (valve_id == 2) core_.setValve2(open);
}

void CatchGroundCubeSubsystem::valvePulse(int valve_id) {
  if (valve_id == 1)      core_.valve1Pulse();
  else if (valve_id == 2) core_.valve2Pulse();
}

void CatchGroundCubeSubsystem::resetValveState(bool valve1_open, bool valve2_open) {
  core_.resetValveState(valve1_open, valve2_open);
}

// =====================================================================
//  放置流程（s32）：start 类带运行守门，cancel 随时可调用
// =====================================================================
bool CatchGroundCubeSubsystem::startPut(float height_mm, PutRelease release) {
  if (!std::isfinite(height_mm)) return false;
  bool accepted = false;
  taskENTER_CRITICAL();
  if (acceptsCommands()) {
    accepted = put_ground_cube_.start(height_mm, release);
  }
  taskEXIT_CRITICAL();
  return accepted;
}

bool CatchGroundCubeSubsystem::startPut(const PutStep* steps, size_t count) {
  bool accepted = false;
  taskENTER_CRITICAL();
  if (acceptsCommands()) {
    accepted = put_ground_cube_.start(steps, count);
  }
  taskEXIT_CRITICAL();
  return accepted;
}

void CatchGroundCubeSubsystem::cancelPut() {
  put_ground_cube_.cancel();   // 只做状态切换，不碰硬件，无需守门
}

// =====================================================================
//  抓取流程（s32）：合爪→锁定→自动抬升；start 带运行守门，cancel 随时可调用
// =====================================================================
bool CatchGroundCubeSubsystem::startGrab(CatchGroundCube::GrabSelection sel) {
  bool accepted = false;
  taskENTER_CRITICAL();
  if (acceptsCommands()) {
    accepted = core_.startGrab(sel);
  }
  taskEXIT_CRITICAL();
  return accepted;
}

void CatchGroundCubeSubsystem::cancelGrab() {
  core_.cancelGrab();
}

CubeStatus CatchGroundCubeSubsystem::getStatus() const {
  CubeStatus status;
  status.height_mm        = core_.getHeightMm();
  status.target_height_mm = core_.target_position_deg_ * core_.config.mm_per_deg
                          + core_.config.ground_clearance_mm;
  status.sync_error_mm    = core_.getSyncErrorMm();
  status.sync_fault       = core_.sync_fault_;
  status.valve1_open      = core_.getValve1();
  status.valve2_open      = core_.getValve2();
  status.grab_state       = static_cast<uint8_t>(core_.getGrabState());
  return status;
}

// =====================================================================
//  Subsystem 生命周期回调
// =====================================================================
bool CatchGroundCubeSubsystem::isOnline(FaultInfo& online_info) const {
  (void)online_info;   // 需要故障详情时可参照 chassis 的写法填充
  return motorsOnline();
}

bool CatchGroundCubeSubsystem::motorsOnline() const {
  taskENTER_CRITICAL();
  const bool online = core_.left_motor_.is_online() && core_.right_motor_.is_online();
  taskEXIT_CRITICAL();
  return online;
}

void CatchGroundCubeSubsystem::onRequestReset() noexcept {
  core_.holdCurrentPosition();   // 撤目标时把升降目标冻结在当前位置（气缸不动）
}

void CatchGroundCubeSubsystem::onPrepare() {
  core_.init();   // PID 配置 + 阀 IO 安全初始态（每次上线只调一次）
}

Subsystem::Readiness CatchGroundCubeSubsystem::onCheckReadiness() {
  // 双电机持续在线即认为就绪；编码器累计位置以上电后首帧反馈自动建零
  return motorsOnline() ? Readiness::Ready : Readiness::Pending;
}

bool CatchGroundCubeSubsystem::onRun(bool enabled) {
  if (enabled) {
    core_.update();             // 1kHz 控制主循环（阀脉冲管理在其内部）
    put_ground_cube_.update();  // 放置流程状态机步进（s32，空闲时立即返回）
    return true;
  }
  core_.stopOutput();    // 关电机输出；阀脉冲收尾由 valveTick 继续保证
  return true;
}
