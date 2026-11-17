/**
 * @file catch_ground_cube.cpp
 * @brief 抓地立方体机构实现（新框架版）—— 双3508同步升降 + 双气缸控制
 * @attention 首次 update() 以两侧均值初始化目标，避免电机跳变
 * @note 控制结构沿用 Lift 模板：共用目标位置环 + 双速度环 + 同步误差交叉补偿
 * @note s24: 电机命令走 set_output_command（受 enable 门控），反馈读数包临界区快照
 */

#include "catch_ground_cube.hpp"
#include "rtos_utility.hpp"

#include <bitset>
#include <cmath>

namespace {

// 把模块的 PidParam 应用到框架 PidController（configure 内部会复位运行状态）
void configure_pid(PidController& pid, const CatchGroundCube::Config::PidParam& p,
                   const std::bitset<PidController::improvement_count>& improvement) {
  PidController::Config c{};
  c.kp.store(p.kp);
  c.ki.store(p.ki);
  c.kd.store(p.kd);
  c.max_out.store(p.max_out);
  c.integral_limit.store(p.integral_limit);
  c.dead_band.store(0.0f);
  c.improvement.store(improvement);
  pid.configure(c);
}

} // namespace

CatchGroundCube::CatchGroundCube(C620Motor& left, C620Motor& right)
    : left_motor_(left),
      right_motor_(right),
      position_pid_("cgc_position"),
      left_speed_pid_("cgc_speed_left"),
      right_speed_pid_("cgc_speed_right") {}

void CatchGroundCube::init() {
  // 上电先把两路阀 IO 拉到低电平，并清零软件记账：
  // 电平模式 → IO 低 = 关；脉冲模式 → 低电平为空闲态，防止误带电
  HAL_GPIO_WritePin(CGC_VALVE1_PORT, CGC_VALVE1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CGC_VALVE2_PORT, CGC_VALVE2_PIN, GPIO_PIN_RESET);
  valve1_open_ = false;
  valve2_open_ = false;
  valve1_pulse_remaining_ms_ = 0;
  valve2_pulse_remaining_ms_ = 0;

  // PID 配置（Improve 标志一一对应旧框架：积分限幅/微分先行/增量输出）
  configure_pid(position_pid_, config.position_pid,
                {1U << PidController::integral_limit |
                 1U << PidController::derivative_on_measurement});
  configure_pid(left_speed_pid_, config.speed_pid,
                {1U << PidController::integral_limit |
                 1U << PidController::derivative_on_measurement |
                 1U << PidController::increment_of_out});
  configure_pid(right_speed_pid_, config.speed_pid,
                {1U << PidController::integral_limit |
                 1U << PidController::derivative_on_measurement |
                 1U << PidController::increment_of_out});

  // 电机配置：减速比 + 电流上限（新框架 init 不清反馈）
  left_motor_.init(config.motor_reduction, config.motor_max_current);
  right_motor_.init(config.motor_reduction, config.motor_max_current);
}

void CatchGroundCube::valveTick() {
  // 阀脉冲倒计时：到点拉低。电平模式下 remaining 恒为 0，本函数自动不生效
  if (valve1_pulse_remaining_ms_ > 0 && --valve1_pulse_remaining_ms_ == 0) {
    HAL_GPIO_WritePin(CGC_VALVE1_PORT, CGC_VALVE1_PIN, GPIO_PIN_RESET);
  }
  if (valve2_pulse_remaining_ms_ > 0 && --valve2_pulse_remaining_ms_ == 0) {
    HAL_GPIO_WritePin(CGC_VALVE2_PORT, CGC_VALVE2_PIN, GPIO_PIN_RESET);
  }
}

void CatchGroundCube::stopOutput() {
  // 失能电机（新框架 disable 会清零命令，总线继续发零电流帧保电调心跳）
  left_motor_.disable();
  right_motor_.disable();
  valveTick();   // 失能期间脉冲照常收尾，IO 不长期带电
}

void CatchGroundCube::holdCurrentPosition() {
  target_position_deg_ = platformPositionDeg();
}

void CatchGroundCube::update() {
  valveTick();

  // 左右符号：left_invert=true 时左电机取负，实现"一正一反"驱动同一机构
  const float left_direction_sign  = config.left_invert ? -1.0f : 1.0f;
  const float right_direction_sign = 1.0f;

  // 反馈在中断里更新，任务侧一次性快照（test_subsystem 同款约定）
  float left_position_deg, right_position_deg, left_speed_radps, right_speed_radps;
  taskENTER_CRITICAL();
  left_position_deg  = left_direction_sign  * left_motor_.get_current_sum_pos();
  right_position_deg = right_direction_sign * right_motor_.get_current_sum_pos();
  left_speed_radps   = left_direction_sign  * left_motor_.get_current_speed();
  right_speed_radps  = right_direction_sign * right_motor_.get_current_speed();
  taskEXIT_CRITICAL();

  float platform_position_deg = (left_position_deg + right_position_deg) * 0.5f;

  sync_error_deg_ = left_position_deg - right_position_deg;   // 左高为正

  // ---- 首次初始化：以当前均值为目标，两侧都不跳 ----
  if (!initialized_) {
    target_position_deg_ = platform_position_deg;
    initialized_ = true;
  }

  // ---- 行程限位钳制 ----
  {
    float target_abs_mm = target_position_deg_ * config.mm_per_deg + config.ground_clearance_mm;
    float cur_abs_mm    = platform_position_deg * config.mm_per_deg + config.ground_clearance_mm;
    target_abs_mm = clampTarget(target_abs_mm, cur_abs_mm);
    target_position_deg_ = (target_abs_mm - config.ground_clearance_mm) / config.mm_per_deg;
  }

  // ---- 同步误差封波保护（带回差，防阈值边界抖动） ----
  if (!sync_fault_ && std::fabs(sync_error_deg_) > config.sync_threshold_deg) {
    sync_fault_ = true;
  } else if (sync_fault_ && std::fabs(sync_error_deg_) < config.sync_threshold_deg * 0.5f) {
    sync_fault_ = false;
  }

  if (sync_fault_) {
    left_motor_.disable();
    right_motor_.disable();
    return;   // 封波期间抓取流程一并冻结（stepGrabSequence 在下方才执行）
  }

  // ---- 抓取流程步进（s32：合爪→等待锁定→自动抬升；只改目标/阀，不改电机输出） ----
  stepGrabSequence();

  // ---- 位置环 100Hz（调用1kHz，10分频） ----
  if (position_loop_prescaler_ >= 10) {
    position_loop_output_ = position_pid_.calculate(platform_position_deg, target_position_deg_);
    // 同步交叉补偿：左高则左减速、右加速
    left_speed_target_  = position_loop_output_ - sync_error_deg_ * config.sync_comp_gain;
    right_speed_target_ = position_loop_output_ + sync_error_deg_ * config.sync_comp_gain;
    position_loop_prescaler_ = 0;
  }

  // ---- 双速度环 1kHz ----
  const float left_current_ma  = left_speed_pid_.calculate(left_speed_radps,  left_speed_target_);
  const float right_current_ma = right_speed_pid_.calculate(right_speed_radps, right_speed_target_);

  // 新框架命令接口：存目标 + 使能（enable 幂等，恢复出封波后自动重新允许输出）
  (void)left_motor_.set_output_command({.current_ma = left_direction_sign * left_current_ma});   // 一正一反 (void)消除nodiscard警告
  (void)right_motor_.set_output_command({.current_ma = right_direction_sign * right_current_ma});
  left_motor_.enable();
  right_motor_.enable();

  position_loop_prescaler_++;
}

// =====================================================================
//  统一设置入口：手动桥接 / 后续自动控制 都走这一个函数
// =====================================================================
void CatchGroundCube::applySetting(const CatchSetMsg &setting) {
  // ---- 气缸 ----
  if (setting.valve1_cmd > 0)       setValve1(true);
  else if (setting.valve1_cmd < 0)  setValve1(false);
  if (setting.valve2_cmd > 0)       setValve2(true);
  else if (setting.valve2_cmd < 0)  setValve2(false);

  // ---- 升降目标 ----
  if (setting.lift_absolute) {
    moveToHeightMm(setting.lift_target_mm);
  } else if (setting.lift_delta_mm != 0.0f) {
    addLiftDelta(setting.lift_delta_mm);
  }
}

// =====================================================================
//  气缸 IO（模式由 config.valve_mode 决定：脉冲自保持 / 电平保持，接口语义一致）
// =====================================================================
void CatchGroundCube::valve1Pulse() {
  if (config.valve_mode == VALVE_MODE_LEVEL) {
    setValve1(!valve1_open_);   // 电平模式无"脉冲"概念：退化为强制翻转状态
    return;
  }
  if (valve1_pulse_remaining_ms_ > 0) {
    return;   // 脉冲进行中，忽略新指令（防抖）
  }
  HAL_GPIO_WritePin(CGC_VALVE1_PORT, CGC_VALVE1_PIN, GPIO_PIN_SET);
  valve1_pulse_remaining_ms_ = config.valve_pulse_ms;  // valveTick 1kHz → 每计数 1ms
  valve1_open_ = !valve1_open_;                        // 一个脉冲 = 切换一次状态
}

void CatchGroundCube::valve2Pulse() {
  if (config.valve_mode == VALVE_MODE_LEVEL) {
    setValve2(!valve2_open_);
    return;
  }
  if (valve2_pulse_remaining_ms_ > 0) {
    return;
  }
  HAL_GPIO_WritePin(CGC_VALVE2_PORT, CGC_VALVE2_PIN, GPIO_PIN_SET);
  valve2_pulse_remaining_ms_ = config.valve_pulse_ms;
  valve2_open_ = !valve2_open_;
}

void CatchGroundCube::setValve1(bool open) {
  if (config.valve_mode == VALVE_MODE_LEVEL) {
    HAL_GPIO_WritePin(CGC_VALVE1_PORT, CGC_VALVE1_PIN,
                      open ? GPIO_PIN_SET : GPIO_PIN_RESET);  // 直接写电平
    valve1_open_ = open;
    return;
  }
  if (open != valve1_open_) {
    valve1Pulse();   // 只有状态不符才发脉冲
  }
}

void CatchGroundCube::setValve2(bool open) {
  if (config.valve_mode == VALVE_MODE_LEVEL) {
    HAL_GPIO_WritePin(CGC_VALVE2_PORT, CGC_VALVE2_PIN,
                      open ? GPIO_PIN_SET : GPIO_PIN_RESET);
    valve2_open_ = open;
    return;
  }
  if (open != valve2_open_) {
    valve2Pulse();
  }
}

bool CatchGroundCube::getValve1() const {
  return valve1_open_;   // 程序记账状态（电平模式下即IO电平）
}

bool CatchGroundCube::getValve2() const {
  return valve2_open_;
}

void CatchGroundCube::resetValveState(bool valve1_open, bool valve2_open) {
  valve1_open_ = valve1_open;   // 上电后若不确定气缸实际位置，先调它对齐记账
  valve2_open_ = valve2_open;
  if (config.valve_mode == VALVE_MODE_LEVEL) {
    // 电平模式：IO 电平即物理状态，直接把 IO 驱到记账状态
    HAL_GPIO_WritePin(CGC_VALVE1_PORT, CGC_VALVE1_PIN,
                      valve1_open_ ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CGC_VALVE2_PORT, CGC_VALVE2_PIN,
                      valve2_open_ ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  // 脉冲模式：物理状态由阀自身保持，程序只记账，不发脉冲
}

// =====================================================================
//  抓取流程（s32 新增：合爪 → 等待锁定 → 自动抬升防地面摩擦）
//  非阻塞：startGrab 立即返回，本函数由 update() 在 1kHz 下步进
// =====================================================================
bool CatchGroundCube::startGrab(GrabSelection sel) {
  // 合爪：把选中爪的阀切到夹紧态（电平/脉冲两种模式语义一致，由 setValve 内部分派）
  if (sel == GrabSelection::BOTH || sel == GrabSelection::VALVE1_ONLY) {
    setValve1(CGC_VALVE_CLAMPED);
  }
  if (sel == GrabSelection::BOTH || sel == GrabSelection::VALVE2_ONLY) {
    setValve2(CGC_VALVE_CLAMPED);
  }
  grab_selection_    = sel;
  grab_timer_ms_     = config.grab_settle_ms;
  grab_lift_pending_ = config.lift_after_grab_mm > 0.0f;   // 0=关闭自动抬升（旧行为）
  grab_state_        = GrabState::CLOSING;
  return true;
}

void CatchGroundCube::cancelGrab() {
  grab_state_        = GrabState::IDLE;
  grab_lift_pending_ = false;
  // 不动阀、不改目标：中断时机构保持现状，由上层决定下一步
}

bool CatchGroundCube::isGrabBusy() const {
  return grab_state_ == GrabState::CLOSING || grab_state_ == GrabState::LIFTING;
}

void CatchGroundCube::stepGrabSequence() {
  switch (grab_state_) {
    case GrabState::IDLE:
    case GrabState::DONE:
      return;

    case GrabState::CLOSING:
      if (grab_timer_ms_ > 0) {
        --grab_timer_ms_;   // 等待爪锁定（阀动作 + 机械到位）
        return;
      }
      if (grab_lift_pending_) {
        addLiftDelta(config.lift_after_grab_mm);   // 抬一点，保证方块离地不摩擦
        grab_lift_pending_ = false;
        grab_state_ = GrabState::LIFTING;
      } else {
        grab_state_ = GrabState::DONE;             // 未启用自动抬升：锁定即完成
      }
      return;

    case GrabState::LIFTING: {
      // 到位判定：当前高度贴近目标（含 1.5mm 容差【待填】）；超时 2s 也视为完成，
      // 抬升量本身很小，卡住时以实际位置为准，不让流程死等
      const float target_mm = target_position_deg_ * config.mm_per_deg + config.ground_clearance_mm;
      if (std::fabs(getHeightMm() - target_mm) <= 1.5f || ++grab_timer_ms_ >= 2000) {
        grab_state_ = GrabState::DONE;
      }
      return;
    }
  }
}

// =====================================================================
//  升降辅助接口
// =====================================================================
void CatchGroundCube::addLiftDelta(float delta_mm) {
  target_position_deg_ += delta_mm / config.mm_per_deg;
}

void CatchGroundCube::setLiftTargetMm(float target_mm) {
  target_position_deg_ = (target_mm - config.ground_clearance_mm) / config.mm_per_deg;
}

bool CatchGroundCube::moveToHeightMm(float target_mm) {
  // 行程上下界（与 update() 内 clampTarget 同一套限位）
  const float min_mm = config.ground_clearance_mm;
  const float max_mm = config.ground_clearance_mm + config.travel_max_mm;

  bool in_range = (target_mm >= min_mm) && (target_mm <= max_mm);

  // 超限时钳到最近边界再执行，防止非法目标损坏机构
  float clamped_mm = target_mm;
  if (clamped_mm < min_mm) clamped_mm = min_mm;
  if (clamped_mm > max_mm) clamped_mm = max_mm;

  setLiftTargetMm(clamped_mm);   // 复用底层接口，单位换算只在一处维护
  return in_range;
}

float CatchGroundCube::platformPositionDeg() const {
  const float left_direction_sign = config.left_invert ? -1.0f : 1.0f;
  float left_pos, right_pos;
  taskENTER_CRITICAL();
  left_pos  = left_direction_sign * left_motor_.get_current_sum_pos();
  right_pos = right_motor_.get_current_sum_pos();
  taskEXIT_CRITICAL();
  return (left_pos + right_pos) * 0.5f;
}

float CatchGroundCube::getHeightMm() const {
  return platformPositionDeg() * config.mm_per_deg + config.ground_clearance_mm;
}

float CatchGroundCube::getSyncErrorMm() const {
  const float left_direction_sign = config.left_invert ? -1.0f : 1.0f;
  float left_pos, right_pos;
  taskENTER_CRITICAL();
  left_pos  = left_direction_sign * left_motor_.get_current_sum_pos();
  right_pos = right_motor_.get_current_sum_pos();
  taskEXIT_CRITICAL();
  return std::fabs(left_pos - right_pos) * config.mm_per_deg;
}

float CatchGroundCube::clampTarget(float target_mm, float current_mm) const {
  const float min_mm = config.ground_clearance_mm;
  const float max_mm = config.ground_clearance_mm + config.travel_max_mm;
  if (min_mm >= max_mm) return target_mm;
  if (target_mm < min_mm) return min_mm;
  if (target_mm > max_mm) return max_mm;
  return target_mm;
}
