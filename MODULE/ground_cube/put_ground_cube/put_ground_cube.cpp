/**
 * @file put_ground_cube.cpp
 * @brief 放地立方体机构实现（新框架版）—— 非阻塞放置状态机
 * @attention
 *  1. update() 假设 1kHz 调用（由 Subsystem onRun 保证），计时直接按毫秒计数。
 *  2. LIFTING 态每帧重发目标高度：外部手动 jog 干扰只会导致超时 FAILED，不会损坏机构。
 *  3. 任一步检测到同步故障立即 FAILED 并停在本帧高度；恢复需上层 cancel/reset 后重发。
 * @note 释放电平 = CGC_VALVE_CLAMPED 的反面（开爪 = 离开夹紧态），与抓取流程共用同一约定
 */

#include "put_ground_cube.hpp"

#include <cmath>

bool PutGroundCube::start(float height_mm, PutRelease release) {
  const PutStep step{height_mm, release};
  return start(&step, 1);
}

bool PutGroundCube::start(const PutStep* steps, size_t count) {
  if (steps == nullptr || count == 0 || count > kMaxSteps) {
    return false;
  }
  for (size_t i = 0; i < count; ++i) {
    if (!std::isfinite(steps[i].height_mm)) {
      return false;
    }
  }
  if (mech_.sync_fault_) {   // 同步封波中不允许起新流程（升降根本动不了）
    return false;
  }

  for (size_t i = 0; i < count; ++i) {
    steps_[i] = steps[i];
  }
  step_count_   = count;
  step_index_   = 0;
  step_timer_ms_= 0;
  state_        = State::LIFTING;
  return true;
}

void PutGroundCube::cancel() {
  state_         = State::IDLE;
  step_timer_ms_ = 0;
  // 注意：不动阀、不改升降目标——中断时机构保持现状，由上层决定下一步
}

void PutGroundCube::reset() {
  if (state_ == State::DONE || state_ == State::FAILED) {
    state_ = State::IDLE;
  }
}

void PutGroundCube::update() {
  switch (state_) {
    case State::IDLE:
    case State::DONE:
    case State::FAILED:
      return;

    case State::LIFTING: {
      if (mech_.sync_fault_) {
        state_ = State::FAILED;
        return;
      }
      const PutStep& step = steps_[step_index_];
      mech_.moveToHeightMm(step.height_mm);   // 每帧重发，防外部干扰丢目标

      const float err_mm = std::fabs(mech_.getHeightMm() - step.height_mm);
      if (err_mm <= config.arrive_tolerance_mm) {
        stable_timer_ms_ = config.stable_time_ms;
        state_ = State::STABLE;
      } else if (++step_timer_ms_ >= config.lift_timeout_ms) {
        state_ = State::FAILED;   // 到不了高度（卡住/撞限位），上报失败而不是死等
      }
      break;
    }

    case State::STABLE: {
      if (mech_.sync_fault_) {
        state_ = State::FAILED;
        return;
      }
      const PutStep& step = steps_[step_index_];
      mech_.moveToHeightMm(step.height_mm);   // 稳定期内继续钉住目标

      if (stable_timer_ms_ > 0) {
        --stable_timer_ms_;
        break;
      }
      // ---- 到位且稳定：开爪 ----
      constexpr bool kRelease = !CGC_VALVE_CLAMPED;   // 释放 = 离开夹紧态
      switch (step.release) {
        case PUT_RELEASE_BOTH:
          mech_.setValve1(kRelease);
          mech_.setValve2(kRelease);
          break;
        case PUT_RELEASE_VALVE1_ONLY:
          mech_.setValve1(kRelease);   // 只放阀1对应的爪，另一只保持夹紧
          break;
        case PUT_RELEASE_VALVE2_ONLY:
          mech_.setValve2(kRelease);
          break;
      }
      release_timer_ms_ = config.release_settle_ms;
      state_ = State::RELEASING;
      break;
    }

    case State::RELEASING: {
      if (release_timer_ms_ > 0) {
        --release_timer_ms_;
        break;
      }
      ++step_index_;
      if (step_index_ < step_count_) {   // 还有后续步骤：去下一个高度继续放
        step_timer_ms_ = 0;
        state_ = State::LIFTING;
      } else {
        state_ = State::DONE;
      }
      break;
    }
  }
}
