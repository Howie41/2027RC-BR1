/**
 * @file catch_ground_cube_bridge.hpp
 * @brief 抓地立方体机构调试桥接层（新框架版）—— debug_task 等调试代码统一从这里调用
 * @attention
 *  1. 调试代码只 include 本文件，不直接接触子系统内部；
 *  2. 本层只做转发，函数签名与旧框架版完全一致，debug_task 调用代码不用改；
 *  3. 升降类调用只有在子系统处于 Running（requestRun 已接受）时才生效，
 *     调试前记得先 requestPrepare() + requestRun()（或整车进入 Test 流程）。
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "catch_ground_cube_subsystem.hpp"

namespace cube_dbg {

/// 气缸编号
enum ValveId : uint8_t {
  VALVE_1 = 1,
  VALVE_2 = 2,
};

/**
 * @brief 输入给定高度，机构自动走位
 * @param target_mm 目标高度(mm)，行程坐标
 * @retval true=原样接受; false=未运行，或超行程已钳到最近边界
 */
bool moveToHeight(float target_mm);

/// 点动：目标增量(mm)，正升负降；false=未运行
bool jog(float delta_mm);

/// 气缸开关：id=VALVE_1/VALVE_2（不受运行状态限制，随时可手动操作）
void valve(ValveId id, bool open);

/// 气缸取反（脉冲模式强制发一个脉冲）
void valvePulse(ValveId id);

/// 上电后校准气缸记账状态
void resetValveState(bool valve1_open, bool valve2_open);

/// 读取状态快照
CubeStatus getStatus();

// ---- 抓取流程（s32：合爪→锁定→自动抬升防摩擦；非阻塞） ----
bool  grab(CatchGroundCube::GrabSelection sel = CatchGroundCube::GrabSelection::BOTH);
void  grabCancel();

// ---- 放置流程（s32：功能1/2/3 统一入口，见 PutRelease 注释） ----
bool  putAt(float height_mm, PutRelease release);          // false=未运行/同步故障/参数非法
bool  putAt(const PutStep* steps, size_t count);           // 多步队列（预留扩展）
void  putCancel();
PutGroundCube::State putState();                           // 查询放置流程状态机

} // namespace cube_dbg
