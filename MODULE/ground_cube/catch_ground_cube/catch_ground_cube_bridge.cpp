/**
 * @file catch_ground_cube_bridge.cpp
 * @brief 抓地立方体机构调试桥接层实现（新框架版）—— 转发到 robot::catch_ground_cube
 */

#include "catch_ground_cube_bridge.hpp"

#include "robot.hpp"   // robot::catch_ground_cube 实例声明处

namespace cube_dbg {

bool moveToHeight(float target_mm) {
  return robot::catch_ground_cube.moveToHeight(target_mm);
}

bool jog(float delta_mm) {
  return robot::catch_ground_cube.jog(delta_mm);
}

void valve(ValveId id, bool open) {
  robot::catch_ground_cube.setValve(static_cast<int>(id), open);
}

void valvePulse(ValveId id) {
  robot::catch_ground_cube.valvePulse(static_cast<int>(id));
}

void resetValveState(bool valve1_open, bool valve2_open) {
  robot::catch_ground_cube.resetValveState(valve1_open, valve2_open);
}

CubeStatus getStatus() {
  return robot::catch_ground_cube.getStatus();
}

bool grab(CatchGroundCube::GrabSelection sel) {
  return robot::catch_ground_cube.startGrab(sel);
}

void grabCancel() {
  robot::catch_ground_cube.cancelGrab();
}

bool putAt(float height_mm, PutRelease release) {
  return robot::catch_ground_cube.startPut(height_mm, release);
}

bool putAt(const PutStep* steps, size_t count) {
  return robot::catch_ground_cube.startPut(steps, count);
}

void putCancel() {
  robot::catch_ground_cube.cancelPut();
}

PutGroundCube::State putState() {
  return robot::catch_ground_cube.put().state();
}

} // namespace cube_dbg
