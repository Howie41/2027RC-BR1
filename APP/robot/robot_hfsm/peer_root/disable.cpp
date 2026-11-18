#include "robot.hpp"
#include "states.hpp"

void robot::Disable::enter() {}

void robot::Disable::update(robot::Control& control) {
    // 提交 pr 前记得切换启动路径到 ManualStarting
    control.changeTo<TestStarting>();
    // control.changeTo<ManualStarting>();
}

void robot::Disable::exit() {}
