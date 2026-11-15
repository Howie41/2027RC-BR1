#include "robot.hpp"
#include "states.hpp"

void robot::Disable::enter() {}

void robot::Disable::update(robot::Control& control) {
    control.changeTo<Enable>();
}

void robot::Disable::exit() {}
