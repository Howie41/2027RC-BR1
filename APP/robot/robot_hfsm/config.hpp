#pragma once
#include "hfsm.hpp"

namespace robot {

struct Robot_Context;

struct Disable;
struct Starting;
struct Enable;
struct Manual;
struct UniversalManual;
struct Auto;
struct WaitUpperComputer;
struct DevelopingAuto;
struct Paused;
struct Fault;
struct Recovering;
using StateList = hfsm::TypeList<Disable, Starting,
                    Enable, Manual, UniversalManual, Auto, WaitUpperComputer,
                    DevelopingAuto, Paused, Fault, Recovering>;
using M = hfsm::MachineT<Robot_Context, StateList>;
using Control = M::Control;
using GuardControl = M::GuardControl;
using State = M::State;

} // namespace robot
