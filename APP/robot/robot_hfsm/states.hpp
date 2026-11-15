#pragma once
#include "config.hpp"
#include "hfsm.hpp"

#include <cstdint>

namespace robot {

struct Starting : M::State {
    HFSM_STATE_CALLBACKS_WITH_BASE(M);

private:
    enum class Phase { Preparing, WaitingRunning };
    Phase phase_{Phase::Preparing};
};
struct Disable : M::State {
    HFSM_STATE_CALLBACKS_WITH_BASE(M);
};
struct Enable : M::State {
    HFSM_STATE_CALLBACKS_WITH_BASE(M);
};
struct Manual : M::State {
    HFSM_STATE_CALLBACKS_WITH_BASE(M);
};
struct UniversalManual : M::State {
    void entryGuard(GuardControl& control);
    HFSM_STATE_CALLBACKS_WITH_BASE(M);
};
struct Auto : M::State {
    HFSM_STATE_CALLBACKS_WITH_BASE(M);
};
struct WaitUpperComputer : M::State {
    HFSM_STATE_CALLBACKS_WITH_BASE(M);
};
struct DevelopingAuto : M::State {
    HFSM_STATE_CALLBACKS_WITH_BASE(M);
};
struct Paused : M::State {
    HFSM_STATE_CALLBACKS_WITH_BASE(M);
};
struct Fault : M::State {
    void exitGuard(GuardControl& control);
    HFSM_STATE_CALLBACKS_WITH_BASE(M);

private:
    bool reentering_{false};
};
struct Recovering : M::State {
    void entryGuard(GuardControl& control);
    HFSM_STATE_CALLBACKS_WITH_BASE(M);

private:
    enum class Phase { Preparing, WaitingRunning };
    Phase phase_{Phase::Preparing};
};

}
