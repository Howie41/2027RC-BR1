#pragma once
#include "config.hpp"
#include "hfsm.hpp"

#include <cstdint>

namespace robot {

struct Starting : M::State {
    static constexpr const char* NAME = "Starting";
    HFSM_STATE_CALLBACKS_WITH_BASE(M);

private:
    enum class Phase { Preparing, WaitingRunning };
    Phase phase_{Phase::Preparing};
};
struct Disable : M::State {
    static constexpr const char* NAME = "Disable";
    HFSM_STATE_CALLBACKS_WITH_BASE(M);
};
struct Enable : M::State {
    static constexpr const char* NAME = "Enable";
    HFSM_STATE_CALLBACKS_WITH_BASE(M);
};
struct Manual : M::State {
    static constexpr const char* NAME = "Manual";
    HFSM_STATE_CALLBACKS_WITH_BASE(M);
};
struct UniversalManual : M::State {
    static constexpr const char* NAME = "UniversalManual";
    void entryGuard(GuardControl& control);
    HFSM_STATE_CALLBACKS_WITH_BASE(M);
};
struct Auto : M::State {
    static constexpr const char* NAME = "Auto";
    HFSM_STATE_CALLBACKS_WITH_BASE(M);
};
struct WaitUpperComputer : M::State {
    static constexpr const char* NAME = "WaitUpperComputer";
    HFSM_STATE_CALLBACKS_WITH_BASE(M);
};
struct DevelopingAuto : M::State {
    static constexpr const char* NAME = "DevelopingAuto";
    HFSM_STATE_CALLBACKS_WITH_BASE(M);
};
struct Paused : M::State {
    static constexpr const char* NAME = "Paused";
    HFSM_STATE_CALLBACKS_WITH_BASE(M);
};
struct Fault : M::State {
    static constexpr const char* NAME = "Fault";
    void exitGuard(GuardControl& control);
    HFSM_STATE_CALLBACKS_WITH_BASE(M);

private:
    bool reentering_{false};
};
struct Recovering : M::State {
    static constexpr const char* NAME = "Recovering";
    void entryGuard(GuardControl& control);
    HFSM_STATE_CALLBACKS_WITH_BASE(M);

private:
    enum class Phase { Preparing, WaitingRunning };
    Phase phase_{Phase::Preparing};
};

}
