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
struct Test : M::State {
    static constexpr const char* NAME = "Test";
    HFSM_STATE_CALLBACKS_WITH_BASE(M);
};
struct TestStarting : M::State {
    static constexpr const char* NAME = "TestStarting";
    HFSM_STATE_CALLBACKS_WITH_BASE(M);

private:
    enum class Phase { Preparing, WaitingRunning };
    Phase phase_{Phase::Preparing};
};
struct TestRunning : M::State {
    static constexpr const char* NAME = "TestRunning";
    void entryGuard(GuardControl& control);
    HFSM_STATE_CALLBACKS_WITH_BASE(M);
// EB: 这里写测试业务要用的量 begin
private:
    bool speed_sequence_active_{false};
    unsigned speed_phase_{0};
    std::uint32_t speed_phase_started_ms_{0};
// EB: end
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
