#pragma once

#include "fault_info.hpp"

#include <atomic>
#include <cstdint>

/**
 * @brief 子系统独立等待上线、准备到待机，再接收业务运行请求。
 * @note 请求用于任务上下文，periodic() 由固定控制任务调用；输出切换异步执行。
 */
class Subsystem {
public:
    enum class State : std::uint8_t { Disabled, Preparing, Standby, Running, Failed };

    virtual ~Subsystem() = default;
    Subsystem(const Subsystem&) = delete;
    Subsystem& operator=(const Subsystem&) = delete;
    Subsystem(Subsystem&&) = delete;
    Subsystem& operator=(Subsystem&&) = delete;

    // 显式开始新一轮准备，也用于故障恢复；基类不限制准备时长。
    void requestPrepare();
    // 返回待机，复用有效准备结果；Failed 时拒绝。
    // 已在 Standby 时重新触发目标接管，准备/进入待机中的重复请求不重置进度。
    // 不清理业务目标，由 onStandby(true) 接管待机目标。
    [[nodiscard]] bool requestStandby();
    // 只在待机且没有其他未处理请求时接受，返回是否接受。
    [[nodiscard]] bool requestRun();
    // 撤销业务目标和运行授权；控制任务关闭输出后才发布 Disabled。
    void requestDisable();

    [[nodiscard]] State state() const noexcept;
    [[nodiscard]] bool isStandby() const noexcept;
    [[nodiscard]] bool isRunning() const noexcept;
    [[nodiscard]] bool hasFailed() const noexcept;
    // 最近一次已发布失败的现场快照；禁用、重新准备不会清除，由下一次失败更新。
    [[nodiscard]] FaultInfo faultInfo() const noexcept;
    void periodic();

protected:
    enum class Readiness : std::uint8_t { Pending, Ready, Failed };
    Subsystem() noexcept = default;

    // 仅在 periodic() 所调用的控制任务回调中记录，本周期开始时清空。
    // 回调记录后仍须返回 false / Readiness::Failed，由基类发布 Failed 和现场。
    FaultInfo current_fault_{};

    // 子系统内部的设备事实；整机不检查在线，派生类须同步读取反馈。
    [[nodiscard]] virtual bool isOnline() const = 0;
    // 准备或禁用请求方的短临界区内清空旧目标；待机请求不调用。
    // 不得阻塞、操作硬件或调用生命周期请求。
    virtual void onRequestReset() noexcept {}
    // 上线后开始实际准备时调用一次；正常返回待机不重复初始化。
    virtual void onPrepare() {}
    // 非阻塞准备步骤及运行条件检查；可执行准备所需的受控动作。
    [[nodiscard]] virtual Readiness onCheckReadiness() = 0;
    // 每周期设置/检查待机目标，默认返回 Ready；不得在此调用 onRun()。
    // 当前控制任务可在此调用普通业务 API，进入待机期间也会获得授权。
    // 返回 Pending 或 Ready 后，基类统一调用 onRun(true) 执行控制。
    [[nodiscard]] virtual Readiness onStandby(bool entering);
    // 条件就绪后，在进入/保持待机和运行期间每周期执行控制输出。
    // enabled==false 必须关闭并清空输出。所有回调须及时返回。
    [[nodiscard]] virtual bool onRun(bool enabled) = 0;
    // 返回当前是否能响应业务，disabled 或 Failed 时拒绝
    // 调用方必须在同一个短临界区内检查并写入。
    [[nodiscard]] bool acceptsCommands() const noexcept;

private:
    enum class Mode : std::uint8_t { Disabled, Prepare, Standby, Run };
    struct Request {
        Mode mode{Mode::Disabled};
        std::uint32_t generation{0};
        bool reset_preparation{true};
    };

    Request latest_request_{}; // 请求方和控制任务通过短临界区同步。
    std::atomic<State> state_{State::Disabled};
    FaultInfo fault_info_{}; // 发布和读取均受短临界区保护。
    // onStandby 回调所属任务的不透明句柄；读写均受短临界区保护。
    void* standby_callback_task_{nullptr};

    // 以下字段仅由控制任务访问。
    std::uint32_t active_generation_{0};
    bool preparation_started_{false};
    bool prepared_{false};
    bool standby_started_{false};
    State phase_{State::Disabled};
};
