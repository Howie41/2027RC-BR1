# Subsystem

`Subsystem` 是架构提供的一个子系统基类，用于驱动**机构**。架构的整车状态机已经实现了自动管理基于Subsystem的机构状态，只需要实现几个回调函数，就能自动完成整车初始化，等大电和故障处理。

建议先完整阅读本文再开始编写子系统

*如果你不打算基于 Subsystem 开发，就需要研究并自行接入整车状态机，详情询问相关人员或阅读相关文档(目前还没写)*

## 整个 Subsystem 的状态

| 状态 | 含义 |
| --- | --- |
| Disabled | 禁用状态，不会有动力输出 |
| Preparing | 正在准备中，等待上电、初始化代码并进入待机 |
| Standby | 机构处于默认状态，随时可以执行业务 |
| Running | 正在运行中 |
| Failed | 故障状态，子系统出现故障 |

业务接口可通过受保护的`acceptsCommands()`检查是否允许写入目标。Standby和Running允许接受业务目标；Preparing拒绝外部任务的业务目标，但基类会为本子系统正在执行onStandby()的控制任务授权，因此待机回调可以直接调用同一套业务API。该授权只在回调执行期间有效，且不能跨子系统或被其他任务借用；本轮请求被新的准备、待机或禁用请求替代后，旧回调也不能继续借用授权。Failed和Disabled拒绝目标，发出requestDisable后也立即拒绝。检查授权和写入目标必须放在同一个短临界区内。Standby接受业务目标不会自动切换成Running，仍由整车状态机管理生命周期。

已经处于Standby时，再次显式调用`requestStandby()`会重新进入待机流程，由下一控制周期的`onStandby(true)`接管目标，避免继续执行待机期间设置的业务目标。准备或进入待机尚未完成时，重复请求不会重置进度；已经有效的准备结果会被复用，不重新调用onPrepare或onRequestReset。

## Subsystem 的回调函数

这些回调函数是 Subsystem 的核心，需要认真考虑其实现。

回调函数的职责如下：

- `bool isOnline(FaultInfo& online_info) const` 同步检查电机、传感器等设备是否在线；离线时可用 `RECORD_FAULT(online_info, "具体设备掉线")` 记录原因后返回 false，全部在线时返回 true。多个设备离线时按检查顺序保留首个原因；未提供原因时由基类补充通用说明。
- `void onRequestReset()` 准备和禁用请求在请求方短临界区内调用，清除残留的业务目标和运行状态，比如把目标速度清零；不能阻塞或操作硬件。待机请求不调用，待机目标由onStandby(true)接管，默认onStandby实现不会清理原目标。
- `void onPrepare()` 初始化“代码”，就是纯粹代码上的初始化，不会控制外设，比如重置计数器，初始化PID控制器等。
- `Readiness onCheckReadiness()` 检查onPrepare()是否正常完成，一般“代码”上的初始化不会失败
- `Readiness onStandby(bool entering)` 通过普通业务API设置待机目标并检查是否到位，建议让机构锁止在当前位置，不自动归位。参数entering为true时，表示本轮进入待机的第一次调用，可用于一次性记录保持位置或提交锁止目标；后续周期继续调用，完成后也会持续调用以保持待机。基类会在回调期间授权当前控制任务调用业务API，无需另写内部版本或绕过acceptsCommands()；应检查业务API返回值。具体控制计算放在onRun()，不要在这里手动调用onRun()。返回Pending或Ready后，基类会统一调用onRun(true)执行控制；返回Failed则关闭输出。默认实现直接返回Ready。
- `bool onRun(bool enable)` 机构控制的所有逻辑都写这里。设备在线且运行条件建立后，进入待机（onStandby返回Pending）、保持待机和业务运行期间，基类都会每周期调用onRun(true)。禁用、离线或故障时调用onRun(false)关闭输出；开始新一轮准备时也会先关闭输出。onCheckReadiness返回Pending、尚未建立运行条件时，不自动调用onRun(true)。返回false表示控制失败，基类会进入Failed并调用onRun(false)关闭输出。

*`Readiness`有3个值：`Pending`（进行中）,`Ready`（完成）,`Failed`（失败）*

## 故障现场记录

在线检查通过参数 `online_info` 记录设备原因，不修改 `current_fault_`。首次等待上线仍保持 Preparing，不发布故障；运行条件建立后掉线才发布 Failed，并保留在线检查处的原因、文件名和行号。关闭输出时的记录不会覆盖掉线原因，故障槽位数量不变。

`onlineInfo()` 返回受临界区保护的最近一次在线检查快照，调试器暂停后可直接查看子系统的 `online_info_`。等待上电时保留首个离线设备的原因、文件名和行号，每次有效检查更新，全部在线时清空；未检查时初始为空。Disabled / Failed 不检查在线，保留上次结果直至重新准备后再次检查；被新请求替代的旧周期不发布结果。这份信息只反映设备在线检查，不表示准备动作或待机动作的进度。

运行 `python MODULE/subsystem/host_tests/run_online_tests.py` 验证掉线诊断，需要宿主机 g++。

回调可以在返回失败前用宏记录具体原因，自动保存文件名和行号：

```cpp
if (!std::isfinite(current)) {
    RECORD_FAULT(current_fault_, "Motor PID output is not finite");
    return false; // onCheckReadiness / onStandby 中应返回 Readiness::Failed
}
```

`RECORD_FAULT(destination, "原因")` 只记录，不切换状态、不进入 Error_Handler，也不关闭中断。原因必须是字符串字面量或具有静态生命周期的字符串，不保存临时字符串的 c_str()。宏不受 ENABLE_RUNTIME_ASSERT 开关影响。

`current_fault_` 仅用于 periodic() 调用的控制任务回调，不用于业务接口、onRequestReset 或其他任务。每个控制周期清空一次；同一周期优先保留第一条原因，基类仅在没有具体原因时补上离线、准备失败、运行失败或待机失败的说明。回调仍须按原约定返回失败并完成必要的输出处理，单独记录不会使子系统失败。

基类仅在本轮请求仍有效且发布 Failed 时提交现场，可通过 `faultInfo()` 获取受临界区保护的快照。已发布的现场在禁用和重新准备后继续保留，下一次有效失败会更新；旧请求被新请求替代时，其计算结果和现场都不会发布。整机进入 Fault 后，把现场复制到 `robot::robot_context.fault_details[i]`，其中 i 与 subsystems / faults 的下标对应。

## 回调的调用时序和子系统的状态切换

1. 程序启动，子系统处于Disabled状态。完成系统初始化后，`onRun(bool enable)`就开始被循环调用，此时的enable为false，
2. 整车状态机发出`requestPrepare()`，子系统进入Preparing并等待设备上线；上线后由periodic()调用一次`onPrepare()`，随后每周期调用`onCheckReadiness()`。返回Pending继续准备，返回Ready开始进入待机，返回Failed进入故障状态。
3. 运行条件建立后，periodic()先调用`onStandby(true)`设置或检查待机目标，再调用`onRun(true)`执行控制。后续待机周期先调用`onStandby(false)`，再调用`onRun(true)`。
4. `onStandby()`返回Pending时，保持Preparing但仍执行`onRun(true)`；返回Ready且控制成功时，进入Standby，之后继续周期执行待机及控制回调。返回Failed时不再执行onRun(true)，而是进入故障并调用onRun(false)；onRun(true)返回false也会进入故障并关闭输出。
5. 整车状态机在所有子系统都处于Standby状态后，才会尝试切换到Running状态。此时`onRun()`传入的参数为true

**任何子系统出现故障时：**
**出现故障的**子系统会进入**disabled状态**，其余子系统会尝试进入Standby状态
