# TestSubsystem

TestSubsystem 是 C620 电机速度闭环的接入示例，当前未创建实例或周期任务，也未注册到整车。实例化时通过构造函数传入电机，以 PID test_motor_speed 执行速度闭环，支持 debug_task 调参。继承 Subsystem 的五状态生命周期，整机通过基类请求和状态查询协调，不直接操作电机。

| 状态 | 电机行为 |
| --- | --- |
| Disabled | 已关闭输出 |
| Preparing | 等待有效反馈、初始化控制器并进入零速度待机 |
| Standby | 默认零速度闭环保持，允许接受业务速度目标 |
| Running | 接受有限速度目标，执行速度闭环 |
| Failed | 保持故障，拒绝待机请求；显式重新准备后才恢复 |

isOnline 是内部反馈查询，在短临界区内读取电机在线信息。首次上线后 onPrepare 初始化 PID；onCheckReadiness 检查在线；onStandby 在首次进入待机时调用公开业务API setTargetSpeed(0)设置零速度目标，接受后返回 Ready，拒绝则返回 Failed。基类随后统一调用 onRun(true) 读取速度反馈、执行 PID 和应用输出，进入及保持待机时每周期持续控制。正常返回待机复用 PID；进入或保持待机期间，已经建立运行条件后掉线会关闭输出并保持 Failed，上线本身不能恢复。显式重新准备后才再次初始化 PID。Standby 表示保持策略已执行，不承诺实测速度已经归零。

setTargetSpeed 在短临界区内通过 acceptsCommands 检查授权，允许 Standby / Running 接受业务速度目标。Preparing时外部调用仍被拒绝，但当前有效请求的onStandby回调所属控制任务可以调用同一API设置待机目标。Failed和Disabled拒绝目标，禁用请求立即撤销授权；待机请求不调用onRequestReset。Standby接受目标不会自动切换到Running。实际输出由每 1 tick 调用的 periodic 异步处理。PID 计算在临界区外，反馈和 CAN 命令写入由派生类同步。

即使已经处于Standby，再次显式请求待机也会由onStandby(true)重新设置零速度目标，不会保留待机期间接收的业务速度；正在进入待机时的重复请求不重置进度，也不重新初始化PID。

接入后，先在线的机构独立准备到 Standby，其他机构继续等待；正式或测试启动状态等对应集合全部 Standby 后提交运行请求，再等待实际 Running。TestRunning 当前不包含具体测试业务。

发生故障时，Fault 保存来源并禁用故障对象，健康对象转入待机。显式置位 robot_context.recovery_requested 后才能进入 Recovering；修复对象重新准备到待机，再请求目标集合运行。健康对象无需重复初始化，恢复失败则回到 Fault 等待新的授权。测试集合外尚未建立运行条件的机构可以等待上线，不阻塞测试恢复；已经因待机掉线而失败、并被整机记录的机构，需要一并修复。
