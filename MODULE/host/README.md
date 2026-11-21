# 上下位机通信

从 `D:\temp\上下位机通信相关`（lxlx 的 USB 通信实现）移植，保留线路协议，使用项目现有工具：

| 原实现 | 当前实现 |
| --- | --- |
| 手动拼接、CRC、`memcpy` 解码 | `frame::Frame`、`Field`、`Length`、`Crc` |
| 按字节解包状态机 | `frame::FrameParser` |
| 运行时 ID / 长度注册表 | `HostComm<消息类型...>` 与类型化回调 |
| 自建 USB 队列、信号量任务 | 现有 `Algorithm::MpscQueue`、`rtos::Task` |
| `TypedTopicPublisher` | `rtos::Publisher<VisionPose>` |

## 线路协议

`FF FE | ID:u8 | LEN:u8 | PAYLOAD:LEN | CRC:u16le | FD FB`

CRC16/MODBUS 只覆盖 `ID + LEN + PAYLOAD`：初值 `0xFFFF`，反射多项式 `0xA001`，无最终异或。`CrcType::Crc16Modbus` 是新增算法，原有 `CrcType::Crc16` 行为保持不变。

每个 ID 对应一种固定载荷布局，最大载荷 64 字节。总帧长为载荷长加 8，允许空载荷。ID 纳入解析器的固定帧头以区分不同消息，同时仍计入 CRC。

当前消息 `host_comm::VisionPose`：ID `0x01`，载荷 20 字节，总长 28 字节。

| 载荷偏移 | 字段 | 格式 |
| --- | --- | --- |
| 0 | x | float32 小端 |
| 4 | y | float32 小端 |
| 8 | yaw | float32 小端 |
| 12 | timestamp_ms | uint32 小端 |
| 16 | seq | uint32 小端 |

单位沿用上位机约定，模块不作换算。编解码不依赖结构体大小、填充或本机字节序。

兼容性测试帧：`x=1, y=-2.5, yaw=0.75, timestamp_ms=0x12345678, seq=0x90ABCDEF`。

```text
FF FE 01 14 00 00 80 3F 00 00 20 C0 00 00 40 3F
78 56 34 12 EF CD AB 90 B5 85 FD FB
```

## APP 接入

`APP/tasks/host_communication.cpp` 自动注册静态任务，每个 tick 处理最多一队列接收块并推进发送。合法消息发布到 `robot::host::kVisionPoseTopic`（`"vision_pose"`）。收发模块不直接修改机构状态或 HFSM。

订阅者须为静态存储期对象，并在 `rtos_resource_init()` 前构造：

```cpp
#include "host_communication.hpp"
#include "pub_sub.hpp"

rtos::Subscriber<host_comm::VisionPose, 4> g_vision_pose{
    robot::host::kVisionPoseTopic};

// 在任务内消费，超时单位为毫秒。
auto pose = g_vision_pose.receive(10);
```

发送：

```cpp
host_comm::VisionPose pose{1.0F, -2.5F, 0.75F, 100, 1};
bool accepted = robot::host::send(pose);
```

`false` 表示未入队，调用方决定是否重试；`true` 只表示完整入队。协议没有 ACK，拔线时正在发送的数据可能丢失；尚未提交的数据保留至重新配置。订阅队列满时沿用发布订阅工具的丢弃策略。

USB 接收溢出或会话切换会改变接收块的 epoch，任务在处理新 epoch 前重置半帧缓存。旧队列中的完整消息仍可交付；业务需要实时性时应检查 `timestamp_ms` / `seq`。

## 扩展消息

在 `host_msgs.hpp` 定义消息：提供唯一 ID、`using Frame = HostFrame<ID, frame::Field<类型>...>`、`to_tuple()` 和 `from_tuple()`，再将类型加入 `robot::host::Communication`，在通信任务构造时按相同顺序补充回调。长度和 CRC 自动生成，未知 ID、非法长度、CRC 或帧尾不会交付回调。

`HostComm` 不绑定 USB，可直接向 `process_buffer()` 输入串口等字节流；同一实例须串行访问，回调中的消息引用仅在回调期间有效。旧 `registerMessage()` / `processBuffer()` / `buildFrame()` / `VisionPose::unpack()` 接口已替换。

## 验证

```powershell
cmake --preset Debug
cmake --build --preset Debug
python host_tests/host_communication/run_host_tests.py
```
