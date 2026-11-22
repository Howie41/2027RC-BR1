# remote

i-Bus 遥控器：流式解析器（`remote::Parser`）+ 消息中心（`remote::Remote`）。

## Parser

FlySky i-Bus 流式解析器，逐字节喂入，收齐 32 字节并校验后输出 14 通道。

```cpp
remote::Parser parser;
std::array<std::uint16_t, remote::Parser::kChannels> channels{};
if (parser.parse(byte, channels)) {
    // channels 已更新。
}
```

帧格式：`0x20 0x40` + 14 通道（uint16 小端）+ 校验和（`0xFFFF - 前 30 字节之和`）。

## Remote

消息中心，由接收任务喂入帧，其它任务读取摇杆与开关。

- 摇杆 `right_x / right_y / left_x / left_y`：归一化 -1.0~1.0，死区 0.08。
- 开关 `switch_a~d`：原始通道值 1000~2000。
- `online()`：最近 100ms 内收到过帧为 true。

```cpp
robot::remote.feed(channels);   // 接收任务调用
if (robot::remote.online()) {
    float vx = robot::remote.left_x();
}
```
