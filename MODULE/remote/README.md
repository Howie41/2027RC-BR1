# remote

i-Bus 遥控器消息中心。由接收任务喂入帧，其它任务读取摇杆与开关。

- 摇杆 `right_x / right_y / left_x / left_y`：归一化 -1.0~1.0，死区 0.08。
- 开关 `switch_a~d`：原始通道值 1000~2000。
- `online()`：最近 100ms 内收到过帧为 true。

```cpp
robot::remote.feed(channels);   // 接收任务调用
if (robot::remote.online()) {
    float vx = robot::remote.left_x();
}
```
