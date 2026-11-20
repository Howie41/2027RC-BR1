# imu

HWT901B-TTL（六轴）流式解析器。逐字节喂入，收齐 11 字节并校验后更新加速度/角速度/角度。

```cpp
#include "hwt901b.hpp"

imu::Parser parser;
imu::Data data;

// 每收到一个字节调用一次；返回帧类型，None 表示尚未收齐或校验失败。
auto type = parser.parse(byte, data);
if (type == imu::FrameType::Angle) {
    // data.yaw / roll / pitch 已更新。
}
```

帧格式（11 字节）：`0x55 <type> <3×int16 LE> <int16 温度/版本> <校验和>`，校验和 = 前 10 字节之和的低 8 位。

- `0x51` = 加速度（ax/ay/az，m/s²）
- `0x52` = 角速度（wx/wy/wz，rad/s）
- `0x53` = 角度（roll/pitch/yaw，rad）

量程按 WITMotion 默认：加速度 `÷32768×16g`、角速度 `÷32768×2000°/s`、角度 `÷32768×180°`。

## YawEstimator

世界航向估计：`yaw = imu.yaw - 偏置`，首次 `predict()` 以当前 yaw 为基准，`reset()` 后重新捕获偏置。

```cpp
#include "yaw.hpp"

imu::YawEstimator heading;
heading.predict(data, now_ms);  // 每帧角度数据到达时调用
heading.reset();                // 以当前方向为基准
```
