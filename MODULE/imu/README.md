# imu

HWT901B-TTL 流式解析器。逐字节喂入，收齐 11 字节并校验后更新航向与角速度。

```cpp
#include "hwt901b.hpp"

imu::Parser parser;
imu::Data data;  // yaw_imu / omega_imu

// 每收到一个字节调用一次；返回帧类型，None 表示尚未收齐或校验失败。
auto type = parser.parse(byte, data);
```

帧格式（11 字节）：`0x55 <type> <3×int16 LE> <int16 温度/版本> <校验和>`，校验和 = 前 10 字节之和的低 8 位。

- `0x52` = 角速度 → `omega_imu`（rad/s）
- `0x53` = 角度 → `yaw_imu`（rad）

量程按 WITMotion 默认：角速度 `÷32768×2000°/s`、角度 `÷32768×180°`。
