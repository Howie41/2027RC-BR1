# ibus

FlySky i-Bus 流式解析器。逐字节喂入，收齐 32 字节并校验通过后输出 14 通道。

```cpp
#include "ibus.hpp"

ibus::Parser parser;
std::array<std::uint16_t, 14> channels{};

// 每收到一个字节调用一次；返回 true 表示整帧解析完成。
if (parser.parse(byte, channels)) {
    // channels 已更新。
}
```

帧格式：`0x20 0x40` + 14 通道（uint16 小端）+ 校验和（`0xFFFF - 前 30 字节之和`）。
