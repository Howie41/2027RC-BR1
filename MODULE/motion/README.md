# motion

梯形速度曲线规划器（一维运动，状态机实现）。

```cpp
#include "trapezoid_profile.hpp"

motion::TrapezoidProfile profile;

// 开始规划：从当前位置前进 0.5 m，最大速度 1 m/s，加速度/减速度 2 m/s²。
profile.plan(0.5f, {.max_velocity = 1.0f, .max_acceleration = 2.0f, .max_deceleration = 2.0f});

// 每个控制周期推进，返回当前速度。
float velocity = profile.update(dt);
if (profile.done()) {
    // 已到达目标，速度为零。
}
```

阶段：`Idle → Accel → Cruise → Decel → Done`。加速到最大速度后匀速，剩余距离不足减速时自动进入减速，到目标位移时速度恰好为零（位移小则退化为三角形，无匀速段）。
