# motion

七段式 S 曲线速度规划器（一维运动，状态机实现）。

```cpp
#include "scurve_profile.hpp"

motion::SCurveProfile profile;

// 开始规划：前进 0.5 m，最大速度 1 m/s，加速度 2 m/s²，减速度 2 m/s²，加加速度 5 m/s³。
profile.plan(0.5f, {
    .max_velocity = 1.0f,
    .max_acceleration = 2.0f,
    .max_deceleration = 2.0f,
    .max_jerk = 5.0f,
});

while (!profile.done()) {
    float velocity = profile.update(dt);  // 当前速度
}
```

阶段：`Idle → 加加速 → 匀加速 → 减加速 → 匀速 → 加减速 → 匀减速 → 减减速 → Done`。

用加加速度 J 限制加速度变化率，到目标时速度与加速度都为零，运动平滑无冲击。位移小则退化为部分段（无匀速段或无匀加速段）。
