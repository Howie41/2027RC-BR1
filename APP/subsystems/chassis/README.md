# chassis

全向轮底盘子系统：四轮 C620 速度闭环，接收归一化速度并解算各轮目标。

| 状态 | 行为 |
| --- | --- |
| Disabled | 关闭输出 |
| Preparing | 等待四轮在线，初始化 PID |
| Standby | 零速度闭环保持 |
| Running | 按 setVelocity 目标执行速度闭环 |
| Failed | 关闭输出，显式重新准备后恢复 |

```cpp
robot::chassis.setVelocity(vx, vy, omega);  // vx 前进 / vy 左移 / omega 逆时针，各 -1.0~1.0
```

四轮顺序 FR/FL/RL/RR，对应 CAN1 电机 1~4（反馈 0x201~0x204）。控制任务
[chassis_task.cpp](../../tasks/chassis_task.cpp) 每 1ms 读取遥控并驱动底盘。
