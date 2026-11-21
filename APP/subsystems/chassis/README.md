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


核心prompt：
我决定不要搞那么复杂，将进行一次重大重构（请以目前的正逆解算公式为准，正逆解公式沿用当前代码，不在此 prompt 里重写；只有遥控器右摇杆的左右需要取反，右摇杆左右取反只做一次，且只在 omega 遥控映射处；omega_imu 不取反）：
现在规定只有一套世界坐标系，用yaw_bias来基准化，使用reset_yaw_bias来将imu当前yaw值赋值给yaw_bias，每次周期性更新时都计算yaw = yaw_imu - yaw_bias作为逻辑上的yaw。
imu也别搞那么复杂，现在imu不会发加速度了，只需要记录当前yaw_imu与omega_imu，对象名称直接叫imu而不要world_frame、estimator啥的。
有头模式下，遥控器左摇杆的上下为ay_body，左右为ax_body，将它们分解为ax_world = ax_body*cos(yaw)-ay_body*sin(yaw)和ay_world = ax_body*sin(yaw)+ay_body*cos(yaw)（可以封装为bodyToWorld方法），然后用ax_world和ay_world分别积分得到vx_world和vy_world，再应用worldToBody得到vx_body和vy_body。
无头模式下遥控器左摇杆不再表示ay_body和ax_body，而是直接输出ay_world和ax_world并执行上述积分得到vx_body和vy_body，因此属性里只需要储存一个headlessMode属性就够了（用if条件判断headlessMode来改遥控器摇杆映射，由于omega没有有头/无头模式之分所以不用进if内部）。
我们用vx_world和vy_world作为底盘线速度（外环）pid（增量式）的目标值，用vx_world_cur和vy_world_cur表示实际值（作为外环pid的当前值），实际值有两种获取方式，它们依赖于属性autoMode（可使用setAutoMode来设置）：当autoMode为false时，直接测量四个轮子的measured转速做运动学正解得到vx_body_cur和vy_body_cur，应用bodyToWorld得到vx_world_cur和vy_world_cur；当autoMode为true时，vx_world_cur和vy_world_cur会在onRun中主动获取速度命令的话题队列来获取值，同样，此时遥控器禁用，vx_world和vy_world也从话题队列取值。
外环pid做完后，得到outputx_world和outputy_world，应用worldToBody得到outputx_body和outputy_body，运动学逆解后作为轮子转速pid（增量式）的目标值（这是内环）。
由有头模式转为无头模式时，进入autoMode时，以及Chassis状态恢复置位时，均会调用一次reset_yaw_bias。
以上重复出现的变量都作为Chassis的属性。请你理解这份prompt，我可能有表达不清楚的地方，请按照原项目风格来，不触碰其他非相关文件。
