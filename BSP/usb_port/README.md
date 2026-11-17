# UsbPort

使用当前 CubeMX 生成的 `hUsbDeviceHS` / `CDC_Transmit_HS`。端口对象位于 D1 SRAM，RX / TX 各使用现有 `Algorithm::MpscQueue` 的 8 个槽，每槽最多 512 字节，无动态分配。

- `UsbPort::instance().read(packet)`：由唯一通信任务消费，返回字节数与接收流 epoch。
- `write_async(bytes)`：完整复制一个数据块，空、超过 512 字节或队列满时拒绝全部数据；可多生产者调用。
- `pump_tx()`：由同一通信任务周期调用，须在调度器启动后执行。
- `stats()`：返回原子计数的快照，`tx_packets` 表示提交成功次数，`tx_busy` 表示推进时观察到忙碌的次数。

发送前先检查 CDC `TxState`，空闲后才从队列复制到暂存区。提交成功后暂存区仍由 USB 使用，直到 `TxState` 清零才允许覆盖。检查、复制与提交位于 FreeRTOS 临界区内，避免 USB 中断在检查后撤销配置。当前 OTG_HS IRQ 优先级为 5，满足此约束；更改中断优先级时须重新确认。

上层应在重连或接收溢出后，通过接收块的 epoch 重置流式解析器。端口不提供应用确认或重传，拔线可能丢失已提交的数据。

CubeMX 接入仅修改用户代码区：

| 文件 / 回调 | 接入 |
| --- | --- |
| `usbd_cdc_if.c` Includes | 引入 `usb_port_c.h` |
| `CDC_Init_HS` / `CDC_DeInit_HS` | 更新接收会话 epoch |
| `CDC_Receive_HS` | 在重新挂接接收缓冲区前复制数据 |
| `CDC_Transmit_HS` | 配置状态与空句柄检查 |
| `main.c` USER CODE 2 | RTOS 对象创建前调用 `MX_USB_DEVICE_Init()` |
| `usb_device.c` 初始化前后处理 | 幂等守卫，兼容默认任务再次调用 |

由 APP 每个 tick 推进发送，因此无需额外信号量或发送完成回调。CubeMX 再生成代码后需确认用户区保留；不要从其他任务直接调用 `CDC_Transmit_HS`，以免绕过缓冲区寿命和单消费者约束。
