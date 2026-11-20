#include "asserter.hpp"
#include "parser.hpp"
#include "robot.hpp"
#include "rtos_task.hpp"

#include <array>
#include <cstdint>

// i-Bus 接收任务：从 UART4 收取字节，解析成通道后喂给遥控器消息中心，并处理恢复命令。
rtos::Task<"remote_rx", [] {
    RUNTIME_ASSERT(robot::uart4_port.start_receive_to_idle_dma() == HAL_OK);

    remote::Parser parser;
    std::array<std::uint8_t, 64> buffer{};
    bool recovery_pressed = false;
    while (true) {
        const auto length = robot::uart4_port.block_for_received_chunk(buffer);
        std::array<std::uint16_t, remote::Parser::kChannels> channels{};
        for (std::size_t i = 0; i < length; ++i) {
            if (parser.parse(buffer[i], channels)) {
                robot::remote.feed(channels);

                // 通道 9（CH9，0 基下标 8）拨到 2000 = 请求故障恢复；上升沿触发一次。
                const bool recovery_now = channels[8] == 2000;
                if (recovery_now && !recovery_pressed) {
                    robot::robot_context.recovery_requested.store(true);
                }
                recovery_pressed = recovery_now;
            }
        }
    }
}, 1024 * 4, rtos::TaskPriority::High> remote_rx_task;
