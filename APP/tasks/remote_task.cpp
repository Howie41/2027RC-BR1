#include "asserter.hpp"
#include "ibus.hpp"
#include "robot.hpp"
#include "rtos_task.hpp"

#include <array>
#include <cstdint>

// i-Bus 接收任务：从 UART4 收取字节，解析成通道后喂给遥控器消息中心。
rtos::Task<"remote_rx", [] {
    RUNTIME_ASSERT(robot::uart4_port.start_receive_to_idle_dma() == HAL_OK);

    ibus::Parser parser;
    std::array<std::uint8_t, 64> buffer{};
    while (true) {
        const auto length = robot::uart4_port.block_for_received_chunk(buffer);
        std::array<std::uint16_t, ibus::Parser::kChannels> channels{};
        for (std::size_t i = 0; i < length; ++i) {
            if (parser.parse(buffer[i], channels)) {
                robot::remote.feed(channels);
            }
        }
    }
// 【s34 扩容】MemManage 崩溃定位到任务栈区，静态分析无深调用链，翻倍排除溢出分支
}, 2048 * 4, rtos::TaskPriority::Normal> remote_rx_task;
