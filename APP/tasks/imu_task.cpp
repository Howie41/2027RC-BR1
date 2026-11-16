#include "hwt901b.hpp"
#include "robot.hpp"
#include "rtos_task.hpp"

#include <array>
#include <cstdint>

// IMU 接收任务：从 UART8 收取字节，解析后在角度帧到达时更新里程计。
rtos::Task<"imu_rx", [] {
    RUNTIME_ASSERT(robot::uart8_port.start_receive_to_idle_dma() == HAL_OK);

    imu::Parser parser;
    std::array<std::uint8_t, 64> buffer{};
    while (true) {
        const auto length = robot::uart8_port.block_for_received_chunk(buffer);
        for (std::size_t i = 0; i < length; ++i) {
            if (parser.parse(buffer[i], robot::imu_data) == imu::FrameType::Angle) {
                robot::odometry.update(robot::imu_data, rtos::now_ms());
            }
        }
    }
}, 1024 * 4, rtos::TaskPriority::High> imu_rx_task;
