#include "asserter.hpp"
#include "logger.hpp"
#include "main.h"
#include "pid_viewer.hpp"
#include "rtos_task.hpp"
#include "uart_port.hpp"
#include "usart.h"

#include <cstdint>

namespace robot {

DMA_BUFFER_BSS inline UartPortDma<128, 128, 512> uart3_debug_port{huart3, {}};

} // namespace

rtos::Task<"debug_task", [] {
    auto& debug_port = robot::uart3_debug_port;
    PidViewer::bind(debug_port);
    Logger::bind(debug_port);
    RUNTIME_ASSERT(debug_port.start_receive_to_idle_dma() == HAL_OK);

    while (true) {
        PidViewer::periodic(rtos::now_ms());
        Logger::periodic();
        rtos::delay(1);
    }
}, 4096, rtos::TaskPriority::BelowNormal> debug_task;
