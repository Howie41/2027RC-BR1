#include "asserter.hpp"
#include "main.h"
#include "pid_viewer.hpp"
#include "rtos_task.hpp"
#include "uart_port.hpp"
#include "usart.h"

#include <cstdint>

//调试
#include "catch_ground_cube_bridge.hpp"
#include "robot.hpp"
//

namespace robot {

DMA_BUFFER_BSS inline UartPortDma<64, 128, 512> uart3_debug_port(huart3, {});

} // namespace

rtos::Task<"debug_task", [] {
    PidViewer::bind(robot::uart3_debug_port);
    robot::catch_ground_cube.requestPrepare();//测试
    while (!robot::catch_ground_cube.requestStandby()) rtos::delay(10);//测试
    while (!robot::catch_ground_cube.requestRun())     rtos::delay(10);//测试
    RUNTIME_ASSERT(robot::uart3_debug_port.start_receive_to_idle_dma() == HAL_OK);

    while (true) {
        PidViewer::periodic(rtos::now_ms());
        cube_dbg::valvePulse(cube_dbg::VALVE_1);   // 阀1(PG3)：1秒切换一次开/合;
        rtos::delay(1000);
    }
}, 4096, rtos::TaskPriority::BelowNormal> debug_task;
