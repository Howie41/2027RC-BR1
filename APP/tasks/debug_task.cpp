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

// =====================================================================
// 【临时 bring-up，s34】阀号/电平标定版：双阀同相、1s 翻转一次。
// 原因：catch 双电机未接 CAN2 时 Test 状态进不到 TestRunning
// （TestStarting 中 requestRun 被 isOnline 拒绝 → 直接 Fault），
// 正式阀测试（test_running.cpp）跑不起来；setValve 无守门，
// 不依赖电机在线与生命周期（电平模式定稿，s32）。
// 观察记录（回填用）：
//   阀1=PG3(PUMP_PICK)=上方爪 / 阀2=PG4(VALVE_CLAW)=下方爪【待标定确认】；
//   得电（IO 高电平）时该爪是夹紧还是松开 → 回填 catch_ground_cube.hpp 的
//   CGC_VALVE_CLAMPED（现默认 true=得电夹紧）。
// 【删除条件】双电机在线、正式 test_running 阀测试通过后，本文件还原为 git 版本
// （PidViewer 调参 + 生命周期自测序列，见 git history）。
// 注：标定期 HFSM 会停在 Fault（catch 无电机在线，启动请求被拒），属预期，不影响本任务。
// =====================================================================
rtos::Task<"debug_task", [] {
    PidViewer::bind(robot::uart3_debug_port);
    RUNTIME_ASSERT(robot::uart3_debug_port.start_receive_to_idle_dma() == HAL_OK);

    // 上电先把两阀置为不得电初始态（与 onPrepare 安全初始态一致）
    robot::catch_ground_cube.setValve(1, false);
    robot::catch_ground_cube.setValve(2, false);
    while (true) {
        PidViewer::periodic(rtos::now_ms());
        robot::catch_ground_cube.setValve(1, true);
        robot::catch_ground_cube.setValve(2, true);
        rtos::delay(1000);
        robot::catch_ground_cube.setValve(1, false);
        robot::catch_ground_cube.setValve(2, false);
        rtos::delay(1000);
    }
// 【s34 扩容】MemManage 崩溃定位到任务栈区，静态分析无深调用链，翻倍排除溢出分支
}, 8192, rtos::TaskPriority::Normal> debug_task;
