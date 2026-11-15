# AGENTS.md

面向 Claude Code 及未来 AI 助手的项目速览与维护者工作流约定。精炼、可机读。

## 项目
- STM32H723ZG 嵌入式固件（RoboMaster 类），CubeMX + CMake + Ninja + `arm-none-eabi-gcc`。
- C++23 / C11；`-fno-rtti -fno-exceptions -fno-threadsafe-statics`；FreeRTOS（CMSIS_RTOS_V2）。
- 现代化 C++ / OOP：模板元编程（frame_codec、Hfsm、static_list）、静态多态、零堆分配优先。

## 分层与依赖方向
```
Core / Drivers / Middlewares    # CubeMX 生成 + 第三方，不做侵入式修改
Hfsm  Algorithm  MODULE/rtos_resource_wrapper  MODULE/frame_codec  MODULE/asserter  MODULE/static_string
BSP                             # 板级：can_bus / uart_port / rs485_bus / bsp_dwt / memory_map / pwm
MODULE                          # 设备与算法：motor / pid / subsystem
APP                             # robot（全局聚合）/ robot_hfsm（状态机）/ subsystems / tasks
```
规则（详见 docs/coding_guide.md）：
- APP 可依赖 BSP、MODULE；BSP、MODULE 不得反向依赖 APP。
- 已知历史耦合：BSP 与 MODULE 存在双向 include（can_bus/uart_port → asserter/rtos；motor/pid → can_bus/uart_port）；改动时勿扩大。

## 「未使用 = 待使用」
- 某些文件当前无 include 引用，是提前备好的实现，非死代码，勿删除。
- 例：Algorithm/{double_buffer,function,lockfree_queue,lockfree_ringbuffer}.hpp、BSP/pwm.hpp、MODULE/motor/im6014.hpp、MODULE/rtos_resource_wrapper/pub_sub.hpp。

## 构建与测试
- 配置/构建：`cmake --preset Debug`、`cmake --build --preset Debug`（见 CMakePresets.json）。
- targets：ROBOCON27（可执行）；stm32cubemx（INTERFACE）；STM32_Drivers / FreeRTOS（OBJECT）。
- 备用工具链：cmake/starm-clang.cmake（clang）。
- 主机测试（非 CMake）：`python MODULE/subsystem/host_tests/run_host_tests.py`，用宿主机 g++ + stub 跑 subsystem/hfsm/robot_hfsm 真源码断言。

## 代码风格（维护者偏好）
- 缩进 4 空格、禁用 Tab、对齐工整（Python 风）；以 .clang-format 为准（LLVM base，ColumnLimit 100，Attach，PointerAlignment Left，AccessModifierOffset -4）。
- 命名（见 docs/coding_guide.md）：类型/枚举 大写驼峰；全局变量 `g_`+小写蛇形；常量 `k`+大写驼峰；命名空间 小写蛇形；宏 大写蛇形。
- 零堆分配优先；容器用数组或零堆链表。

## 注释风格（维护者偏好，重要）
- 极简：注释尽可能少，不冗余、不复述代码、不写叙事长文。
- 注释是给 AI 看的：采用「精简中文 Doxygen」——保留 @brief/@param/@return 等结构，但每处只写一句，直接说明“做什么/约束/为什么”；删除“这个函数是用来…”类铺垫。
- 中文；仅在删掉注释会误导时才保留。

## 提交约定
- Conventional Commits：`type(scope)!: 中文描述`，type ∈ feat/fix/refactor/docs/chore/test；`!` 表破坏性变更；PR 合并时带 `(#N)`。
- commit 单元化：一个 commit 只做一件事，描述清晰。

## 工作流
- 提交/烧录前：确认调试开关未误合入；检查警告、未初始化变量、越界/悬空引用/数据竞争/堆分配。
- 尽量不改动第三方库与 CubeMX 生成代码。
