#pragma once

// DTCMRAM 不可由 DMA1/DMA2 访问；D2 SRAM 用于 DMA 缓冲区。
#if defined(__GNUC__)

// 使用含路径的文件名和展开行号；同一行只能定义一个自动分段实体。
#define U_MEMORY_MAP_STRINGIFY_IMPL(value) #value
#define U_MEMORY_MAP_STRINGIFY(value) U_MEMORY_MAP_STRINGIFY_IMPL(value)
#define U_MEMORY_MAP_LOCATION __FILE__ "." U_MEMORY_MAP_STRINGIFY(__LINE__)

// D2 SRAM：DATA 从 Flash 复制初值，BSS 在启动时清零；DMA 缓冲区按 32 字节对齐。
#define DMA_BUFFER_DATA \
    __attribute__((section(".dma_buffer_data." U_MEMORY_MAP_LOCATION), aligned(32)))
#define DMA_BUFFER_BSS \
    __attribute__((section(".dma_buffer_bss." U_MEMORY_MAP_LOCATION), aligned(32)))

// D1 AXI SRAM：DATA 从 Flash 复制初值，BSS 在启动时清零。
#define RAM_D1_DATA \
    __attribute__((section(".ram_d1_data." U_MEMORY_MAP_LOCATION), aligned(8)))
#define RAM_D1_BSS \
    __attribute__((section(".ram_d1_bss." U_MEMORY_MAP_LOCATION), aligned(8)))

// 保留旧名称供现有的零初始化对象使用。
#define DMA_BUFFER DMA_BUFFER_BSS
#define RAM_D1 RAM_D1_BSS

#define ITCM_FUNCTION __attribute__((section(".itcm_text." U_MEMORY_MAP_LOCATION), noinline))

#else
#error "Unsupported compiler"
#endif
