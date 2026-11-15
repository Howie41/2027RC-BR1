#pragma once

#include <cstdint>

// 只保存静态字符串指针，不分配内存；原因必须使用字符串字面量或静态字符串。
struct FaultInfo {
    const char* reason{nullptr};
    const char* file{nullptr};
    std::uint32_t line{0};

    // 同一次失败保留首个原因，避免关闭输出或基类兜底覆盖具体来源。
    void record(const char* message, const char* filename, std::uint32_t line_number) noexcept {
        if (reason != nullptr) return;
        reason = message;
        file = filename;
        line = line_number;
    }
};

// 仅记录现场，调用方仍须按原流程返回失败或请求状态转换。
#define RECORD_FAULT(destination, message)                              \
    do {                                                               \
        (destination).record((message), __FILE_NAME__, __LINE__);        \
    } while (false)
