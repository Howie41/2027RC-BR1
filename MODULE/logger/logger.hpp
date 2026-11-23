#pragma once

#include "main.h"
#include "rtos_task.hpp"
#include "rtos_queue.hpp"
#include "rtos_utility.hpp"
#include "static_string.hpp"
#include "uart_port.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <string_view>
#include <utility>

class Logger {
    static constexpr size_t kMessageCapacity = 100;
    static constexpr size_t kQueueCapacity = 10;
    static constexpr size_t kMaxFrameSize = 128;

    using Port = UartPortDma<kMaxFrameSize, 128, 512>;

public:
    Logger() = delete;

    /// @brief 绑定串口
    static void bind(Port& port) { m_uart_port = &port; }

    /// @brief 将日志入队
    static bool log(const StaticString<kMessageCapacity>& message) {
        if (m_log_queue.handle() == nullptr) return false;

        const auto created_at_tick = rtos::now_tick();
        const auto id = m_next_id.fetch_add(1, std::memory_order_relaxed);
        
        Log log{
            id, 
            created_at_tick, 
            message
        };
        
        return m_log_queue.put(log);
    }

    /// @brief 周期任务
    static void periodic() {
        if (m_uart_port == nullptr || m_log_queue.handle() == nullptr) {
            return;
        }

        const auto log = m_log_queue.peek();
        if (!log) return;

        StaticString<kMaxFrameSize> frame{
            "#%u\t%u\t%s\r\n", log->id, log->created_at_tick, log->message.c_str()
        };
        
        while (m_uart_port->is_transmitting()) { rtos::delay(1); }
        const auto status = m_uart_port->start_transmit_dma(
            std::span<const uint8_t>{
                reinterpret_cast<const uint8_t*>(frame.data()), 
                frame.size()
            }
        );
        if (status == HAL_OK) {
            (void) m_log_queue.get();
        }
    }

private:
    struct Log {
        uint32_t id{0};
        uint32_t created_at_tick{0};
        StaticString<kMessageCapacity> message{};
        Log() : id(0), created_at_tick(rtos::now_tick()), message("") {}
        Log(
            uint32_t id, uint32_t created_at_tick, 
            StaticString<kMessageCapacity> message
        ) : id(id), created_at_tick(created_at_tick), message(message) {}
    };

    inline static rtos::Queue<Log, kQueueCapacity> m_log_queue{};
    inline static std::atomic<uint32_t> m_next_id{0};
    inline static Port* m_uart_port{nullptr};
};
