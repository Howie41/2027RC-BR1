#pragma once

#include "lockfree_queue.hpp"

#include <array>
#include <atomic>
#include <span>

/// @brief USB CDC 字节流端口；read()、pump_tx() 由同一任务调用，write_async() 可多生产者调用。
class UsbPort {
public:
    static constexpr std::size_t kPacketCapacity = 512;
    static constexpr std::size_t kQueueDepth = 8;

    struct Packet {
        std::array<std::uint8_t, kPacketCapacity> data{};
        std::uint16_t size{};
        std::uint32_t stream_epoch{};
    };

    struct Stats {
        std::uint32_t rx_packets{};
        std::uint32_t rx_dropped{};
        std::uint32_t tx_packets{};
        std::uint32_t tx_rejected{};
        std::uint32_t tx_busy{};
    };

    [[nodiscard]] static UsbPort& instance() noexcept { return m_instance; }

    UsbPort(const UsbPort&) = delete;
    UsbPort& operator=(const UsbPort&) = delete;

    /// @brief 非阻塞读取一个接收块；epoch 变化时须重置上层半帧缓存。
    bool read(Packet& packet) noexcept;

    /// @brief 完整复制并入队一个数据块；空、超长或队列满时返回 false，不部分入队。
    bool write_async(std::span<const std::uint8_t> bytes) noexcept;

    /// @brief 调度器启动后周期调用，USB 就绪且上一笔发送结束后才复用发送缓冲区。
    void pump_tx();

    /// @brief 在重新挂接 USB 接收缓冲区前复制数据，不调用 RTOS API。
    void on_receive_from_isr(std::span<const std::uint8_t> bytes) noexcept;

    /// @brief USB 配置建立或撤销时划分接收会话，不调用 RTOS API。
    void on_session_changed_from_isr() noexcept;

    /// @brief 获取可并发读取的计数快照，各字段可能来自相邻时刻。
    [[nodiscard]] Stats stats() const noexcept;

private:
    UsbPort() = default;
    static UsbPort m_instance;

    using PacketQueue = Algorithm::MpscQueue<Packet, kQueueDepth>;
    PacketQueue m_rx_queue{};
    PacketQueue m_tx_queue{};
    Packet m_tx_staging{};
    bool m_tx_staging_valid{false};
    std::atomic<std::uint32_t> m_stream_epoch{0};
    std::atomic<std::uint32_t> m_rx_packets{0};
    std::atomic<std::uint32_t> m_rx_dropped{0};
    std::atomic<std::uint32_t> m_tx_packets{0};
    std::atomic<std::uint32_t> m_tx_rejected{0};
    std::atomic<std::uint32_t> m_tx_busy{0};
};
