#include "usb_port.hpp"
#include "usb_port_c.h"

#include "FreeRTOS.h"
#include "memory_map.h"
#include "task.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include <algorithm>

extern "C" USBD_HandleTypeDef hUsbDeviceHS;

RAM_D1_BSS UsbPort UsbPort::m_instance{};

bool UsbPort::read(Packet& packet) noexcept {
    return m_rx_queue.TryPop(packet) == Algorithm::QueueError::OK;
}

bool UsbPort::write_async(std::span<const std::uint8_t> bytes) noexcept {
    if (bytes.empty() || bytes.size() > kPacketCapacity) {
        m_tx_rejected.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    Packet packet{};
    packet.size = static_cast<std::uint16_t>(bytes.size());
    std::ranges::copy(bytes, packet.data.begin());
    if (m_tx_queue.TryPush(packet) != Algorithm::QueueError::OK) {
        m_tx_rejected.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    return true;
}

void UsbPort::pump_tx() {
    // USB IRQ 优先级须满足 FreeRTOS 临界区约束；检查到提交期间禁止 USB 撤销配置。
    taskENTER_CRITICAL();
    auto* cdc = static_cast<USBD_CDC_HandleTypeDef*>(hUsbDeviceHS.pClassData);
    if (hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED && cdc != nullptr) {
        if (cdc->TxState != 0U) {
            m_tx_busy.fetch_add(1, std::memory_order_relaxed);
        } else {
            if (!m_tx_staging_valid) {
                m_tx_staging_valid = m_tx_queue.TryPop(m_tx_staging) == Algorithm::QueueError::OK;
            }
            if (m_tx_staging_valid) {
                const auto result = CDC_Transmit_HS(m_tx_staging.data.data(), m_tx_staging.size);
                if (result == USBD_OK) {
                    m_tx_staging_valid = false;
                    m_tx_packets.fetch_add(1, std::memory_order_relaxed);
                }
            }
        }
    }
    taskEXIT_CRITICAL();
}

void UsbPort::on_receive_from_isr(std::span<const std::uint8_t> bytes) noexcept {
    if (bytes.empty())
        return;
    if (bytes.size() <= kPacketCapacity) {
        Packet packet{};
        packet.size = static_cast<std::uint16_t>(bytes.size());
        packet.stream_epoch = m_stream_epoch.load(std::memory_order_relaxed);
        std::ranges::copy(bytes, packet.data.begin());
        if (m_rx_queue.TryPush(packet) == Algorithm::QueueError::OK) {
            m_rx_packets.fetch_add(1, std::memory_order_relaxed);
            return;
        }
    }
    m_rx_dropped.fetch_add(1, std::memory_order_relaxed);
    on_session_changed_from_isr();
}

void UsbPort::on_session_changed_from_isr() noexcept {
    m_stream_epoch.fetch_add(1, std::memory_order_relaxed);
}

UsbPort::Stats UsbPort::stats() const noexcept {
    return {
        m_rx_packets.load(std::memory_order_relaxed), m_rx_dropped.load(std::memory_order_relaxed),
        m_tx_packets.load(std::memory_order_relaxed), m_tx_rejected.load(std::memory_order_relaxed),
        m_tx_busy.load(std::memory_order_relaxed)};
}

extern "C" void usb_port_on_receive_from_isr(const uint8_t* data, uint32_t size) {
    if (data != nullptr)
        UsbPort::instance().on_receive_from_isr({data, size});
}

extern "C" void usb_port_on_session_changed_from_isr(void) {
    UsbPort::instance().on_session_changed_from_isr();
}
