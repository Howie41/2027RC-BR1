#include "usb_port.hpp"
#include "usb_port_c.h"
#include "usbd_cdc_if.h"

#include <algorithm>
#include <cassert>

extern "C" {
USBD_HandleTypeDef hUsbDeviceHS{};
}

namespace {
USBD_CDC_HandleTypeDef g_cdc{};
int g_critical_depth{};
std::uint8_t g_result{USBD_OK};
bool g_complete_immediately{};
std::uint8_t* g_active_data{};
std::uint16_t g_active_size{};
std::size_t g_submitted{};
} // namespace

void test_enter_critical() {
    ++g_critical_depth;
}
void test_exit_critical() {
    assert(--g_critical_depth == 0);
}

extern "C" std::uint8_t CDC_Transmit_HS(std::uint8_t* data, std::uint16_t size) {
    assert(g_critical_depth == 1);
    assert(g_cdc.TxState == 0);
    if (g_result != USBD_OK)
        return g_result;
    g_active_data = data;
    g_active_size = size;
    ++g_submitted;
    g_cdc.TxState = g_complete_immediately ? 0 : 1;
    return USBD_OK;
}

int main() {
    auto& port = UsbPort::instance();
    const std::array<std::uint8_t, 4> first{1, 2, 3, 4};
    const std::array<std::uint8_t, 4> second{5, 6, 7, 8};

    assert(port.write_async(first));
    port.pump_tx();
    assert(g_submitted == 0);
    hUsbDeviceHS.dev_state = USBD_STATE_CONFIGURED;
    port.pump_tx(); // 尚无 CDC 配置。
    assert(g_submitted == 0);
    hUsbDeviceHS.pClassData = &g_cdc;
    port.pump_tx();
    assert(g_submitted == 1 && g_active_size == first.size());
    assert(std::equal(first.begin(), first.end(), g_active_data));
    assert(port.write_async(second));
    for (int i = 0; i < 5; ++i)
        port.pump_tx();
    assert(g_submitted == 1);
    assert(std::equal(first.begin(), first.end(), g_active_data));
    g_cdc.TxState = 0;
    port.pump_tx();
    assert(g_submitted == 2);
    assert(std::equal(second.begin(), second.end(), g_active_data));

    g_cdc.TxState = 0;
    g_result = USBD_BUSY;
    assert(port.write_async(first));
    port.pump_tx();
    assert(g_submitted == 2);
    g_result = USBD_FAIL;
    port.pump_tx();
    assert(g_submitted == 2);
    g_result = USBD_OK;
    g_complete_immediately = true;
    port.pump_tx();
    assert(g_submitted == 3);
    assert(port.write_async(second));
    port.pump_tx();
    assert(g_submitted == 4); // 提交过程中完成也不会锁死。

    for (std::size_t i = 0; i < UsbPort::kQueueDepth; ++i)
        assert(port.write_async(first));
    std::array<std::uint8_t, 72> frame{};
    frame.fill(0xAA);
    assert(!port.write_async(frame));
    for (std::size_t i = 0; i < UsbPort::kQueueDepth; ++i)
        port.pump_tx();
    const auto submitted = g_submitted;
    port.pump_tx();
    assert(g_submitted == submitted);
    assert(!port.write_async({}));
    std::array<std::uint8_t, UsbPort::kPacketCapacity + 1> oversize{};
    assert(!port.write_async(oversize));
    assert(port.write_async(frame));
    port.pump_tx();
    assert(g_active_size == frame.size());
    assert(std::equal(frame.begin(), frame.end(), g_active_data));

    // 拔插后无需等待上一会话的完成回调。
    g_complete_immediately = false;
    assert(port.write_async(first));
    port.pump_tx();
    assert(port.write_async(second));
    hUsbDeviceHS.dev_state = 0;
    hUsbDeviceHS.pClassData = nullptr;
    port.pump_tx();
    g_cdc.TxState = 0;
    hUsbDeviceHS.pClassData = &g_cdc;
    hUsbDeviceHS.dev_state = USBD_STATE_CONFIGURED;
    port.pump_tx();
    assert(std::equal(second.begin(), second.end(), g_active_data));

    UsbPort::Packet packet{};
    usb_port_on_receive_from_isr(first.data(), first.size());
    assert(port.read(packet));
    const auto initial_epoch = packet.stream_epoch;
    assert(std::ranges::equal(std::span{packet.data}.first(packet.size), first));
    for (std::size_t i = 0; i < UsbPort::kQueueDepth; ++i)
        usb_port_on_receive_from_isr(first.data(), first.size());
    usb_port_on_receive_from_isr(first.data(), first.size()); // 溢出划分字节流。
    while (port.read(packet))
        assert(packet.stream_epoch == initial_epoch);
    usb_port_on_receive_from_isr(second.data(), second.size());
    assert(port.read(packet) && packet.stream_epoch != initial_epoch);
    const auto overflow_epoch = packet.stream_epoch;
    usb_port_on_session_changed_from_isr();
    usb_port_on_receive_from_isr(first.data(), first.size());
    assert(port.read(packet) && packet.stream_epoch != overflow_epoch);
    usb_port_on_receive_from_isr(oversize.data(), oversize.size());
    assert(!port.read(packet));
    usb_port_on_receive_from_isr(nullptr, 1);
    assert(!port.read(packet));
    const auto stats = port.stats();
    assert(stats.rx_dropped == 2 && stats.tx_rejected == 3);
    assert(stats.tx_packets == g_submitted);
    assert(g_critical_depth == 0);
}
