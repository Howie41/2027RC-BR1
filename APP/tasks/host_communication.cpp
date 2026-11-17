#include "host_communication.hpp"
#include "memory_map.h"
#include "pub_sub.hpp"
#include "rtos_task.hpp"

namespace {

rtos::Publisher<host_comm::VisionPose> g_vision_pose_publisher{robot::host::kVisionPoseTopic};

constexpr auto kCommunicationEntry = [] {
    robot::host::Communication communication({[](const host_comm::VisionPose& message, void*) {
        g_vision_pose_publisher.publish(message);
    }});
    auto& port = UsbPort::instance();
    UsbPort::Packet packet{};
    std::uint32_t stream_epoch{};
    while (true) {
        for (std::size_t i = 0; i < UsbPort::kQueueDepth && port.read(packet); ++i) {
            if (stream_epoch != packet.stream_epoch) {
                communication.reset();
                stream_epoch = packet.stream_epoch;
            }
            communication.process_buffer(std::span{packet.data}.first(packet.size));
        }
        port.pump_tx();
        rtos::delay(1);
    }
};

RAM_D1_BSS
    rtos::Task<"host_communication", kCommunicationEntry, 4096, rtos::TaskPriority::AboveNormal>
        g_host_communication_task;

} // namespace
